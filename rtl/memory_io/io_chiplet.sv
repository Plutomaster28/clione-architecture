// ============================================================================
// Clione Processor — I/O Chiplet
// PCIe Gen5 x16, DMA Engine (32-channel), GPIO, UART/SPI/I2C, Ethernet
// ============================================================================
`include "clione_pkg.sv"

module io_chiplet
  import clione_pkg::*;
#(
  parameter logic [4:0] CHIPLET_NODE_ID = 5'h1E,
  parameter int         DMA_CHANNELS    = 32,
  parameter int         PCIE_LANES      = 16
)(
  input  logic                            clk,
  input  logic                            rst_n,

  // NoC interface (to/from core die and memory)
  input  noc_pkt_t                        rx_pkt,
  input  logic                            rx_valid,
  output logic                            rx_ready,

  output noc_pkt_t                        tx_pkt,
  output logic                            tx_valid,
  input  logic                            tx_ready,

  // PCIe SerDes (abstracted)
  input  logic [PCIE_LANES-1:0]           pcie_rx_p,
  input  logic [PCIE_LANES-1:0]           pcie_rx_n,
  output logic [PCIE_LANES-1:0]           pcie_tx_p,
  output logic [PCIE_LANES-1:0]           pcie_tx_n,
  output logic                            pcie_clkreq_n,
  input  logic                            pcie_refclk,

  // Ethernet SGMII (1Gbps) / USXGMII (10Gbps)
  input  logic [3:0]                      eth_rx_p,
  input  logic [3:0]                      eth_rx_n,
  output logic [3:0]                      eth_tx_p,
  output logic [3:0]                      eth_tx_n,

  // UART
  input  logic                            uart_rxd,
  output logic                            uart_txd,

  // SPI (master)
  output logic                            spi_sclk,
  output logic                            spi_mosi,
  input  logic                            spi_miso,
  output logic [3:0]                      spi_cs_n,

  // I2C
  inout  logic                            i2c_sda,
  inout  logic                            i2c_scl,

  // GPIO
  inout  logic [31:0]                     gpio,

  // Interrupt output to core die
  output logic [15:0]                     irq
);

  // --------------------------------------------------------------------------
  // MMIO Register Map (accessed via NoC NOC_LOAD_REQ/NOC_STORE_REQ)
  // --------------------------------------------------------------------------
  localparam logic [15:0] REG_DMA_SRC   [DMA_CHANNELS] = '{default: 16'h0};
  localparam logic [15:0] REG_DMA_DST   = 16'h0200;
  localparam logic [15:0] REG_DMA_LEN   = 16'h0400;
  localparam logic [15:0] REG_DMA_CFG   = 16'h0600; // [0]=start, [1]=irq_en, [2]=dir (0=rd,1=wr)
  localparam logic [15:0] REG_IRQ_STAT  = 16'h0800;
  localparam logic [15:0] REG_IRQ_MASK  = 16'h0900;
  localparam logic [15:0] REG_PCIE_STAT = 16'h1000;
  localparam logic [15:0] REG_GPIO_DIR  = 16'h2000;
  localparam logic [15:0] REG_GPIO_OUT  = 16'h2008;
  localparam logic [15:0] REG_GPIO_IN   = 16'h2010;

  // --------------------------------------------------------------------------
  // Registers
  // --------------------------------------------------------------------------
  logic [63:0] dma_src      [DMA_CHANNELS-1:0];
  logic [63:0] dma_dst      [DMA_CHANNELS-1:0];
  logic [31:0] dma_len      [DMA_CHANNELS-1:0];
  logic [7:0]  dma_cfg      [DMA_CHANNELS-1:0];  // [0]=start, [1]=busy, [2]=irq_en
  logic [63:0] dma_cur_addr [DMA_CHANNELS-1:0];
  logic [31:0] dma_done_cnt [DMA_CHANNELS-1:0];

  logic [15:0] irq_status;
  logic [15:0] irq_mask;
  logic [31:0] gpio_dir;
  logic [31:0] gpio_out_reg;
  logic [31:0] gpio_in_sync;

  // --------------------------------------------------------------------------
  // PCIe link state (simplified)
  // --------------------------------------------------------------------------
  typedef enum logic [1:0] {
    PCIE_DETECT     = 2'b00,
    PCIE_POLLING    = 2'b01,
    PCIE_L0         = 2'b10,    // Active
    PCIE_L1         = 2'b11     // Low-power
  } pcie_state_e;

  pcie_state_e pcie_state;
  logic [7:0]  pcie_ltssm_timer;
  logic        pcie_link_up;

  // PCIe TX/RX data FIFOs
  logic [255:0] pcie_tx_fifo [63:0];  // 256-bit data per entry
  logic [255:0] pcie_rx_fifo [63:0];
  logic [5:0]   pcie_tx_head, pcie_tx_tail;
  logic [5:0]   pcie_rx_head, pcie_rx_tail;

  // --------------------------------------------------------------------------
  // UART
  // --------------------------------------------------------------------------
  localparam int UART_BAUD_DIV = 868; // 100MHz / 115200
  logic [9:0]  uart_baud_cnt;
  logic [3:0]  uart_bit_cnt;
  logic [9:0]  uart_shift_rx;
  logic [9:0]  uart_shift_tx;
  logic        uart_tx_busy;
  logic [7:0]  uart_rx_buf [15:0];
  logic [7:0]  uart_tx_buf [15:0];
  logic [3:0]  uart_rx_head, uart_rx_tail;
  logic [3:0]  uart_tx_head, uart_tx_tail;

  // --------------------------------------------------------------------------
  // SPI Master
  // --------------------------------------------------------------------------
  localparam int SPI_CLK_DIV = 4; // SCLK = clk / 8
  logic [2:0]  spi_clk_cnt;
  logic [6:0]  spi_bit_cnt;
  logic [63:0] spi_shift_reg;
  logic        spi_busy;
  logic [63:0] spi_rx_data;

  // --------------------------------------------------------------------------
  // I2C Tri-state
  // --------------------------------------------------------------------------
  logic i2c_sda_oe, i2c_scl_oe;
  logic i2c_sda_out, i2c_scl_out;
  assign i2c_sda = i2c_sda_oe ? i2c_sda_out : 1'bZ;
  assign i2c_scl = i2c_scl_oe ? i2c_scl_out : 1'bZ;

  // --------------------------------------------------------------------------
  // GPIO
  // --------------------------------------------------------------------------
  genvar gi;
  generate
    for (gi = 0; gi < 32; gi++) begin : g_gpio
      assign gpio[gi] = gpio_dir[gi] ? gpio_out_reg[gi] : 1'bZ;
    end
  endgenerate

  // --------------------------------------------------------------------------
  // Main Logic
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_ready  <= 1'b1;
      tx_valid  <= 1'b0;
      irq       <= '0;
      irq_status<= '0;
      irq_mask  <= '0;
      pcie_state<= PCIE_DETECT;
      pcie_link_up <= 1'b0;
      pcie_ltssm_timer <= 8'hFF;
      uart_baud_cnt <= '0; uart_bit_cnt <= '0; uart_tx_busy <= 1'b0;
      spi_clk_cnt <= '0; spi_busy <= 1'b0;
      spi_sclk <= 1'b0; spi_mosi <= 1'b0;
      gpio_dir  <= '0; gpio_out_reg <= '0;
      pcie_tx_head <= '0; pcie_tx_tail <= '0;
      pcie_rx_head <= '0; pcie_rx_tail <= '0;
      uart_tx_head <= '0; uart_tx_tail <= '0;
      uart_rx_head <= '0; uart_rx_tail <= '0;
      uart_txd <= 1'b1;
      for (int d = 0; d < DMA_CHANNELS; d++) begin
        dma_src[d]   <= '0; dma_dst[d]   <= '0;
        dma_len[d]   <= '0; dma_cfg[d]   <= '0;
        dma_done_cnt[d] <= '0;
        dma_cur_addr[d] <= '0;
      end
    end else begin
      tx_valid <= 1'b0;
      rx_ready <= 1'b1;

      // ---- PCIe LTSSM (simplified) ----
      case (pcie_state)
        PCIE_DETECT: begin
          if (|pcie_rx_p) begin
            pcie_state <= PCIE_POLLING;
            pcie_ltssm_timer <= 8'd100;
          end
        end
        PCIE_POLLING: begin
          if (pcie_ltssm_timer > 0)
            pcie_ltssm_timer <= pcie_ltssm_timer - 8'd1;
          else begin
            pcie_state    <= PCIE_L0;
            pcie_link_up  <= 1'b1;
            irq_status[0] <= 1'b1; // PCIe link up IRQ
          end
        end
        PCIE_L0: begin
          // Active: echo RX to DMA RX FIFO
          pcie_rx_fifo[pcie_rx_tail] <= {pcie_rx_p, 240'b0};
          pcie_rx_tail <= pcie_rx_tail + 6'd1;
        end
        PCIE_L1: begin
          if (|pcie_rx_p) pcie_state <= PCIE_L0;
        end
      endcase

      // PCIe TX: drain FIFO
      if (pcie_tx_head != pcie_tx_tail) begin
        pcie_tx_p <= PCIE_LANES'(pcie_tx_fifo[pcie_tx_head][PCIE_LANES-1:0]);
        pcie_tx_n <= ~PCIE_LANES'(pcie_tx_fifo[pcie_tx_head][PCIE_LANES-1:0]);
        pcie_tx_head <= pcie_tx_head + 6'd1;
      end else begin
        pcie_tx_p <= '0; pcie_tx_n <= '1;
      end

      // ---- UART RX ----
      uart_baud_cnt <= uart_baud_cnt + 10'd1;
      if (uart_baud_cnt == 10'(UART_BAUD_DIV)) begin
        uart_baud_cnt <= '0;
        uart_shift_rx <= {uart_rxd, uart_shift_rx[9:1]};
        uart_bit_cnt  <= uart_bit_cnt + 4'd1;
        if (uart_bit_cnt == 4'd9) begin
          uart_bit_cnt          <= '0;
          uart_rx_buf[uart_rx_tail] <= uart_shift_rx[8:1];
          uart_rx_tail          <= uart_rx_tail + 4'd1;
          irq_status[2]         <= 1'b1; // UART RX ready
        end
      end

      // ---- UART TX ----
      if (!uart_tx_busy && uart_tx_head != uart_tx_tail) begin
        uart_shift_tx <= {1'b1, uart_tx_buf[uart_tx_head], 1'b0}; // stop|data|start
        uart_tx_busy  <= 1'b1;
        uart_tx_head  <= uart_tx_head + 4'd1;
      end else if (uart_tx_busy) begin
        if (uart_baud_cnt == '0) begin
          uart_txd      <= uart_shift_tx[0];
          uart_shift_tx <= {1'b1, uart_shift_tx[9:1]};
          if (uart_shift_tx[9:1] == 9'b1_1111_1111)
            uart_tx_busy <= 1'b0;
        end
      end else begin
        uart_txd <= 1'b1; // IDLE
      end

      // ---- GPIO sync ----
      gpio_in_sync <= gpio;

      // ---- DMA engine ----
      for (int ch = 0; ch < DMA_CHANNELS; ch++) begin
        if (dma_cfg[ch][0] && dma_cfg[ch][1]) begin // start && busy
          // Issue a NoC LOAD request for each 64-byte block
          if (dma_done_cnt[ch] < dma_len[ch]) begin
            tx_pkt.pkt_type <= NOC_LOAD_REQ;
            tx_pkt.dst_node <= 5'h1F; // To memory controller
            tx_pkt.src_node <= CHIPLET_NODE_ID;
            tx_pkt.addr     <= dma_cur_addr[ch];
            tx_pkt.rob_idx  <= ROB_PTR_WIDTH'(ch);
            tx_pkt.tid      <= TID_WIDTH'(0);
            tx_pkt.valid    <= 1'b1;
            tx_valid        <= 1'b1;
            dma_cur_addr[ch] <= dma_cur_addr[ch] + 64'(LINE_BYTES);
            dma_done_cnt[ch] <= dma_done_cnt[ch] + 32'd1;
          end else begin
            dma_cfg[ch][1]   <= 1'b0; // Clear busy
            dma_cfg[ch][0]   <= 1'b0; // Clear start
            dma_done_cnt[ch] <= '0;
            if (dma_cfg[ch][2]) irq_status[4+ch[3:0]] <= 1'b1; // IRQ if enabled
          end
        end
      end

      // ---- NoC MMIO access handler ----
      if (rx_valid) begin
        automatic logic [15:0] reg_addr = rx_pkt.addr[15:0];
        automatic logic [63:0] rdata = '0;
        if (rx_pkt.pkt_type == NOC_STORE_REQ) begin
          case (reg_addr[15:8])
            8'h00: begin // DMA SRC
              automatic int ch = int'(reg_addr[7:3]);
              if (ch < DMA_CHANNELS) dma_src[ch] <= rx_pkt.data[63:0];
            end
            8'h02: begin // DMA DST
              automatic int ch = int'(reg_addr[7:3]);
              if (ch < DMA_CHANNELS) dma_dst[ch] <= rx_pkt.data[63:0];
            end
            8'h04: begin // DMA LEN
              automatic int ch = int'(reg_addr[7:3]);
              if (ch < DMA_CHANNELS) dma_len[ch] <= rx_pkt.data[31:0];
            end
            8'h06: begin // DMA CFG
              automatic int ch = int'(reg_addr[7:3]);
              if (ch < DMA_CHANNELS) begin
                dma_cfg[ch]      <= rx_pkt.data[7:0];
                dma_cur_addr[ch] <= dma_src[ch];
                dma_done_cnt[ch] <= '0;
              end
            end
            8'h09: irq_mask <= rx_pkt.data[15:0];
            8'h20: gpio_dir <= rx_pkt.data[31:0];
            8'h20: gpio_out_reg <= rx_pkt.data[31:0]; // REG_GPIO_OUT
            default: ;
          endcase
        end else if (rx_pkt.pkt_type == NOC_LOAD_REQ) begin
          case (reg_addr[15:8])
            8'h08: rdata = {48'b0, irq_status};
            8'h09: rdata = {48'b0, irq_mask};
            8'h10: rdata = {62'b0, pcie_link_up, 1'b0};
            8'h20: rdata = {32'b0, gpio_in_sync};
            default: rdata = '0;
          endcase
          tx_pkt.pkt_type <= NOC_LOAD_RESP;
          tx_pkt.dst_node <= rx_pkt.src_node;
          tx_pkt.src_node <= CHIPLET_NODE_ID;
          tx_pkt.addr     <= rx_pkt.addr;
          tx_pkt.data     <= {8{rdata}};
          tx_pkt.rob_idx  <= rx_pkt.rob_idx;
          tx_pkt.tid      <= rx_pkt.tid;
          tx_pkt.valid    <= 1'b1;
          tx_valid        <= 1'b1;
        end
      end

      // ---- IRQ generation ----
      irq <= irq_status & irq_mask;
    end
  end

  // pcie_clkreq_n
  assign pcie_clkreq_n = ~pcie_link_up;

endmodule : io_chiplet
