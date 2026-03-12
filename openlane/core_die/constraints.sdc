# Core die SDC constraints
create_clock -name clk -period 3.33 [get_ports clk]
set_clock_uncertainty 0.15 [get_clocks clk]
set_clock_transition 0.10 [get_clocks clk]

# Input/output delays relative to clock
set_input_delay  -clock clk -max 0.5 [all_inputs]
set_input_delay  -clock clk -min 0.1 [all_inputs]
set_output_delay -clock clk -max 0.5 [all_outputs]
set_output_delay -clock clk -min 0.1 [all_outputs]

# Async resets
set_false_path -from [get_ports rst_n]

# NoC TX/RX ports — multicycle where pkt takes 2 cycles
set_multicycle_path 2 -setup -from [get_ports noc_tx_*] -to [get_ports noc_rx_*]
set_multicycle_path 1 -hold  -from [get_ports noc_tx_*] -to [get_ports noc_rx_*]
