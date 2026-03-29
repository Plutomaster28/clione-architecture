; =============================================================================
;  SeaBird ISA — General-Purpose Hardware Initialization & Boot Stub
;  SeaBird ISA Reference Manual v1.1  (Meisei / Naoki Saito)
;
;  Execution begins in Clownfish (16-bit real) mode.
;  The stub detects the target platform's capability registers, elevates
;  through Tetra (32-bit protected) mode, and optionally continues to
;  Dragonet (64-bit long) mode before handing off to a higher-level loader.
;
;  This file is intentionally platform-agnostic:  hardware-specific I/O
;  addresses are #define'd at the top so the same source assembles for any
;  board that carries a SeaBird-family processor.
;
;  Assembler: SeaBird Hybrid Assembler (or compatible)
;  Entry point: physical address 0xCAFE0000  (vector-reset address per §2.1)
; =============================================================================

; ---------------------------------------------------------------------------
;  Board-level tunables  — edit these for your target hardware
; ---------------------------------------------------------------------------
.equ UART_BASE,         0x3F8       ; base I/O address of debug serial port
.equ PIC_MASTER_CMD,    0xA0        ; master PIC command register
.equ PIC_MASTER_DATA,   0xA1        ; master PIC data register
.equ PIC_SLAVE_CMD,     0xA8        ; slave PIC command register
.equ PIC_SLAVE_DATA,    0xA9        ; slave PIC data register
.equ PIC_EOI,           0x20        ; end-of-interrupt byte
.equ IRQ_MASTER_BASE,   0x20        ; remap master PIC to INT 0x20–0x27
.equ IRQ_SLAVE_BASE,    0x28        ; remap slave  PIC to INT 0x28–0x2F
.equ CPUID_REG,         0xFF00      ; hypothetical capability/ID MMIO register
.equ CR0_PE,            0x00000001  ; CR0.PE  — protected mode enable
.equ CR0_PG,            0x80000000  ; CR0.PG  — paging enable
.equ CR0_WP,            0x00010000  ; CR0.WP  — write-protect in kernel
.equ CR4_PAE,           0x00000020  ; CR4.PAE — physical address extension
.equ PML4_BASE,         0x1000      ; physical address where we build PML4
.equ STACK_TOP_16,      0x7C00      ; 16-bit stack (below conventional memory)
.equ STACK_TOP_32,      0x9FC00     ; 32-bit stack top (just below EBDA)
.equ STACK_TOP_64,      0x00200000  ; 64-bit stack top (2 MB mark)
.equ GDT_BASE,          0x0500      ; GDT lives at 0x0500 (below BIOS data)
.equ IDT_BASE,          0x0600      ; IDT lives at 0x0600
.equ PAGE_PRESENT,      0x003       ; P | R/W  — §17.2
.equ PAGE_HUGE,         0x083       ; P | R/W | PS (2 MB huge page)

; ---------------------------------------------------------------------------
;  Capability flags returned by our internal CPUID helper (RDCR CR4 analogue)
;  Bit 0 = supports 32-bit Tetra mode
;  Bit 1 = supports 64-bit Dragonet mode
;  Bit 2 = supports 128-bit Droplet/hypervisor mode
;  Bit 3 = has hardware FPU
;  Bit 4 = has SIMD / vector unit
; ---------------------------------------------------------------------------
.equ CAP_TETRA,         0x01
.equ CAP_DRAGONET,      0x02
.equ CAP_DROPLET,       0x04
.equ CAP_FPU,           0x08
.equ CAP_VECTOR,        0x10

; ---------------------------------------------------------------------------
;  Optimization: start conservative, widen once in 64-bit mode
; ---------------------------------------------------------------------------
.cpu    seabird_generic
.arch   SYS
.mopt   0                           ; no micro-fusion during init (debug-safe)
.mode   CLOWNFISH                   ; §11.5 — assemble 16-bit code from here


; =============================================================================
;  SECTION: Reset Vector  (physical 0xCAFE0000, §2.1 "Vector Reset: 0xCAFE")
; =============================================================================
.org    0xCAFE0000
.text

reset_vector:
    ; The very first instruction executed after power-on reset.
    ; IP is 16-bit; all registers are in an undefined state.
    ; SeaBird spec says vector table entry is at 0xCAFE — we land here.

    CLI                             ; §10.7 — disable interrupts immediately
    MOVI    SP, #STACK_TOP_16       ; set up a usable 16-bit stack
    MOVI    BP, #0                  ; clear base pointer
    MOVI    AX, #0                  ; zero the primary accumulator
    MOVI    BX, #0
    MOVI    CX, #0
    MOVI    DX, #0

    ; Clear direction flag and any stray overflow by pushing/popping FLAGS
    PUSHF
    POP     AX
    AND     AX, #0xF73A             ; clear DF(10), TF(8), IF(9), NT(14)
    PUSH    AX
    POPF

    JMP     clownfish_init          ; jump past data, into real init logic


; =============================================================================
;  SECTION: GDT  (built in 16-bit mode before we enter Tetra)
;  Flat 4 GB model: null, kernel code (0–4G), kernel data (0–4G)
; =============================================================================
.align  8
.org    GDT_BASE

gdt_start:
    ; Null descriptor
    .qword  0x0000000000000000

gdt_kernel_code_32:
    ; Base=0, Limit=0xFFFFF, G=1(4K gran), D/B=1(32-bit), P=1, DPL=0, S=1,
    ; Type=0xA (execute/read)
    .dword  0x0000FFFF              ; Limit 15:0 = 0xFFFF, Base 15:0 = 0x0000
    .dword  0x00CF9A00              ; Base 23:16=0x00, Flags=0xCF, Type/DPL=0x9A

gdt_kernel_data_32:
    .dword  0x0000FFFF
    .dword  0x00CF9200              ; Type=0x92 (data, read/write)

gdt_kernel_code_64:
    ; Long-mode (L=1) code segment — base/limit ignored in 64-bit
    .dword  0x00000000
    .dword  0x00AF9A00              ; L=1, D=0, G=1, P=1, DPL=0, Type=0xA

gdt_kernel_data_64:
    .dword  0x0000FFFF
    .dword  0x00AF9200

gdt_end:

gdt_descriptor:
    .word   gdt_end - gdt_start - 1    ; GDT limit (16-bit)
    .dword  GDT_BASE                   ; GDT base (32-bit linear address)

; Segment selector constants
.equ SEL_CODE32,    0x08            ; index 1, RPL 0
.equ SEL_DATA32,    0x10            ; index 2
.equ SEL_CODE64,    0x18            ; index 3
.equ SEL_DATA64,    0x20            ; index 4


; =============================================================================
;  SECTION: IDT placeholder (filled properly after mode elevation)
; =============================================================================
.align  8
.org    IDT_BASE

idt_null_entries:
    .space  256 * 8                 ; 256 × 8-byte gate descriptors, zeroed

idt_descriptor_32:
    .word   256 * 8 - 1
    .dword  IDT_BASE

idt_descriptor_64:
    .word   256 * 16 - 1
    .qword  IDT_BASE


; =============================================================================
;  SECTION: Clownfish (16-bit) Initialization
; =============================================================================
.mode   CLOWNFISH
.text

clownfish_init:
    ; -----------------------------------------------------------------------
    ;  1. Report entry via debug UART (16-bit polled)
    ; -----------------------------------------------------------------------
    MOVI    AX, #'B'
    CALL    uart_putchar_16
    MOVI    AX, #'O'
    CALL    uart_putchar_16
    MOVI    AX, #'O'
    CALL    uart_putchar_16
    MOVI    AX, #'T'
    CALL    uart_putchar_16
    MOVI    AX, #'\n'
    CALL    uart_putchar_16

    ; -----------------------------------------------------------------------
    ;  2. Reprogram the PIC — remap IRQs away from processor exceptions §15.1
    ;     (mirrors PC/AT 8259 sequence; adapt for your interrupt controller)
    ; -----------------------------------------------------------------------
    ; Send ICW1: initialise, ICW4 needed
    MOVI    AX, #0x11
    ST      [PIC_MASTER_CMD],  AX
    ST      [PIC_SLAVE_CMD],   AX

    ; Send ICW2: new vector offsets
    MOVI    AX, #IRQ_MASTER_BASE
    ST      [PIC_MASTER_DATA], AX
    MOVI    AX, #IRQ_SLAVE_BASE
    ST      [PIC_SLAVE_DATA],  AX

    ; Send ICW3: master has slave on IRQ2 (bit 2 = 0x04); slave ID = 2
    MOVI    AX, #0x04
    ST      [PIC_MASTER_DATA], AX
    MOVI    AX, #0x02
    ST      [PIC_SLAVE_DATA],  AX

    ; Send ICW4: 8086 mode
    MOVI    AX, #0x01
    ST      [PIC_MASTER_DATA], AX
    ST      [PIC_SLAVE_DATA],  AX

    ; Mask all IRQs for now — the OS will unmask what it needs
    MOVI    AX, #0xFF
    ST      [PIC_MASTER_DATA], AX
    ST      [PIC_SLAVE_DATA],  AX

    ; -----------------------------------------------------------------------
    ;  3. Read processor capability register to decide target mode
    ; -----------------------------------------------------------------------
    RDCR    AX, CR4                 ; §16 — re-purpose CR4 as capability probe
    ; On a real SeaBird board, RDCR CR4 returns the implementation caps.
    ; Store in a well-known 16-bit memory cell for later reference.
    ST      [0x0490], AX            ; stash caps at BIOS data area scratch

    ; -----------------------------------------------------------------------
    ;  4. Load the GDT — required before entering protected mode
    ; -----------------------------------------------------------------------
    ; Build a 6-byte LGDT pseudo-descriptor and call WRCR (or assembler
    ; intrinsic) — SeaBird uses WRCR / direct memory descriptor loading.
    LEA     AX, [gdt_descriptor]
    WRCR    CR3, AX                 ; point CR3 at descriptor (platform-specific
                                    ; hook; replace with LGDT equivalent if your
                                    ; SeaBird toolchain has one)

    ; -----------------------------------------------------------------------
    ;  5. Enable protected mode: set CR0.PE
    ; -----------------------------------------------------------------------
    RDCR    AX, CR0
    OR      AX, #CR0_PE
    WRCR    CR0, AX

    ; Far jump to flush the instruction prefetch queue and load CS with a
    ; 32-bit code selector.  The .mode directive below switches the assembler
    ; to emit 32-bit encodings from this point on.
    JMP     SEL_CODE32 : tetra_init ; long jump: selector + offset


; =============================================================================
;  SECTION: Tetra (32-bit protected) Initialization
; =============================================================================
.mode   TETRA
.text

tetra_init:
    ; CS is now SEL_CODE32.  Reload all data segment registers.
    MOVI    AX, #SEL_DATA32
    MOV     DX, AX                  ; use DX as scratch (no segment regs in
                                    ; SeaBird — all segment enforcement is
                                    ; MMU-based per §2.2, so we update the
                                    ; kernel's working data selector via CR)
    WRCR    CR1, AX                 ; platform: store current data selector

    ; -----------------------------------------------------------------------
    ;  6. Set up a proper 32-bit stack
    ; -----------------------------------------------------------------------
    MOVI    SP, #STACK_TOP_32
    MOVI    BP, #STACK_TOP_32

    ; -----------------------------------------------------------------------
    ;  7. Zero-initialise BSS
    ; -----------------------------------------------------------------------
    LEA     AX, [bss_start]
    LEA     BX, [bss_end]
    SUB     CX, BX, AX              ; CX = byte count
    MOVI    DX, #0
    MEMFILL AX, DX, CX              ; §10.6 — fill with zero

    ; -----------------------------------------------------------------------
    ;  8. Set up a basic identity-mapped page directory for 32-bit paging
    ;     (4 MB pages — only first 4 entries cover 0–16 MB for boot purposes)
    ; -----------------------------------------------------------------------
    MOVI    AX, #PML4_BASE
    MOVI    CX, #1024               ; 1024 PDE entries

.pd_clear_loop:
    STW     [AX], #0                ; clear entry
    ADDI    AX, #4
    SUBI    CX, #1
    JNZ     .pd_clear_loop

    ; Identity-map first 4 × 4 MB pages (0x00000000 – 0x00FFFFFF)
    MOVI    AX, #PML4_BASE
    MOVI    BX, #(PAGE_HUGE)        ; 4 MB page, P + R/W + PS
    STW     [AX+0],  BX             ; PDE[0]  → 0x00000000
    ADDI    BX, #0x00400000
    STW     [AX+4],  BX             ; PDE[1]  → 0x00400000
    ADDI    BX, #0x00400000
    STW     [AX+8],  BX             ; PDE[2]  → 0x00800000
    ADDI    BX, #0x00400000
    STW     [AX+12], BX             ; PDE[3]  → 0x00C00000

    ; Point CR3 at page directory
    MOVI    AX, #PML4_BASE
    WRCR    CR3, AX

    ; Enable paging: set CR0.PG | CR0.WP
    RDCR    AX, CR0
    OR      AX, #(CR0_PG | CR0_WP)
    WRCR    CR0, AX

    ; -----------------------------------------------------------------------
    ;  9. Install a minimal 32-bit IDT (all handlers → generic_fault_32)
    ; -----------------------------------------------------------------------
    MOVI    CX, #256                ; 256 vectors
    MOVI    AX, #IDT_BASE
    LEA     BX, [generic_fault_32]

.idt32_fill:
    ; 32-bit interrupt gate descriptor (8 bytes):
    ;   [15:0]   offset[15:0]
    ;   [31:16]  selector
    ;   [47:32]  type/attributes (0x8E = 32-bit interrupt gate, DPL=0, P=1)
    ;   [63:48]  offset[31:16]
    STW     [AX+0], BX              ; offset low
    STW     [AX+2], #SEL_CODE32     ; selector
    STW     [AX+4], #0x8E00         ; type | zero byte
    STW     [AX+6], #0              ; offset high (stub at low address)
    ADDI    AX, #8
    SUBI    CX, #1
    JNZ     .idt32_fill

    ; Load IDT
    LEA     AX, [idt_descriptor_32]
    WRCR    CR2, AX                 ; platform: load IDT descriptor

    ; -----------------------------------------------------------------------
    ; 10. Probe capability bits: can we go to 64-bit?
    ; -----------------------------------------------------------------------
    LD      AX, [0x0490]            ; retrieve stashed caps
    TST     AX, #CAP_DRAGONET
    JZ      skip_dragonet           ; no 64-bit support — stay in Tetra

    ; -----------------------------------------------------------------------
    ; 11. Enable PAE (required for long mode / Dragonet)
    ; -----------------------------------------------------------------------
    RDCR    AX, CR4
    OR      AX, #CR4_PAE
    WRCR    CR4, AX

    ; -----------------------------------------------------------------------
    ; 12. Build a minimal 4-level page table (PML4) for first 1 GB identity
    ;     Each level lives at 4 KB-aligned addresses starting at PML4_BASE.
    ;
    ;     Layout:
    ;       PML4_BASE + 0x0000  = PML4  (512 × 8 bytes)
    ;       PML4_BASE + 0x1000  = PDPT  (512 × 8 bytes)
    ;       PML4_BASE + 0x2000  = PD    (512 × 8 bytes, 2 MB huge pages)
    ; -----------------------------------------------------------------------
    ; Clear all three tables (3 × 4 KB = 12 KB)
    MOVI    AX, #PML4_BASE
    MOVI    CX, #(3 * 4096)
    MOVI    DX, #0
    MEMFILL AX, DX, CX

    ; PML4[0] → PDPT
    MOVI    AX, #PML4_BASE
    MOVI    BX, #(PML4_BASE + 0x1000)
    OR      BX, #PAGE_PRESENT
    STQ     [AX], BX

    ; PDPT[0] → PD
    MOVI    AX, #(PML4_BASE + 0x1000)
    MOVI    BX, #(PML4_BASE + 0x2000)
    OR      BX, #PAGE_PRESENT
    STQ     [AX], BX

    ; PD[0..511] → identity 2 MB huge pages covering 0–1 GB
    MOVI    AX, #(PML4_BASE + 0x2000)
    MOVI    BX, #PAGE_HUGE          ; 2 MB, P | R/W | PS
    MOVI    CX, #512

.pml4_fill:
    STQ     [AX], BX
    ADDI    AX, #8
    ADDI    BX, #0x200000           ; next 2 MB frame
    SUBI    CX, #1
    JNZ     .pml4_fill

    ; Point CR3 at new PML4
    MOVI    AX, #PML4_BASE
    WRCR    CR3, AX

    ; -----------------------------------------------------------------------
    ; 13. Activate Dragonet (64-bit long) mode
    ;     On SeaBird this is done by writing the mode selector to CR0 extended
    ;     bits and performing a far jump with a 64-bit code selector.
    ; -----------------------------------------------------------------------
    RDCR    AX, CR0
    OR      AX, #CR0_PE
    WRCR    CR0, AX

    JMP     SEL_CODE64 : dragonet_init  ; far jump flushes pipeline, sets CS


; =============================================================================
;  SECTION: Dragonet (64-bit long mode) Initialization
; =============================================================================
.mode   DRAGONET
.text

dragonet_init:
    ; CS = SEL_CODE64.  All other segment registers are effectively flat.
    MOVI    AX, #SEL_DATA64
    WRCR    CR1, AX                 ; update data selector record

    ; -----------------------------------------------------------------------
    ; 14. 64-bit stack
    ; -----------------------------------------------------------------------
    MOVI    SP, #STACK_TOP_64
    MOVI    BP, #STACK_TOP_64

    ; -----------------------------------------------------------------------
    ; 15. Rebuild IDT as 64-bit interrupt gates (16 bytes each)
    ; -----------------------------------------------------------------------
    MOVI    CX, #256
    MOVI    AX, #IDT_BASE
    LEA     BX, [generic_fault_64]

.idt64_fill:
    ; 64-bit interrupt gate (16 bytes):
    ;   [15:0]   offset[15:0]
    ;   [31:16]  selector
    ;   [47:32]  type = 0x8E00
    ;   [63:48]  offset[31:16]
    ;   [95:64]  offset[63:32]
    ;   [127:96] reserved (zero)
    STW     [AX+0],  BX             ; offset[15:0]
    STW     [AX+2],  #SEL_CODE64
    STW     [AX+4],  #0x8E00
    SHR     DX, BX, #16
    STW     [AX+6],  DX             ; offset[31:16]
    SHR     DX, BX, #32
    STW     [AX+8],  DX             ; offset[63:32] (zero for low-mem stubs)
    STW     [AX+10], #0
    STW     [AX+12], #0
    STW     [AX+14], #0
    ADDI    AX, #16
    SUBI    CX, #1
    JNZ     .idt64_fill

    LEA     AX, [idt_descriptor_64]
    WRCR    CR2, AX

    ; -----------------------------------------------------------------------
    ; 16. Probe optional hardware features and configure
    ; -----------------------------------------------------------------------
    LD      AX, [0x0490]

    ; FPU / x87 ?
    TST     AX, #CAP_FPU
    JZ      .no_fpu
    RDCR    BX, CR0
    AND     BX, #~0x04              ; clear CR0.EM (no FPU emulation)
    OR      BX, #0x02               ; set CR0.MP (monitor coprocessor)
    WRCR    CR0, BX
.no_fpu:

    ; SIMD / Vector unit ?
    TST     AX, #CAP_VECTOR
    JZ      .no_vec
    RDCR    BX, CR4
    OR      BX, #0x0600             ; hypothetical OSFXSR | OSXMMEXCPT bits
    WRCR    CR4, BX
.no_vec:

    ; -----------------------------------------------------------------------
    ; 17. Reenable micro-op optimisation now that we're stable
    ; -----------------------------------------------------------------------
.mopt   2                           ; full aggressive micro-crunching §11.5
.autovec on

    ; -----------------------------------------------------------------------
    ; 18. Enable interrupts and hand off to the next-stage bootloader / kernel
    ; -----------------------------------------------------------------------
    STI                             ; §10.7 — enable interrupts
    JMP     boot_handoff            ; → next stage


; =============================================================================
;  SECTION: boot_handoff — reached regardless of highest mode achieved
; =============================================================================
.mode   DRAGONET                    ; assemble as 64-bit (harmless if 32-bit
                                    ; path branches to skip_dragonet variant)

skip_dragonet:
    ; We stayed in 32-bit Tetra mode.  Enable interrupts and hand off.
    STI
    ; Fall through to a 32-bit handoff (adjust .mode as appropriate in your
    ; real toolchain — this label is intentionally in 64-bit section for
    ; single-file simplicity; wrap in .ifdef TETRA_ONLY if needed.)

boot_handoff:
    ; At this point:
    ;   • SeaBird processor is in the highest supported operating mode
    ;   • Interrupts are enabled (PIC remapped, all IRQs still masked)
    ;   • Identity-mapped paging covers the first 1 GB of physical memory
    ;   • GDT has flat code+data descriptors for 32- and 64-bit modes
    ;   • IDT has a catch-all fault handler installed for all 256 vectors
    ;   • Stack is set up at STACK_TOP_{16|32|64}
    ;   • BSS is zeroed
    ;   • UART is available for early debug output
    ;   • Capability flags are cached at 0x0490
    ;
    ; Load the kernel or second-stage loader here.
    ; Convention: pass capability flags in R0, current mode in R1.
    LD      R0, [0x0490]            ; R0 = capability flags
    RDTS    R1                      ; R1 = timestamp (uniquely identifies boot)
    JMP     [kernel_entry_point]    ; → OS / second-stage loader


; =============================================================================
;  SECTION: Fault & Exception Stubs
; =============================================================================

generic_fault_32:
    ; Minimal 32-bit fault handler — halts the machine and debugs via UART.
    CLI
    MOVI    AX, #'!'
    CALL    uart_putchar_32
    HLT                             ; §10.7 — stop execution
    JMP     generic_fault_32        ; should never reach here


generic_fault_64:
    ; 64-bit variant
    CLI
    MOVI    AX, #'!'
    CALL    uart_putchar_64
    HLT
    JMP     generic_fault_64


; =============================================================================
;  SECTION: UART helpers (polled, mode-specific variants)
;  Signature: AX = character to send
; =============================================================================

; --- 16-bit variant ---
.mode   CLOWNFISH

uart_putchar_16:
    PUSH    BX
.uart16_wait:
    LD      BX, [UART_BASE + 5]     ; read Line Status Register
    TST     BX, #0x20               ; bit 5 = Transmitter Holding Register Empty
    JZ      .uart16_wait
    ST      [UART_BASE], AX         ; write character
    POP     BX
    RET

; --- 32-bit variant ---
.mode   TETRA

uart_putchar_32:
    PUSH    BX
.uart32_wait:
    LD      BX, [UART_BASE + 5]
    TST     BX, #0x20
    JZ      .uart32_wait
    ST      [UART_BASE], AX
    POP     BX
    RET

; --- 64-bit variant ---
.mode   DRAGONET

uart_putchar_64:
    PUSH    BX
.uart64_wait:
    LD      BX, [UART_BASE + 5]
    TST     BX, #0x20
    JZ      .uart64_wait
    ST      [UART_BASE], AX
    POP     BX
    RET


; =============================================================================
;  SECTION: BSS region (zero-initialised by step 7 above)
; =============================================================================
.bss
.align  16

bss_start:
    .space  4096                    ; 4 KB scratch BSS for boot-time use
bss_end:


; =============================================================================
;  SECTION: Kernel entry pointer (filled by linker or second-stage)
; =============================================================================
.data
.align  8

kernel_entry_point:
    .qword  0x0000000000100000      ; default: 1 MB mark (conventional load addr)
                                    ; override via linker script or board config


; =============================================================================
;  END OF FILE
; =============================================================================
