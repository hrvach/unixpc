/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The CPU of the UNIX PC board: an FX68K (a 68000 with 68010 extensions) in the bus level interface that the board (unixpc_board.sv) expects.

    The 20 MHz master clock drives the core; the CPU itself runs at 10 MHz, its two clock phases (enPhi1, enPhi2) alternate every clock. The board
    reconstructs address bit 0 from the two data strobes, so the address output has a constant 0 there.
*/
module cpu_fx68k
(
    input         clk,              /* 20 MHz master clock */
    input         reset,            /* active high, synchronous */

    /* data bus */
    input  [15:0] din,              /* read data from the board */
    output [15:0] dout,             /* write data to the board */

    /* address, strobes and function code */
    output [23:0] addr,             /* byte address; bit 0 is always 0 */
    output        as_n,             /* address strobe */
    output        uds_n,            /* upper data strobe (D15..D8) */
    output        lds_n,            /* lower data strobe (D7..D0) */
    output        rw_n,             /* 1 = read, 0 = write */
    output [2:0]  fc,               /* function code: user / supervisor, program / data, interrupt acknowledge */

    /* bus cycle termination and interrupts */
    input         dtack_n,          /* data transfer acknowledge */
    input         berr_n,           /* bus error */
    input         vpa_n,            /* valid peripheral address: autovector */
    input  [2:0]  ipl_n,            /* interrupt priority level, inverted */

    /* bus arbitration (DMA) */
    input         br_n,             /* bus request */
    input         bgack_n,          /* bus grant acknowledge: a DMA master has the bus */
    output        bg_n,             /* bus grant */

    /* processor state */
    output        reset_out_n,      /* the CPU executes RESET */
    output        halted_n          /* the CPU is halted */
);

/*
    The reset is registered once: it keeps the control logic of the core free of top level input dependencies, which makes simulation in Verilator much
    faster. While reset is active the clock phase is held at 0, so the first phase after reset is always enPhi1.
*/
reg reset_registered = 1'b1;
reg phase = 1'b0;

always @(posedge clk) begin
    reset_registered <= reset;
    phase            <= reset_registered ? 1'b0 : ~phase;
end

wire phase_1_enable = ~phase;
wire phase_2_enable =  phase;

wire [23:1] word_address;

assign addr = {word_address, 1'b0};

fx68k cpu
(
    .clk      (clk),
    .HALTn    (1'b1),
    .extReset (reset_registered),
    .pwrUp    (reset_registered),
    .enPhi1   (phase_1_enable),
    .enPhi2   (phase_2_enable),

    .eRWn     (rw_n),
    .ASn      (as_n),
    .LDSn     (lds_n),
    .UDSn     (uds_n),
    .E        (),
    .VMAn     (),

    .FC0      (fc[0]),
    .FC1      (fc[1]),
    .FC2      (fc[2]),
    .BGn      (bg_n),
    .oRESETn  (reset_out_n),
    .oHALTEDn (halted_n),
    .DTACKn   (dtack_n),
    .VPAn     (vpa_n),
    .BERRn    (berr_n),
    .BRn      (br_n),
    .BGACKn   (bgack_n),
    .IPL0n    (ipl_n[0]),
    .IPL1n    (ipl_n[1]),
    .IPL2n    (ipl_n[2]),
    .iEdb     (din),
    .oEdb     (dout),
    .eab      (word_address)
);

endmodule
