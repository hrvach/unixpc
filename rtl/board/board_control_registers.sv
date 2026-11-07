/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The board's write-only control registers (logic board sheet 6): the general control register GCR (an addressable latch, IC 7K) and the two modem /
    miscellaneous registers MCR and MCR2.

    GCR is written at E4x000: address bits A14..A12 choose the latch bit, and D15 is the value written to it (so the kernel sets a bit with a single
    store).
      bit 0  EE+      error enable: a bus error condition raises BERR*
      bit 1  PIE+     parity interrupt enable
      bit 2  BP+      force bad parity on RAM writes (diagnostic)
      bit 3  ROMLMAP+ 0: ROM is forced into the low 4 MB (the state after reset), 1: RAM is there
      bits 4..6       latch bits that nothing in this design looks at (latch address 7 selects nothing)
    MCR (4Axxxx) holds, among others, the CLRSINT* bit (15), the printer strobe (13) and the four front panel LEDs (bits 11..8). MCR2 (E2xxxx) bit 4
    reads back in the telephony status.

    The 60 Hz real time interrupt (the level 6 request) lives here too, because MCR bit 15 (CLRSINT*) is its clear: held low it clears the request and
    keeps it masked. On the real board the interrupt is the line frequency; here a counter divides the 20 MHz clock down to 60 Hz. The request stays
    up until the kernel clears it.
*/
module board_control_registers
import unixpc_registers_pkg::*;
(
    input         clk,
    input         rst,
    input         write_go,                     /* one clock pulse per CPU write */
    input         general_control_selected,     /* the write is to GCR (E4xxxx) */
    input         modem_control_selected,       /* ... to MCR (4Axxxx) */
    input         modem_control_2_selected,     /* ... to MCR2 (E2xxxx) */
    input  [14:12] latch_select,                /* A14..A12: which GCR latch bit */
    input  [15:0] cpu_data,

    output        error_enable,                 /* GCR EE */
    output        parity_interrupt_enable,      /* GCR PIE */
    output        force_bad_parity,             /* GCR BP */
    output        rom_low_map,                  /* GCR ROMLMAP */
    output [15:0] mcr,                          /* layout: misc_control_reg_t in unixpc_registers_pkg.sv */
    output reg [15:0] mcr2 = 16'h0000,
    output [3:0]  leds,                         /* MCR LED0..LED3 */

    output reg    real_time_interrupt_pending = 1'b0
);

localparam [2:0] LATCH_SELECT_NOTHING = 3'd7;

reg [6:0]             gcr = 7'd0;           /* one latch per bit; the field names are in general_control_reg_t */
misc_control_reg_t    mcr_fields = 16'h0000;
general_control_reg_t gcr_fields;

assign gcr_fields = gcr;

assign error_enable            = gcr_fields.EE;
assign parity_interrupt_enable = gcr_fields.PIE;
assign force_bad_parity        = gcr_fields.BP;
assign rom_low_map             = gcr_fields.ROMLMAP;

assign mcr = mcr_fields;

always @(posedge clk) begin
    if (rst) begin
        gcr  <= 7'd0;
        mcr_fields <= 16'h0000;
        mcr2 <= 16'h0000;
    end else if (write_go) begin
        if (general_control_selected && latch_select != LATCH_SELECT_NOTHING)
            gcr[latch_select] <= cpu_data[15];

        if (modem_control_selected)
            mcr_fields <= cpu_data;

        if (modem_control_2_selected)
            mcr2 <= cpu_data;
    end
end

assign leds = {mcr_fields.LED0, mcr_fields.LED1, mcr_fields.LED2, mcr_fields.LED3};

/* ---- the 60 Hz interrupt */
localparam [18:0] CLOCKS_PER_TICK_MINUS_ONE = 19'd333332;      /* 20 MHz / 60 Hz = 333 333 clocks */

reg [18:0] clocks_into_tick = 19'd0;
wire       tick             = (clocks_into_tick == CLOCKS_PER_TICK_MINUS_ONE);

always @(posedge clk) begin
    clocks_into_tick            <= tick ? 19'd0 : clocks_into_tick + 19'd1;
    real_time_interrupt_pending <= mcr_fields.CLRSINT & (real_time_interrupt_pending | tick);
end

endmodule
