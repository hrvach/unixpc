/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Interrupt priority encoder (IC 18F, logic board sheet 6).

    Seven interrupt request lines come in, one per 68010 interrupt level. When several are active the highest level wins, and its number is put on the
    CPU's three interrupt priority level pins IPL2*..IPL0*. Those pins are active low, so the output is the level number inverted: level 7 (the NMI)
    is 000, "no interrupt" is 111.

    The inputs are active high here (the sources in this design are all active high). It does not model the vector: the 68010 takes the autovector of
    the level, which is what the 3B1 kernel expects.
*/
module irq_ctrl
(
    input        level7_parity_or_mmu_error,    /* NMI: parity error or MMU error */
    input        level6_real_time_clock,        /* 60 Hz interrupt of the real time clock */
    input        level5_expansion,              /* expansion slot (the Ethernet board uses it for its second interrupt) */
    input        level4_serial_7201,            /* the 7201 serial controller */
    input        level3_keyboard_or_modem,      /* keyboard (6850 ACIA) and modem */
    input        level2_disk_or_printer,        /* floppy and hard disk controllers, parallel printer */
    input        level1_expansion,              /* expansion slot (the Ethernet board's LANCE) */

    output [2:0] nIPL                           /* IPL2*..IPL0* pins of the 68010: the winning level, inverted */
);

localparam [2:0] LEVEL_NONE = 3'd0;
localparam [2:0] LEVEL_1    = 3'd1;
localparam [2:0] LEVEL_2    = 3'd2;
localparam [2:0] LEVEL_3    = 3'd3;
localparam [2:0] LEVEL_4    = 3'd4;
localparam [2:0] LEVEL_5    = 3'd5;
localparam [2:0] LEVEL_6    = 3'd6;
localparam [2:0] LEVEL_7    = 3'd7;

reg [2:0] winning_level;

/* The first row that matches wins: a request on a higher level hides every lower one ("?" = does not matter). */
always @* begin
    casez ({level7_parity_or_mmu_error,
            level6_real_time_clock,
            level5_expansion,
            level4_serial_7201,
            level3_keyboard_or_modem,
            level2_disk_or_printer,
            level1_expansion})
        7'b1??????: winning_level = LEVEL_7;
        7'b01?????: winning_level = LEVEL_6;
        7'b001????: winning_level = LEVEL_5;
        7'b0001???: winning_level = LEVEL_4;
        7'b00001??: winning_level = LEVEL_3;
        7'b000001?: winning_level = LEVEL_2;
        7'b0000001: winning_level = LEVEL_1;
        default:    winning_level = LEVEL_NONE;
    endcase
end

assign nIPL = ~winning_level;

endmodule
