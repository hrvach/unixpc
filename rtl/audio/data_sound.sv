/*
    Copyright (c) 2026 by Hrvoje Cavrak

    What the speaker of a modem sounds like while data flows (FPGA only): the bytes the guest sends (originate side) and receives (answer side)
    are turned into line signal on two tone_gen voices, see data_sound_voice.sv for the signals of the three speeds (300 FSK, 1200 DPSK, 2400 QAM).
    The OSD "Modem sound" choice says which directions sound.
*/
module data_sound
#(
    parameter CLK_HZ = 20000000,
    parameter SDIV   = 417
)
(
    input         clk,
    input         rst,
    input         active,               /* data mode and no call sound running */
    input  [1:0]  select,               /* OSD: 0 off, 1 both directions, 2 receive only, 3 transmit only */
    input  [1:0]  speed,                /* 0 = 300 baud, 1 = 1200 baud, 2 = 2400 baud */
    input         transmit_valid,       /* a byte the guest sends (the originate side) */
    input  [7:0]  transmit_data,
    input         receive_valid,        /* a byte the guest receives (the answer side) */
    input  [7:0]  receive_data,

    output [31:0] voice2_increment,     /* the originate side is voice 2 of tone_gen, the answer side voice 3 */
    output [31:0] voice3_increment,
    output [31:0] voice2_phase_offset,
    output [31:0] voice3_phase_offset,
    output [7:0]  voice2_amplitude,
    output [7:0]  voice3_amplitude
);

localparam [1:0] SELECT_BOTH     = 2'd1;
localparam [1:0] SELECT_RECEIVE  = 2'd2;
localparam [1:0] SELECT_TRANSMIT = 2'd3;

wire transmit_sounds = (select == SELECT_BOTH) || (select == SELECT_TRANSMIT);
wire receive_sounds  = (select == SELECT_BOTH) || (select == SELECT_RECEIVE);

data_sound_voice #(.CLK_HZ(CLK_HZ), .SDIV(SDIV), .ORIGINATE(1)) originate_voice
(
    .clk          (clk),
    .rst          (rst),
    .active       (active),
    .enabled      (transmit_sounds),
    .speed        (speed),
    .byte_valid   (transmit_valid),
    .byte_data    (transmit_data),
    .increment    (voice2_increment),
    .phase_offset (voice2_phase_offset),
    .amplitude    (voice2_amplitude)
);

data_sound_voice #(.CLK_HZ(CLK_HZ), .SDIV(SDIV), .ORIGINATE(0)) answer_voice
(
    .clk          (clk),
    .rst          (rst),
    .active       (active),
    .enabled      (receive_sounds),
    .speed        (speed),
    .byte_valid   (receive_valid),
    .byte_data    (receive_data),
    .increment    (voice3_increment),
    .phase_offset (voice3_phase_offset),
    .amplitude    (voice3_amplitude)
);

endmodule
