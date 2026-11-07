/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Tone part of the 838A dialer word (FPGA only; on the real board the 838A chip makes these tones). The kernel (dialer.h) sends a 16 bit word to
    the dialer; its low byte tells the chip which two sine tones to play and how loud:

      bits 1:0   low group code, the row frequency of the DTMF keypad:    0 = 697 Hz, 1 = 770 Hz, 2 = 852 Hz, 3 = 941 Hz
      bits 3:2   high group code, the column frequency:                    0 = 1209 Hz, 1 = 1477 Hz, 2 = 1336 Hz, 3 = 1633 Hz
      bits 5:4   mode:    3 = both tones, 2 = high tone only, 1 = low tone only, 0 = both tones if a volume is set, else silence
      bits 7:6   volume:  2 = loud, 0 and 1 = normal, 3 = soft

    The outputs are what tone_gen needs for its voices 0 (low tone) and 1 (high tone): a phase increment each, and a target amplitude each (0 = off).
    The master volume from the OSD is applied later, in tone_gen.
*/
module dialer_tone_decoder
#(
    parameter CLK_HZ = 20000000,
    parameter SDIV   = 417
)
(
    input  [15:0] dialer_word,
    output reg [31:0] low_tone_increment,           /* phase increment per audio sample of the low (row) tone */
    output reg [31:0] high_tone_increment,          /* phase increment per audio sample of the high (column) tone */
    output [7:0]      low_tone_amplitude,           /* target amplitude 0..128 of the low tone */
    output [7:0]      high_tone_amplitude           /* target amplitude 0..128 of the high tone */
);

/* mode field values */
localparam [1:0] MODE_BOTH_IF_VOLUME_SET = 2'd0;
localparam [1:0] MODE_LOW_ONLY           = 2'd1;
localparam [1:0] MODE_HIGH_ONLY          = 2'd2;
localparam [1:0] MODE_BOTH               = 2'd3;

/* volume field values: the amplitude is out of 128 = full scale of the voice */
localparam [1:0] VOLUME_LOUD = 2'd2;
localparam [1:0] VOLUME_SOFT = 2'd3;
localparam [7:0] AMPLITUDE_LOUD   = 8'd128;
localparam [7:0] AMPLITUDE_NORMAL = 8'd90;
localparam [7:0] AMPLITUDE_SOFT   = 8'd45;

function [31:0] increment_for_hz(input [15:0] frequency_hz);
    increment_for_hz = tone_pkg::phase_increment(frequency_hz, CLK_HZ, SDIV);
endfunction

wire [1:0] low_group_code  = dialer_word[1:0];
wire [1:0] high_group_code = dialer_word[3:2];
wire [1:0] mode            = dialer_word[5:4];
wire [1:0] volume          = dialer_word[7:6];

/* the frequencies are constants selected by the group codes, so there is no divider in the design */
always @* begin
    case (low_group_code)
        2'd0:    low_tone_increment = increment_for_hz(16'd697);
        2'd1:    low_tone_increment = increment_for_hz(16'd770);
        2'd2:    low_tone_increment = increment_for_hz(16'd852);
        default: low_tone_increment = increment_for_hz(16'd941);
    endcase

    case (high_group_code)
        2'd0:    high_tone_increment = increment_for_hz(16'd1209);
        2'd1:    high_tone_increment = increment_for_hz(16'd1477);
        2'd2:    high_tone_increment = increment_for_hz(16'd1336);
        default: high_tone_increment = increment_for_hz(16'd1633);
    endcase
end

wire sound_requested = (mode != MODE_BOTH_IF_VOLUME_SET) || (volume != 2'd0);
wire low_tone_on     = sound_requested && (mode != MODE_HIGH_ONLY);
wire high_tone_on    = sound_requested && (mode != MODE_LOW_ONLY);

reg [7:0] amplitude_for_volume;

always @* begin
    case (volume)
        VOLUME_LOUD: amplitude_for_volume = AMPLITUDE_LOUD;
        VOLUME_SOFT: amplitude_for_volume = AMPLITUDE_SOFT;
        default:     amplitude_for_volume = AMPLITUDE_NORMAL;
    endcase
end

assign low_tone_amplitude  = low_tone_on  ? amplitude_for_volume : 8'd0;
assign high_tone_amplitude = high_tone_on ? amplitude_for_volume : 8'd0;

endmodule
