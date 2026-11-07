/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Tone synthesiser of the 3B1 telephone sounds (FPGA only; the real board has the 838A dialer chip and the line, here the sound is made digitally).

    Four sine voices are mixed into one mono 16 bit signed sample, 48 kHz (the rate of the HDMI audio of the MiSTer, see below):

      voice 0   low DTMF tone     driven by the dialer word (dialer_tone_decoder.sv)
      voice 1   high DTMF tone    driven by the dialer word
      voice 2   line sound A      driven from outside: the call sounds (line_sound.sv) or the modem data sound (data_sound.sv)
      voice 3   line sound B      driven from outside

    Each voice has a 32 bit phase that advances by its increment every sample. A quarter wave table (below) is shared by all voices, and the
    voices take turns on it: three clocks per voice, so the four voices need 12 of the 417 clocks of a sample.

      step 1   from the top bits of the phase: which quarter of the wave (first bit: second half is negative, second bit: odd quarters run backwards)
               and the table address
      step 2   the table gives its answer one clock later (registered read)
      step 3   multiply by the voice amplitude and add to the sum of the voices so far

    A voice amplitude moves towards its target by one step per sample, so a tone that is switched on or off ramps (no clicks). The master volume of
    the OSD scales every target: off, 1/4, 1/2, full.

    The sample rate: the framework takes the samples at exactly 48 kHz. A sample every 417 clocks of 20 MHz would be 47.96 kHz, 38 samples per second
    too few, so 38 times a second the framework would take the same sample twice, a steady flutter on top of every tone. 20 MHz / 48 kHz = 416.67
    clocks, so two samples in three are 417 clocks long and the third is 416 clocks (SHORT_SAMPLE_EVERY = 3): three samples take 1250 clocks, exactly
    62.5 microseconds, and the average rate is exactly 48 kHz.
*/
`ifndef SINE_FILE
`define SINE_FILE "sine.mem"
`endif

module tone_gen
#(
    parameter CLK_HZ = 20000000,
    parameter SDIV   = 417,         /* clocks per audio sample (the long ones) */
    parameter SHORT_SAMPLE_EVERY = 3        /* every this many samples one is a clock shorter (3: exactly 48 kHz at 20 MHz); 0: all samples SDIV clocks */
)
(
    input         clk,
    input         rst,
    input  [1:0]  master_volume,                /* OSD: 0 off, 1 low (1/4), 2 mid (1/2), 3 high (full) */
    input  [15:0] dialer_word,                  /* the word the kernel sent to the dialer, see dialer_tone_decoder.sv */
    input         line_voices_enabled,          /* voices 2 and 3 may sound */
    input  [31:0] voice2_increment,             /* phase increment per sample of voice 2 (tone_pkg::phase_increment) */
    input  [31:0] voice3_increment,
    input  [31:0] voice2_phase_offset,          /* added to the phase of voice 2, for phase shift keying */
    input  [31:0] voice3_phase_offset,
    input  [7:0]  voice2_amplitude,             /* target amplitude 0..128 */
    input  [7:0]  voice3_amplitude,

    output reg signed [15:0] audio_out = 16'sd0
);

localparam VOICES = 4;

/* ---- where the voices get their frequency and amplitude */
wire [31:0] dtmf_low_increment;
wire [31:0] dtmf_high_increment;
wire [7:0]  dtmf_low_amplitude;
wire [7:0]  dtmf_high_amplitude;

dialer_tone_decoder #(.CLK_HZ(CLK_HZ), .SDIV(SDIV)) dtmf
(
    .dialer_word         (dialer_word),
    .low_tone_increment  (dtmf_low_increment),
    .high_tone_increment (dtmf_high_increment),
    .low_tone_amplitude  (dtmf_low_amplitude),
    .high_tone_amplitude (dtmf_high_amplitude)
);

/* the OSD master volume: full, half, a quarter or silence */
function [7:0] apply_master_volume(input [7:0] amplitude, input [1:0] volume);
    case (volume)
        2'd3:    apply_master_volume = amplitude;
        2'd2:    apply_master_volume = {1'b0, amplitude[7:1]};
        2'd1:    apply_master_volume = {2'b00, amplitude[7:2]};
        default: apply_master_volume = 8'd0;
    endcase
endfunction

wire [31:0] voice_increment    [0:VOICES - 1];
wire [31:0] voice_phase_offset [0:VOICES - 1];
wire [7:0]  voice_target       [0:VOICES - 1];

assign voice_increment[0] = dtmf_low_increment;
assign voice_increment[1] = dtmf_high_increment;
assign voice_increment[2] = voice2_increment;
assign voice_increment[3] = voice3_increment;

assign voice_phase_offset[0] = 32'd0;
assign voice_phase_offset[1] = 32'd0;
assign voice_phase_offset[2] = voice2_phase_offset;
assign voice_phase_offset[3] = voice3_phase_offset;

assign voice_target[0] = apply_master_volume(dtmf_low_amplitude, master_volume);
assign voice_target[1] = apply_master_volume(dtmf_high_amplitude, master_volume);
assign voice_target[2] = line_voices_enabled ? apply_master_volume(voice2_amplitude, master_volume) : 8'd0;
assign voice_target[3] = line_voices_enabled ? apply_master_volume(voice3_amplitude, master_volume) : 8'd0;

/* ---- state of the voices */
reg [31:0] voice_phase     [0:VOICES - 1];
reg [7:0]  voice_amplitude [0:VOICES - 1];

integer voice_index;

initial begin
    for (voice_index = 0; voice_index < VOICES; voice_index = voice_index + 1) begin
        voice_phase[voice_index]     = 32'd0;
        voice_amplitude[voice_index] = 8'd0;
    end
end

/* ---- the sample cycle: one sample every SDIV clocks, then the voices one after the other */
typedef enum logic [1:0] {
    STEP_FIND_TABLE_ADDRESS = 2'd0,
    STEP_WAIT_FOR_TABLE     = 2'd1,
    STEP_ADD_TO_SUM         = 2'd2
} mixer_step_t;

reg [8:0]  clocks_in_sample = 9'd0;
reg        mixing = 1'b0;                       /* the voices of this sample are being added up */
reg [1:0]  current_voice = 2'd0;
mixer_step_t step = STEP_FIND_TABLE_ADDRESS;
reg signed [17:0] sum_of_voices = 18'sd0;

/* the position of this sample in the group of SHORT_SAMPLE_EVERY: the last one of the group is one clock shorter */
reg [1:0] sample_in_group = 2'd0;

wire short_sample  = (SHORT_SAMPLE_EVERY != 0) && (sample_in_group == SHORT_SAMPLE_EVERY - 1);
wire [8:0] sample_clocks = short_sample ? SDIV - 1 : SDIV;

wire sample_starts = (clocks_in_sample == sample_clocks - 1);

/* the voice phase, the way the table is read: the top two bits are the quarter, the next eight bits are the position in the quarter */
wire [31:0] current_phase = voice_phase[current_voice] + voice_phase_offset[current_voice];
wire        in_second_half    = current_phase[31];      /* the sine is negative */
wire        in_odd_quarter    = current_phase[30];      /* the quarter is traversed backwards */
wire [7:0]  position_in_quarter = current_phase[29:22];

reg [7:0] table_address = 8'd0;
reg       negative = 1'b0;                      /* sign of the table value on its way, remembered from step 1 */
reg [7:0] amplitude_in_use = 8'd0;


/*
    The sine table: one quarter wave, 256 entries of 15 bits, round(32767 * sin((i + 0.5) * pi / 512)). The values are in roms/sine.mem (written by
    tools/gen_sine_rom.py), read at start-up by $readmemh like the other memory files: 3B1.qsf defines SINE_FILE as roms/sine.mem for Quartus; the
    simulators define SINE_FILE with their own path. A block ROM on purpose: the romstyle attribute makes Quartus put it into an M10K block
    (registered read) instead of building 256 x 15 bits from logic.
*/
(* romstyle = "M10K" *) reg [14:0] sine_table [0:255];
reg [14:0] table_value;

initial $readmemh(`SINE_FILE, sine_table);

always @(posedge clk)
    table_value <= sine_table[table_address];

wire signed [15:0] signed_table_value = negative ? -$signed({1'b0, table_value}) : $signed({1'b0, table_value});
wire signed [24:0] voice_product      = signed_table_value * $signed({1'b0, amplitude_in_use});        /* at most 32767 * 128 */
wire signed [17:0] new_sum            = sum_of_voices + {{1{voice_product[24]}}, voice_product[24:8]};

/* the sum is limited to the 16 bit sample range (symmetric: -32767 .. 32767) */
function signed [15:0] clip_to_sample(input signed [17:0] value);
    if (value > 18'sd32767)
        clip_to_sample = 16'sd32767;
    else if (value < -18'sd32767)
        clip_to_sample = -16'sd32767;
    else
        clip_to_sample = value[15:0];
endfunction

always @(posedge clk) begin
    if (rst) begin
        clocks_in_sample <= 9'd0;
        sample_in_group  <= 2'd0;
        mixing           <= 1'b0;
        sum_of_voices    <= 18'sd0;
        audio_out        <= 16'sd0;

        for (voice_index = 0; voice_index < VOICES; voice_index = voice_index + 1) begin
            voice_phase[voice_index]     <= 32'd0;
            voice_amplitude[voice_index] <= 8'd0;
        end
    end
    else begin
        /* a new sample: advance every phase, move every amplitude one step towards its target, start mixing from the first voice */
        if (sample_starts) begin
            clocks_in_sample <= 9'd0;
            sample_in_group  <= (sample_in_group == SHORT_SAMPLE_EVERY - 1) ? 2'd0 : sample_in_group + 2'd1;
            mixing           <= 1'b1;
            current_voice    <= 2'd0;
            step             <= STEP_FIND_TABLE_ADDRESS;
            sum_of_voices    <= 18'sd0;

            for (voice_index = 0; voice_index < VOICES; voice_index = voice_index + 1) begin
                voice_phase[voice_index] <= voice_phase[voice_index] + voice_increment[voice_index];

                if (voice_amplitude[voice_index] < voice_target[voice_index])
                    voice_amplitude[voice_index] <= voice_amplitude[voice_index] + 8'd1;
                else if (voice_amplitude[voice_index] > voice_target[voice_index])
                    voice_amplitude[voice_index] <= voice_amplitude[voice_index] - 8'd1;
            end
        end
        else begin
            clocks_in_sample <= clocks_in_sample + 9'd1;
        end

        if (mixing && !sample_starts) begin
            case (step)
                STEP_FIND_TABLE_ADDRESS: begin
                    table_address    <= in_odd_quarter ? ~position_in_quarter : position_in_quarter;
                    negative         <= in_second_half;
                    amplitude_in_use <= voice_amplitude[current_voice];
                    step             <= STEP_WAIT_FOR_TABLE;
                end

                STEP_WAIT_FOR_TABLE: begin
                    step <= STEP_ADD_TO_SUM;
                end

                default: begin
                    sum_of_voices <= new_sum;
                    step          <= STEP_FIND_TABLE_ADDRESS;

                    if (current_voice == VOICES - 1) begin
                        mixing    <= 1'b0;
                        audio_out <= clip_to_sample(new_sum);
                    end
                    else begin
                        current_voice <= current_voice + 2'd1;
                    end
                end
            endcase
        end
    end
end

endmodule
