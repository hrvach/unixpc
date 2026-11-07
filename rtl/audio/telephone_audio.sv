/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Audio of the core (FPGA only): everything you hear. It puts the three sound sources in front of the tone generator:

      dialer word        the DTMF beeps, ring and key tones the kernel asks the 838A dialer for      (tone_gen voices 0 and 1)
      line_sound         the sounds of a call on the modem bridge: dial tone, ringback, handshake, busy   (voices 2 and 3)
      data_sound         the line signal of the data bytes during a call                                   (voices 2 and 3)

    A call sound scene has priority: while one plays the data sound is held back, and the tone generator's voices 2 and 3 take the scene's tones.
    Outside a scene they take the data sound's tones. The result is one mono signed 16 bit sample stream at 48 kHz.
*/
module telephone_audio
#(
    parameter CLK_HZ = 20000000,
    parameter SHORT_SAMPLE_EVERY = 3            /* see tone_gen.sv: 3 makes the sample rate exactly 48 kHz; 0 only in the shortened-time tests */
)
(
    input         clk,
    input         reset,
    input  [1:0]  master_volume,                /* OSD sound volume: 0 off, 1 low, 2 mid, 3 high */
    input         call_sounds_enabled,          /* OSD call sounds on, and the modem mode is active */
    input         modem_mode,                   /* the serial channel B is connected to the modem bridge */
    input  [1:0]  data_sound_select,            /* OSD modem sound: 0 call sounds only, 1 both directions, 2 receive only, 3 transmit only */
    input  [15:0] dialer_word,                  /* the word the kernel last sent to the 838A dialer */
    input  [2:0]  modem_call_state,             /* modem_bridge state */
    input         digits_dialled,
    input         carrier,
    input         line_speed_1200,              /* 882A modem chip: W1 bit 4 (1200 baud) */
    input         line_speed_2400,              /* 882A modem chip: W8 bit 3 (2400 baud, with W1 bit 4) */
    input         guest_transmit_valid,         /* a byte the guest sends on channel B */
    input  [7:0]  guest_transmit_data,
    input         guest_receive_valid,          /* a byte the guest receives on channel B */
    input  [7:0]  guest_receive_data,

    output signed [15:0] audio_out
);

wire [31:0] scene_increment2;
wire [31:0] scene_increment3;
wire [7:0]  scene_amplitude2;
wire [7:0]  scene_amplitude3;
wire        scene_running;

line_sound #(.CLK_HZ(CLK_HZ)) call_scenes
(
    .clk              (clk),
    .rst              (reset),
    .enabled          (call_sounds_enabled),
    .call_state       (modem_call_state),
    .digits_dialled   (digits_dialled),
    .carrier          (carrier),
    .voice2_increment (scene_increment2),
    .voice3_increment (scene_increment3),
    .voice2_amplitude (scene_amplitude2),
    .voice3_amplitude (scene_amplitude3),
    .scene_running    (scene_running)
);

/* the speed the guest selected: 300 baud unless the modem chip is set to 1200, and 2400 if it is set to 2400 as well */
reg [1:0] data_speed;

always @* begin
    if (!line_speed_1200)
        data_speed = 2'd0;
    else if (line_speed_2400)
        data_speed = 2'd2;
    else
        data_speed = 2'd1;
end

wire [31:0] data_increment2;
wire [31:0] data_increment3;
wire [31:0] data_phase_offset2;
wire [31:0] data_phase_offset3;
wire [7:0]  data_amplitude2;
wire [7:0]  data_amplitude3;

data_sound #(.CLK_HZ(CLK_HZ)) data_bytes
(
    .clk                 (clk),
    .rst                 (reset),
    .active              (modem_mode & carrier & ~scene_running),
    .select              (data_sound_select),
    .speed               (data_speed),
    .transmit_valid      (guest_transmit_valid),
    .transmit_data       (guest_transmit_data),
    .receive_valid       (guest_receive_valid),
    .receive_data        (guest_receive_data),
    .voice2_increment    (data_increment2),
    .voice3_increment    (data_increment3),
    .voice2_phase_offset (data_phase_offset2),
    .voice3_phase_offset (data_phase_offset3),
    .voice2_amplitude    (data_amplitude2),
    .voice3_amplitude    (data_amplitude3)
);

wire data_sound_playing = (data_amplitude2 != 8'd0) || (data_amplitude3 != 8'd0);

tone_gen #(.CLK_HZ(CLK_HZ), .SHORT_SAMPLE_EVERY(SHORT_SAMPLE_EVERY)) synthesiser
(
    .clk                 (clk),
    .rst                 (reset),
    .master_volume       (master_volume),
    .dialer_word         (dialer_word),
    .line_voices_enabled (call_sounds_enabled | data_sound_playing),
    .voice2_increment    (scene_running ? scene_increment2 : data_increment2),
    .voice3_increment    (scene_running ? scene_increment3 : data_increment3),
    .voice2_phase_offset (scene_running ? 32'd0 : data_phase_offset2),
    .voice3_phase_offset (scene_running ? 32'd0 : data_phase_offset3),
    .voice2_amplitude    (scene_running ? scene_amplitude2 : data_amplitude2),
    .voice3_amplitude    (scene_running ? scene_amplitude3 : data_amplitude3),
    .audio_out           (audio_out)
);

endmodule
