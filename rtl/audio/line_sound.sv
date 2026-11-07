/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Sounds of a call on the built-in modem (FPGA only), played on voices 2 and 3 of tone_gen. The real 3B1 only passes the telephone line to its
    speaker; with the modem bridge there is no line, so the sounds are synthesised from what the call is doing (the state of modem_bridge.sv):

      scene      starts when                                   sounds like
      DIAL       the hook relay closes, no digit dialled yet   dial tone, 350 + 440 Hz, until the first digit
      RING       the dial command goes out                     ringback, 440 + 480 Hz, 2 s on and 4 s off, until CONNECT or failure
      HANDSHAKE  CONNECT arrives                               2100 Hz answer tone for 1.5 s, 0.3 s of silence, then random FSK chatter,
                                                               5 s in all
      BUSY       the call fails or the far side hangs up       480 + 620 Hz, 0.5 s on and 0.5 s off, three times

    The DTMF digits themselves are not made here: they come from the dialer word (tone_gen voices 0 and 1).

    The scene clock counts milliseconds. The chatter of the handshake is FSK, a new random bit every 3 ms choosing between mark and space.
*/
module line_sound
import modem_pkg::*;
#(
    parameter CLK_HZ = 20000000,
    parameter SDIV   = 417
)
(
    input         clk,
    input         rst,
    input         enabled,                      /* call sounds on (OSD) and the modem mode is active */
    input  [2:0]  call_state,                   /* modem_bridge state: 0 on hook, 1 off hook, 2 sending command, 3 waiting for CONNECT, 4 data mode */
    input         digits_dialled,               /* at least one digit has been dialled in this call */
    input         carrier,

    output reg [31:0] voice2_increment,         /* the two tones of the scene, for tone_gen voices 2 and 3 */
    output reg [31:0] voice3_increment,
    output reg [7:0]  voice2_amplitude,
    output reg [7:0]  voice3_amplitude,
    output            scene_running             /* a call sound scene is playing (the data sound waits for it) */
);

/* ---- the call states of modem_bridge.sv are in modem_pkg.sv */

/* ---- scenes */
typedef enum logic [2:0] {
    SCENE_NONE      = 3'd0,
    SCENE_DIAL_TONE = 3'd1,
    SCENE_RINGBACK  = 3'd2,
    SCENE_HANDSHAKE = 3'd3,
    SCENE_BUSY      = 3'd4
} scene_t;

/* ---- timing of the scenes, in milliseconds */
localparam [15:0] RINGBACK_ON_MS        = 16'd2000;
localparam [15:0] RINGBACK_LAST_MS      = 16'd5999;     /* 2 s on, 4 s off, then it starts over */
localparam [15:0] BUSY_ON_MS            = 16'd500;
localparam [15:0] BUSY_LAST_MS          = 16'd999;      /* 0.5 s on, 0.5 s off */
localparam [1:0]  BUSY_LAST_CYCLE       = 2'd2;         /* the third cycle is the last */
localparam [15:0] ANSWER_TONE_END_MS    = 16'd1500;
localparam [15:0] CHATTER_START_MS      = 16'd1800;
localparam [15:0] HANDSHAKE_LENGTH_MS   = 16'd5000;
localparam [1:0]  CHATTER_BIT_LAST_MS   = 2'd2;         /* the FSK chatter changes bit every 3 ms: counts 0, 1, 2 */

/* ---- tones and loudness (amplitudes out of 128 = full scale of a voice) */
localparam [7:0] AMPLITUDE_CALL_TONES   = 8'd48;
localparam [7:0] AMPLITUDE_ANSWER_TONE  = 8'd64;
localparam [7:0] AMPLITUDE_CHATTER      = 8'd56;

function [31:0] increment_for_hz(input [15:0] frequency_hz);
    increment_for_hz = tone_pkg::phase_increment(frequency_hz, CLK_HZ, SDIV);
endfunction

localparam CLOCKS_PER_MS = CLK_HZ / 1000;

scene_t scene = SCENE_NONE;
reg [2:0]  call_state_before = 3'd0;
reg        carrier_before = 1'b0;
reg [14:0] clocks_in_ms = 15'd0;
reg [15:0] scene_ms = 16'd0;                    /* milliseconds since the scene started (it stops counting at 65535) */
reg [1:0]  chatter_ms = 2'd0;                   /* counts the milliseconds of one chatter bit */
reg [1:0]  busy_cycles_done = 2'd0;
reg [15:0] chatter_random = 16'hACE1;           /* LFSR: bit 0 is the mark / space choice */

wire millisecond_ends = (clocks_in_ms == CLOCKS_PER_MS - 1);
wire chatter_bit_ends = (chatter_ms == CHATTER_BIT_LAST_MS);

/* ---- the events that start a scene: a change of the call state, seen against the state one clock earlier */
wire hook_closed_before_dialling = (call_state == CALL_OFF_HOOK) && (call_state_before != CALL_OFF_HOOK) && !digits_dialled;
wire dial_command_sent           = (call_state == CALL_SENDING_COMMAND) && (call_state_before != CALL_SENDING_COMMAND);
wire connected                   = (call_state == CALL_DATA_MODE) && (call_state_before != CALL_DATA_MODE);
wire connect_failed              = (call_state_before == CALL_WAITING_FOR_CONNECT) && (call_state != CALL_WAITING_FOR_CONNECT) &&
                                   (call_state != CALL_DATA_MODE);
wire far_side_hung_up            = (call_state == CALL_DATA_MODE) && carrier_before && !carrier;
wire we_hung_up                  = (call_state == CALL_ON_HOOK) && (call_state_before != CALL_ON_HOOK);

always @(posedge clk) begin
    call_state_before <= call_state;
    carrier_before    <= carrier;

    if (rst || !enabled) begin
        scene         <= SCENE_NONE;
        scene_ms      <= 16'd0;
        clocks_in_ms  <= 15'd0;
    end
    else begin
        /* the millisecond clock, the scene clock and the chatter */
        if (millisecond_ends) begin
            clocks_in_ms <= 15'd0;

            if (scene_ms != 16'hFFFF)
                scene_ms <= scene_ms + 16'd1;

            if (chatter_bit_ends) begin
                chatter_ms     <= 2'd0;
                chatter_random <= {chatter_random[14:0], chatter_random[15] ^ chatter_random[13] ^ chatter_random[12] ^ chatter_random[10]};
            end
            else begin
                chatter_ms <= chatter_ms + 2'd1;
            end
        end
        else begin
            clocks_in_ms <= clocks_in_ms + 15'd1;
        end

        /* how a scene ends (or repeats) */
        case (scene)
            SCENE_DIAL_TONE: begin
                if (digits_dialled || call_state != CALL_OFF_HOOK) begin
                    scene    <= SCENE_NONE;
                    scene_ms <= 16'd0;
                end
            end

            SCENE_RINGBACK: begin
                if (call_state != CALL_SENDING_COMMAND && call_state != CALL_WAITING_FOR_CONNECT) begin
                    scene    <= SCENE_NONE;
                    scene_ms <= 16'd0;
                end
                else if (millisecond_ends && scene_ms == RINGBACK_LAST_MS) begin
                    scene_ms <= 16'd0;
                end
            end

            SCENE_HANDSHAKE: begin
                if (scene_ms >= HANDSHAKE_LENGTH_MS) begin
                    scene    <= SCENE_NONE;
                    scene_ms <= 16'd0;
                end
            end

            SCENE_BUSY: begin
                if (millisecond_ends && scene_ms == BUSY_LAST_MS) begin
                    scene_ms         <= 16'd0;
                    busy_cycles_done <= busy_cycles_done + 2'd1;

                    if (busy_cycles_done == BUSY_LAST_CYCLE)
                        scene <= SCENE_NONE;
                end
            end

            default: begin
            end
        endcase

        /* how a scene starts; these come last, so a new scene wins over the end of the old one */
        if (hook_closed_before_dialling) begin
            scene    <= SCENE_DIAL_TONE;
            scene_ms <= 16'd0;
        end

        if (dial_command_sent) begin
            scene    <= SCENE_RINGBACK;
            scene_ms <= 16'd0;
        end

        if (connected) begin
            scene    <= SCENE_HANDSHAKE;
            scene_ms <= 16'd0;
        end

        if (connect_failed) begin
            scene            <= SCENE_BUSY;
            scene_ms         <= 16'd0;
            busy_cycles_done <= 2'd0;
        end

        if (far_side_hung_up) begin
            scene            <= SCENE_BUSY;
            scene_ms         <= 16'd0;
            busy_cycles_done <= 2'd0;
        end

        if (we_hung_up && scene != SCENE_BUSY) begin
            scene    <= SCENE_NONE;
            scene_ms <= 16'd0;
        end
    end
end

assign scene_running = (scene != SCENE_NONE);

/* ---- the tones of the running scene */
wire chatter_is_mark = chatter_random[0];

always @* begin
    voice2_increment = 32'd0;
    voice3_increment = 32'd0;
    voice2_amplitude = 8'd0;
    voice3_amplitude = 8'd0;

    case (scene)
        SCENE_DIAL_TONE: begin
            voice2_increment = increment_for_hz(16'd350);
            voice3_increment = increment_for_hz(16'd440);
            voice2_amplitude = AMPLITUDE_CALL_TONES;
            voice3_amplitude = AMPLITUDE_CALL_TONES;
        end

        SCENE_RINGBACK: begin
            if (scene_ms < RINGBACK_ON_MS) begin
                voice2_increment = increment_for_hz(16'd440);
                voice3_increment = increment_for_hz(16'd480);
                voice2_amplitude = AMPLITUDE_CALL_TONES;
                voice3_amplitude = AMPLITUDE_CALL_TONES;
            end
        end

        SCENE_BUSY: begin
            if (scene_ms < BUSY_ON_MS) begin
                voice2_increment = increment_for_hz(16'd480);
                voice3_increment = increment_for_hz(16'd620);
                voice2_amplitude = AMPLITUDE_CALL_TONES;
                voice3_amplitude = AMPLITUDE_CALL_TONES;
            end
        end

        SCENE_HANDSHAKE: begin
            if (scene_ms < ANSWER_TONE_END_MS) begin
                voice2_increment = increment_for_hz(16'd2100);
                voice2_amplitude = AMPLITUDE_ANSWER_TONE;
            end
            else if (scene_ms >= CHATTER_START_MS) begin
                voice2_increment = chatter_is_mark ? increment_for_hz(16'd2225) : increment_for_hz(16'd2025);
                voice2_amplitude = AMPLITUDE_CHATTER;
            end
        end

        default: begin
        end
    endcase
end

endmodule
