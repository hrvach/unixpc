/*
    Copyright (c) 2026 by Hrvoje Cavrak

    One direction of the modem data sound (FPGA only): turns the bytes that go through the serial channel into the line signal a modem speaker would
    make, on one tone_gen voice. data_sound.sv instantiates one for the originate side (what the guest sends) and one for the answer side (what the
    guest receives).

    The bytes wait in a small FIFO. A symbol engine runs at the symbol rate and takes the bits of the bytes, framed as a serial line: a start bit (0),
    8 data bits least significant first, a stop bit (1). With no byte to send the line idles with ones.

      speed       symbol rate   bits per symbol   signal
      300 baud    300 /s        1                 Bell 103 FSK: mark / space tone by the bit
      1200 baud   600 /s        2                 Bell 212A DPSK: the bits are the phase step (0, 90, 180, 270 degrees) of the carrier
      2400 baud   600 /s        4                 V.22bis: two bits phase step, two bits amplitude (4 levels)

    The PSK speeds scramble the bits with a free running LFSR first, so an idle line hisses like the real thing.
*/
module data_sound_voice
#(
    parameter CLK_HZ     = 20000000,
    parameter SDIV       = 417,
    parameter ORIGINATE  = 1            /* 1: originate side (carrier 1200 Hz, FSK 1270 / 1070); 0: answer side (carrier 2400 Hz, FSK 2225 / 2025) */
)
(
    input         clk,
    input         rst,
    input         active,               /* data mode and no call sound running */
    input         enabled,              /* the OSD lets this direction sound */
    input  [1:0]  speed,                /* 0 = 300 baud, 1 = 1200 baud, 2 = 2400 baud */
    input         byte_valid,           /* a byte of this direction goes through the serial channel */
    input  [7:0]  byte_data,

    output [31:0] increment,            /* the voice for tone_gen: phase increment, phase offset (PSK) and amplitude */
    output [31:0] phase_offset,
    output [7:0]  amplitude
);

localparam [1:0] SPEED_300  = 2'd0;
localparam [1:0] SPEED_1200 = 2'd1;
localparam [1:0] SPEED_2400 = 2'd2;

localparam [0:0] START_BIT = 1'b0;
localparam [0:0] STOP_BIT  = 1'b1;
localparam [4:0] BITS_PER_BYTE_FRAME = 5'd10;           /* start, 8 data bits, stop */

function [31:0] increment_for_hz(input [15:0] frequency_hz);
    increment_for_hz = tone_pkg::phase_increment(frequency_hz, CLK_HZ, SDIV);
endfunction

reg [2:0]  bits_per_symbol;
reg [31:0] symbol_clocks;

always @* begin
    case (speed)
        SPEED_300: begin
            bits_per_symbol = 3'd1;
            symbol_clocks   = CLK_HZ / 300;
        end

        SPEED_1200: begin
            bits_per_symbol = 3'd2;
            symbol_clocks   = CLK_HZ / 600;
        end

        default: begin
            bits_per_symbol = 3'd4;
            symbol_clocks   = CLK_HZ / 600;
        end
    endcase
end

/* ---- the byte FIFO: eight bytes, pointers one bit wider than the index so that full and empty can be told apart */
reg [7:0] fifo_bytes [0:7];
reg [3:0] write_pointer = 4'd0;
reg [3:0] read_pointer  = 4'd0;

wire fifo_not_empty = (write_pointer != read_pointer);
wire fifo_has_room  = ((write_pointer - read_pointer) < 4'd8);

/* ---- the bit queue between the FIFO and the symbol engine, least significant bit first */
reg [19:0] bit_queue = 20'd0;
reg [4:0]  bits_in_queue = 5'd0;

/* when the queue holds fewer bits than a symbol needs it is refilled: with the next byte frame, or with idle ones if the FIFO is empty */
reg [19:0] queue_after_refill;
reg [4:0]  bits_after_refill;
reg        take_byte_from_fifo;

always @* begin
    queue_after_refill  = bit_queue;
    bits_after_refill   = bits_in_queue;
    take_byte_from_fifo = 1'b0;

    if (bits_in_queue < bits_per_symbol) begin
        if (fifo_not_empty) begin
            queue_after_refill  = bit_queue | ({10'd0, STOP_BIT, fifo_bytes[read_pointer[2:0]], START_BIT} << bits_in_queue);
            bits_after_refill   = bits_in_queue + BITS_PER_BYTE_FRAME;
            take_byte_from_fifo = 1'b1;
        end
        else begin
            /*
                Idle: ones. Four of them are written although only bits_per_symbol are counted, so below 2400 baud the surplus ones stay above the
                queue and are OR-ed into the start bit and first data bits of the next byte. This is how the module has always sounded and is kept as
                it is (it only changes the sound of the first symbols after an idle gap).
            */
            queue_after_refill = bit_queue | (20'd15 << bits_in_queue);
            bits_after_refill  = bits_in_queue + {2'd0, bits_per_symbol};
        end
    end
end

wire [3:0] symbol_bits = queue_after_refill[3:0];

/* ---- scrambler: 16 bit LFSR, advanced four steps per symbol */
reg [15:0] scrambler = 16'hB5A3;

function [15:0] scrambler_step(input [15:0] state);
    scrambler_step = {state[14:0], state[15] ^ state[14] ^ state[12] ^ state[3]};
endfunction

wire [15:0] scrambler_after_symbol = scrambler_step(scrambler_step(scrambler_step(scrambler_step(scrambler))));

/* ---- the signal state */
reg [31:0] symbol_timer = 32'd0;
reg [2:0]  carrier_phase_step = 3'd0;           /* the carrier phase, in steps of 45 degrees */
reg [1:0]  amplitude_level = 2'd3;
reg        fsk_mark = 1'b1;                     /* the FSK bit being sent: 1 = mark tone */

wire symbol_ends = (symbol_timer == symbol_clocks - 1);

wire [1:0] scrambled_phase_step_1200 = symbol_bits[1:0] ^ scrambler_after_symbol[1:0];
wire [3:0] scrambled_symbol_2400     = symbol_bits ^ scrambler_after_symbol[3:0];

always @(posedge clk) begin
    if (rst || !active) begin
        write_pointer      <= 4'd0;
        read_pointer       <= 4'd0;
        bit_queue          <= 20'd0;
        bits_in_queue      <= 5'd0;
        symbol_timer       <= 32'd0;
        carrier_phase_step <= 3'd0;
        amplitude_level    <= 2'd3;
        fsk_mark           <= 1'b1;
    end
    else begin
        if (byte_valid && enabled && fifo_has_room) begin
            fifo_bytes[write_pointer[2:0]] <= byte_data;
            write_pointer                  <= write_pointer + 4'd1;
        end

        if (symbol_ends) begin
            symbol_timer  <= 32'd0;
            bit_queue     <= queue_after_refill >> bits_per_symbol;
            bits_in_queue <= bits_after_refill - {2'd0, bits_per_symbol};
            scrambler     <= scrambler_after_symbol;

            if (take_byte_from_fifo)
                read_pointer <= read_pointer + 4'd1;

            case (speed)
                SPEED_300: begin
                    fsk_mark <= symbol_bits[0];
                end

                SPEED_1200: begin
                    carrier_phase_step <= carrier_phase_step + {scrambled_phase_step_1200, 1'b0};
                end

                SPEED_2400: begin
                    carrier_phase_step <= carrier_phase_step + {scrambled_symbol_2400[1:0], 1'b0};
                    amplitude_level    <= scrambled_symbol_2400[3:2];
                end

                default: begin
                end
            endcase
        end
        else begin
            symbol_timer <= symbol_timer + 32'd1;
        end
    end
end

/* ---- what tone_gen is told */
wire [31:0] carrier_increment = ORIGINATE ? increment_for_hz(16'd1200) : increment_for_hz(16'd2400);

reg [31:0] fsk_increment;

always @* begin
    if (ORIGINATE)
        fsk_increment = fsk_mark ? increment_for_hz(16'd1270) : increment_for_hz(16'd1070);
    else
        fsk_increment = fsk_mark ? increment_for_hz(16'd2225) : increment_for_hz(16'd2025);
end

assign increment    = (speed == SPEED_300) ? fsk_increment : carrier_increment;
assign phase_offset = {carrier_phase_step, 29'd0};            /* 45 degrees = 2^29 of the 2^32 of a full turn */

/* the 2400 baud signal carries two bits in four amplitude levels; every other speed has one constant amplitude */
reg [7:0] level_amplitude;

always @* begin
    case (amplitude_level)
        2'd0:    level_amplitude = 8'd24;
        2'd1:    level_amplitude = 8'd30;
        2'd2:    level_amplitude = 8'd36;
        default: level_amplitude = 8'd42;
    endcase
end

reg [7:0] voice_amplitude;

always @* begin
    if (!(active && enabled))
        voice_amplitude = 8'd0;
    else if (speed == SPEED_2400)
        voice_amplitude = level_amplitude;
    else
        voice_amplitude = 8'd40;
end

assign amplitude = voice_amplitude;

endmodule
