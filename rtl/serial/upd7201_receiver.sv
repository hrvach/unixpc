/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Receiver of one uPD7201 channel (asynchronous mode), character level.

    The FPGA does not see the individual bits on the line: the host side hands over a whole character (host_valid / host_data) and the receiver keeps
    it in the shift register for as long as the real chip would need to assemble it (start bit + data bits + parity + stop bits, in serial clocks).
    When that time is over the character moves to the FIFO. If the FIFO is still full then, the character is lost and the overrun flag is set (RR1 bit
    5).
*/
module upd7201_receiver
(
    input         clk,
    input         rst,

    input         channel_reset,            /* WR0 command "channel reset": drop the character in the shifter and clear the overrun */
    input         clear_overrun,            /* WR0 command "error reset" */

    input  [1:0]  data_bits_code,           /* WR3 bits 7:6: receive character length 5 / 7 / 6 / 8 bits */
    input  [31:0] clocks_per_character,     /* system clocks one character takes on the line */

    input         accept,                   /* the host character on host_data is taken now (only given while idle) */
    input  [7:0]  host_data,

    input         fifo_full,

    output        idle,                     /* nothing in the shifter: a new character can be accepted */
    output        character_done,           /* the character in the shifter is complete this clock */
    output [7:0]  character,                /* ... and this is it */
    output reg    overrun = 1'b0            /* a complete character found the FIFO full */
);

localparam LENGTH_5_BITS = 2'b00;
localparam LENGTH_7_BITS = 2'b01;
localparam LENGTH_6_BITS = 2'b10;           /* (WR3 and WR5 do not order the lengths by size) */

reg [7:0]  shifter;
reg [31:0] clocks_left = 32'd0;             /* 0: empty; counts down to 1, the clock it is 1 the character is complete */

assign idle           = (clocks_left == 32'd0);
assign character_done = (clocks_left == 32'd1);
assign character      = shifter;

/* Bits above the character length read as zero. */
function [7:0] mask_to_length(input [7:0] raw, input [1:0] code);
    case (code)
        LENGTH_5_BITS: mask_to_length = raw & 8'h1f;
        LENGTH_7_BITS: mask_to_length = raw & 8'h7f;
        LENGTH_6_BITS: mask_to_length = raw & 8'h3f;
        default:       mask_to_length = raw;
    endcase
endfunction

always @(posedge clk) begin
    if (rst) begin
        overrun     <= 1'b0;
        clocks_left <= 32'd0;
    end
    else begin
        if (clocks_left > 32'd1) begin
            clocks_left <= clocks_left - 32'd1;
        end
        else if (character_done) begin
            clocks_left <= 32'd0;
            if (fifo_full) overrun <= 1'b1;
        end

        if (accept) begin
            shifter     <= mask_to_length(host_data, data_bits_code);
            clocks_left <= clocks_per_character;
        end

        if (clear_overrun || channel_reset) overrun     <= 1'b0;
        if (channel_reset)                  clocks_left <= 32'd0;
    end
end

endmodule
