/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Baud rate divider for the UART bridges (FPGA only, not a chip of the 3B1). One divider serves both bridges of the core, because they are always
    set to the same speed.

    The MiSTer framework tells the core the UART speed in baud (the OSD "UART" setting, `speed`). The UARTs need it as a number of system clocks per
    bit:

        bit_clocks = CLK_HZ / speed

    A divider that works through one result bit per clock is enough, because the speed changes only when the user touches the menu. It is the
    schoolbook "restoring" long division: shift the next dividend bit into the remainder, and if the remainder is at least the divisor, subtract the
    divisor and write a 1 into the quotient, otherwise a 0. The dividend (CLK_HZ) has BIT_CLOCKS_BITS bits, so that many steps are needed.

    A speed of 0 means the line is idle: bit_clocks is 0 and the UARTs do nothing. A very high speed is limited to 8 clocks per bit; the divisor is
    therefore limited to 22 bits (above 4.19 Mbaud the result is below 8 for every clock up to 33 MHz anyway), which keeps the registers small.
*/
module uart_baud_divider
#(
    parameter CLK_HZ          = 20000000,
    parameter BIT_CLOCKS_BITS = 25                  /* wide enough for CLK_HZ (below 33.5 MHz) */
)
(
    input         clk,
    input         rst,

    input  [31:0] speed,                    /* baud rate (from hps_io) */
    output reg [BIT_CLOCKS_BITS-1:0] bit_clocks = 0     /* system clocks per bit, 0 = idle line */
);

localparam MIN_BIT_CLOCKS = 8;
localparam DIVISOR_BITS   = 22;

reg                       dividing      = 1'b0;           /* a division is running */
reg [31:0]                speed_divided = 32'd0;          /* the speed the last (or the running) division was started for */
reg [5:0]                 steps_left    = 6'd0;
reg [DIVISOR_BITS-1:0]    divisor       = 0;
reg [DIVISOR_BITS:0]      remainder     = 0;
/* holds the dividend at the start; its bits are shifted out at the top while the result bits shift in at the bottom */
reg [BIT_CLOCKS_BITS-1:0] quotient      = 0;

wire [DIVISOR_BITS:0] remainder_with_next_bit = {remainder[DIVISOR_BITS-1:0], quotient[BIT_CLOCKS_BITS-1]};
wire                  divisor_fits            = (remainder_with_next_bit >= {1'b0, divisor});

/* a speed above what 22 bits can hold is limited: the result is the minimum anyway */
wire [DIVISOR_BITS-1:0] speed_limited = (speed > {{(32-DIVISOR_BITS){1'b0}}, {DIVISOR_BITS{1'b1}}}) ? {DIVISOR_BITS{1'b1}} : speed[DIVISOR_BITS-1:0];

always @(posedge clk) begin
    if (rst) begin
        dividing      <= 1'b0;
        speed_divided <= 32'd0;
        bit_clocks    <= 0;
    end
    else if (!dividing) begin
        if (speed != speed_divided) begin
            speed_divided <= speed;

            if (speed == 32'd0) begin
                bit_clocks <= 0;                        /* idle line, nothing to divide */
            end
            else begin
                dividing   <= 1'b1;
                steps_left <= 6'd32;                    /* 32 clocks like the first version; the first ones are idle steps */
                remainder  <= 0;
                quotient   <= CLK_HZ;
                divisor    <= speed_limited;
            end
        end
    end
    else if (steps_left != 6'd0) begin
        if (steps_left > BIT_CLOCKS_BITS) begin
            /* the dividend has fewer bits than the 32 steps of the first version: nothing to shift yet */
        end
        else if (divisor_fits) begin
            remainder <= remainder_with_next_bit - {1'b0, divisor};
            quotient  <= {quotient[BIT_CLOCKS_BITS-2:0], 1'b1};
        end
        else begin
            remainder <= remainder_with_next_bit;
            quotient  <= {quotient[BIT_CLOCKS_BITS-2:0], 1'b0};
        end

        steps_left <= steps_left - 6'd1;
    end
    else begin
        bit_clocks <= (quotient < MIN_BIT_CLOCKS) ? MIN_BIT_CLOCKS : quotient;
        dividing   <= 1'b0;
    end
end

endmodule
