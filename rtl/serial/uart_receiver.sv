/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Receiver of the UART bridge: 8N1 frames from the pin become characters for the 7201 model (FPGA only).

    The pin is first passed through a three stage synchroniser (it comes from outside and has no relation to the system clock). The receiver then
    waits for the falling edge of a start bit, looks at the line in the middle of the start bit (a short glitch is thrown away), and then samples the
    eight data bits and the stop bit, one bit time apart, each in the middle of its bit.

    One finished character is held until the 7201 model takes it (character_ready). A character that is finished while the last one is still waiting,
    or a frame without a stop bit, is dropped.
*/
module uart_receiver
#(
    parameter BIT_CLOCKS_BITS = 25
)
(
    input         clk,
    input         rst,

    input  [BIT_CLOCKS_BITS-1:0] bit_clocks,           /* system clocks per bit from uart_baud_divider, 0 = idle line (nothing is received) */

    output reg    character_valid = 1'b0,   /* a character is waiting */
    output reg [7:0] character,
    input         character_ready,          /* the 7201 model takes it */

    input         rxd                       /* the pin */
);

/*
    Where the receiver is in a frame. bits_to_go counts down: IDLE = no frame; START_BIT = checking the middle of the start bit; then the eight data
    bits (9 down to 2); STOP_BIT = the stop bit.
*/
localparam IDLE      = 4'd0;
localparam STOP_BIT  = 4'd1;
localparam FIRST_DATA_BIT = 4'd9;
localparam START_BIT = 4'd10;

reg [2:0]  rxd_synchronised = 3'b111;
wire       line             = rxd_synchronised[2];

reg [3:0]  bits_to_go   = IDLE;
reg [BIT_CLOCKS_BITS-1:0] clocks_left = 0;            /* until the middle of the next bit */
reg [7:0]  shifter      = 8'd0;             /* data bits come in at the top and move down: the first bit received ends up in bit 0 */

always @(posedge clk) begin
    rxd_synchronised <= {rxd_synchronised[1:0], rxd};

    if (rst) begin
        bits_to_go      <= IDLE;
        character_valid <= 1'b0;
    end
    else begin
        if (character_valid && character_ready) character_valid <= 1'b0;

        if (bits_to_go == IDLE) begin
            if (!line && bit_clocks != 0) begin                     /* falling edge of a start bit: look again in the middle of it */
                bits_to_go  <= START_BIT;
                clocks_left <= bit_clocks >> 1;
            end
        end
        else if (bit_clocks == 0) begin                             /* the speed was set to 0 in the middle of a frame: give the frame up */
            bits_to_go <= IDLE;
        end
        else if (clocks_left != 0) begin
            clocks_left <= clocks_left - 1;
        end
        else begin
            clocks_left <= bit_clocks - 1;                          /* from now on one bit time to the middle of the next bit */
            case (bits_to_go)
                START_BIT: begin
                    if (line) bits_to_go <= IDLE;                       /* the line is high again: a glitch, not a start bit */
                    else      bits_to_go <= FIRST_DATA_BIT;
                end
                STOP_BIT: begin
                    bits_to_go <= IDLE;
                    if (line && !character_valid) begin                 /* a good stop bit and room for the character */
                        character       <= shifter;
                        character_valid <= 1'b1;
                    end
                end
                default: begin                                          /* one of the eight data bits */
                    shifter    <= {line, shifter[7:1]};
                    bits_to_go <= bits_to_go - 4'd1;
                end
            endcase
        end
    end
end

endmodule
