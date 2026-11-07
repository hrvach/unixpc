/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Transmitter of the UART bridge: characters from the 7201 model go out on the pin as 8N1 frames (FPGA only).

    A frame is the idle level 1, one start bit (0), eight data bits, least significant first, and one stop bit (1). The line is held at 1 between
    frames.

    The 7201 model can hand over characters faster than the line takes them (the guest may be programmed for a higher baud rate than the host side),
    so a 16 character FIFO sits in front of the shifter. A character that arrives with the FIFO full is dropped.
*/
module uart_transmitter
#(
    parameter BIT_CLOCKS_BITS = 25
)
(
    input         clk,
    input         rst,

    input  [BIT_CLOCKS_BITS-1:0] bit_clocks,       /* system clocks per bit from uart_baud_divider, 0 = idle line (nothing is sent) */

    input         character_valid,  /* the 7201 model sent a character */
    input  [7:0]  character,

    output reg    txd = 1'b1        /* the pin */
);

localparam FIFO_DEPTH  = 16;
localparam FRAME_BITS  = 10;        /* start + 8 data + stop */

/* ---- the FIFO: write and read pointers one bit wider than the index, so that "full" and "empty" can be told apart */
reg [7:0] fifo [0:FIFO_DEPTH-1];
reg [4:0] write_pointer = 5'd0;
reg [4:0] read_pointer  = 5'd0;

wire [4:0] characters_waiting = write_pointer - read_pointer;
wire       fifo_empty         = (characters_waiting == 5'd0);
wire       fifo_full          = (characters_waiting == FIFO_DEPTH);

/* ---- the shifter */
reg [FRAME_BITS-1:0] frame      = 10'h3ff;      /* bit 0 is on the pin; the rest shifts down, ones fill in at the top */
reg [3:0]            bits_left  = 4'd0;         /* 0: no frame is being sent */
reg [BIT_CLOCKS_BITS-1:0] clocks_left = 0;       /* until the current bit ends */

wire sending = (bits_left != 4'd0);

always @(posedge clk) begin
    if (rst) begin
        write_pointer <= 5'd0;
        read_pointer  <= 5'd0;
        bits_left     <= 4'd0;
        clocks_left   <= 0;
        txd           <= 1'b1;
    end
    else begin
        if (character_valid && !fifo_full) begin
            fifo[write_pointer[3:0]] <= character;
            write_pointer            <= write_pointer + 5'd1;
        end

        if (!sending) begin
            txd <= 1'b1;
            if (!fifo_empty && bit_clocks != 0) begin               /* take the next character from the FIFO and start its frame */
                frame         <= {1'b1, fifo[read_pointer[3:0]], 1'b0}; /* stop bit, data, start bit */
                read_pointer  <= read_pointer + 5'd1;
                bits_left     <= FRAME_BITS;
                clocks_left   <= bit_clocks - 1;
            end
        end
        else if (bit_clocks == 0) begin                             /* the speed was set to 0 in the middle of a frame: give the frame up */
            txd       <= 1'b1;
            bits_left <= 4'd0;
        end
        else begin
            txd <= frame[0];
            if (clocks_left == 0) begin                             /* this bit is over: the next one goes out */
                frame       <= {1'b1, frame[FRAME_BITS-1:1]};
                bits_left   <= bits_left - 4'd1;
                clocks_left <= bit_clocks - 1;
            end
            else begin
                clocks_left <= clocks_left - 1;
            end
        end
    end
end

endmodule
