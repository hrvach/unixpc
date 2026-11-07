/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Receive data FIFO of one uPD7201 channel: three characters deep, oldest character at the head.

    The CPU reads the head (RR8, the data register); the read removes it and the others move up. A new character is stored behind the last one.
    Reading and storing may happen in the same clock: the head leaves and the new character takes the place behind the others. The caller never pushes
    into a full FIFO (a character that completes while the FIFO is full is a receive overrun, see upd7201_receiver.sv).
*/
module upd7201_rx_fifo
(
    input        clk,
    input        rst,

    input        clear,         /* channel reset command: forget all characters */
    input        push,          /* store push_data behind the last character */
    input  [7:0] push_data,
    input        pop,           /* the CPU read the data register: drop the head */

    output [1:0] count,         /* 0..3 characters held */
    output       not_empty,
    output       full,
    output [7:0] head           /* oldest character; 00 when empty (what the CPU reads from an empty FIFO) */
);

localparam DEPTH = 3;

reg [7:0] slot [0:DEPTH-1];     /* slot[0] is the head */
reg [1:0] used = 2'd0;

assign count     = used;
assign not_empty = (used != 2'd0);
assign full      = (used == DEPTH);
assign head      = not_empty ? slot[0] : 8'h00;

always @(posedge clk) begin
    if (rst || clear) begin
        used <= 2'd0;
    end
    else if (pop && not_empty) begin
        slot[0] <= slot[1];
        slot[1] <= slot[2];
        if (push) slot[used - 2'd1] <= push_data;       /* the characters moved up by one, the new one lands behind them */
        else      used <= used - 2'd1;                  /* pushed and popped in the same clock: the number of characters stays */
    end
    else if (push) begin
        slot[used] <= push_data;
        used       <= used + 2'd1;
    end
end

endmodule
