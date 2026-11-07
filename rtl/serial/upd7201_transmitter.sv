/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Transmitter of one uPD7201 channel (asynchronous mode), character level.

    Two places hold a character, as in the chip: the holding register (what the CPU writes) and the shifter (what is on the line). A character written
    while the shifter is free goes straight into the shifter. Otherwise it waits in the holding register, and moves into the shifter when the
    character before it has left.

    The FPGA does not make the individual bits: the shifter just counts the time the character occupies the line (start + data bits + parity + stop
    bits, in serial clocks). When the time is over, the character is handed to the host side (sent / sent_data, one clock pulse).

    Interrupt: "transmit buffer empty" is pending from the moment the holding register has been emptied, until the CPU writes the next character,
    resets it with the WR0 command, or channel reset.
*/
module upd7201_transmitter
(
    input         clk,
    input         rst,

    input         channel_reset,            /* WR0 command "channel reset" */
    input         enable,                   /* WR5 bit 3 (and CTS when auto enables are on): the shifter may start a character */
    input  [31:0] clocks_per_character,     /* system clocks one character takes on the line */

    input         write_character,          /* the CPU writes the data register */
    input  [7:0]  write_data,

    input         clear_interrupt,          /* WR0 command "reset transmit interrupt pending" */
    input         raise_interrupt,          /* WR1 bit 1 was just switched on while the holding register is empty */

    output reg    interrupt_pending = 1'b0, /* transmit buffer empty */
    output reg    holding_full      = 1'b0, /* a character waits in the holding register (RR0 bit 2 is the inverse) */
    output        shifting,                 /* a character is on the line */
    output reg    underrun          = 1'b1, /* RR0 bit 6: nothing left to send */

    output        finishing,                /* the character in the shifter is complete in this clock ... */
    output [7:0]  finishing_data,           /* ... and this is it (the loopback plug feeds it back to the receiver) */
    output reg    sent              = 1'b0, /* one clock pulse, the clock after: the character left the transmitter */
    output reg [7:0] sent_data
);

reg [7:0]  holding_register;
reg [7:0]  shifter;
reg [31:0] clocks_left = 32'd0;             /* 0: shifter free; counts down to 1, the clock it is 1 the character is complete */

wire shifter_free = (clocks_left == 32'd0);

assign shifting       = ~shifter_free;
assign finishing      = (clocks_left == 32'd1);
assign finishing_data = shifter;

always @(posedge clk) begin
    sent <= 1'b0;

    if (rst) begin
        interrupt_pending <= 1'b0;
        holding_full      <= 1'b0;
        underrun          <= 1'b1;
        clocks_left       <= 32'd0;
    end
    else begin
        /* ---- the shifter: count the character down, then take the next one from the holding register */
        if (clocks_left > 32'd1) begin
            clocks_left <= clocks_left - 32'd1;
        end
        else if (finishing) begin
            clocks_left <= 32'd0;
            sent_data   <= shifter;
            sent        <= 1'b1;
        end
        else if (holding_full && enable) begin
            shifter           <= holding_register;
            holding_full      <= 1'b0;
            interrupt_pending <= 1'b1;
            clocks_left       <= clocks_per_character;
            underrun          <= 1'b0;
        end
        else if (!holding_full) begin
            underrun <= 1'b1;
        end

        /* ---- the CPU writes a character */
        if (write_character) begin
            if (shifter_free && !holding_full && enable) begin
                shifter           <= write_data;                /* straight to the line, the holding register stays empty */
                clocks_left       <= clocks_per_character;
                interrupt_pending <= 1'b1;
                underrun          <= 1'b0;
            end
            else begin
                holding_register  <= write_data;                /* buffer full again: the interrupt goes away */
                holding_full      <= 1'b1;
                interrupt_pending <= 1'b0;
            end
        end

        /* ---- WR0 commands and WR1 changes */
        if (raise_interrupt) interrupt_pending <= 1'b1;
        if (clear_interrupt) interrupt_pending <= 1'b0;

        if (channel_reset) begin
            interrupt_pending <= 1'b0;
            holding_full      <= 1'b0;
            clocks_left       <= 32'd0;
        end
    end
end

endmodule
