/*
    Copyright (c) 2026 by Hrvoje Cavrak

    MC6850 ACIA, the keyboard port (logic board sheet 7). The CPU sees two registers at E70000 (status / control) and E70002 (data), on the upper data
    byte.

    The keyboard controller of the real machine talks to the ACIA over a 9600 baud serial line. This model has no bit timing: a character from the
    keyboard controller (received_byte with a received_strobe pulse) goes straight into the receive data register, and a character the CPU writes is
    handed on through transmit_strobe / transmitted_byte. The only timing kept is that the transmitter stays busy (TDRE low) for one character time
    after a write, about 1 ms, because the kernel waits for it.

    Status register (read at E70000):  bit 7 IRQ, bit 1 TDRE (transmit data register empty), bit 0 RDRF (receive data register full). Bits 6..2
    read 0: parity and framing errors do not occur, overrun (bit 5) is tracked internally but reads 0 as before, DCD and CTS are tied active (bits 2
    and 3 = 0).

    Control register (written at E70000): bits 1..0 counter divide (11 = master reset), bits 6..5 transmit control (01 = transmit interrupt enabled),
    bit 7 receive interrupt enable.
*/
module acia6850
(
    input        clk,
    input        rst,

    input        chip_select,           /* the address decoder selected the keyboard port */
    input        register_select,       /* RS pin: 0 = status / control register, 1 = data register */
    input        read_pulse,            /* one clock pulse per CPU read cycle */
    input        write_pulse,           /* one clock pulse per CPU write cycle */
    input  [7:0] write_data,            /* D15:D8 of the CPU bus */
    output [7:0] read_data,

    input  [7:0] received_byte,         /* character from the keyboard controller */
    input        received_strobe,       /* high while a new character is presented (one edge per character) */

    output       transmit_strobe,       /* one clock pulse: the CPU wrote a character to the transmit data register */
    output [7:0] transmitted_byte,      /* that character (mouse enable / disable commands) */

    output       irq                    /* IRQ* pin, here active high: level 3 request */
);

/* One character time on the keyboard line: 10 bits at 9600 baud = about 1 ms = 20 000 clocks of 20 MHz. */
localparam [15:0] CHARACTER_TIME_CLOCKS = 16'd20000;

localparam [1:0]  COUNTER_DIVIDE_MASTER_RESET = 2'b11;
localparam [1:0]  TRANSMIT_CONTROL_INTERRUPT_ENABLED = 2'b01;

reg [7:0]  control_register          = 8'h00;
reg        receive_data_full         = 1'b0;     /* RDRF */
reg        transmit_data_empty       = 1'b1;     /* TDRE */
reg        receiver_overrun          = 1'b0;     /* OVRN */
reg [7:0]  receive_data_register     = 8'h00;
reg        received_strobe_previous  = 1'b0;
reg        transmit_pulse            = 1'b0;
reg [15:0] transmit_busy_clocks_left = 16'd0;    /* TDRE stays low while this counts down */
reg [7:0]  transmit_data_register    = 8'h00;

wire receive_interrupt_enable       = control_register[7];
wire transmit_interrupt_enable      = (control_register[6:5] == TRANSMIT_CONTROL_INTERRUPT_ENABLED);

wire new_character_arrives          = ~received_strobe_previous & received_strobe;
wire data_register_read             = chip_select & read_pulse  & register_select;
wire control_register_written       = chip_select & write_pulse & ~register_select;
wire data_register_written          = chip_select & write_pulse &  register_select;

/*
    The interrupt request: a received character (or an overrun) with the receive interrupt enabled, or an empty transmitter with the transmit
    interrupt enabled.
*/
assign irq = (receive_interrupt_enable  & (receive_data_full | receiver_overrun))
           | (transmit_interrupt_enable & transmit_data_empty);

assign read_data = register_select ? receive_data_register
                                   : {irq, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, transmit_data_empty, receive_data_full};

assign transmit_strobe  = transmit_pulse;
assign transmitted_byte = transmit_data_register;

always @(posedge clk) begin
    received_strobe_previous <= received_strobe;
    transmit_pulse           <= 1'b0;

    if (rst) begin
        control_register          <= 8'h00;
        receive_data_full         <= 1'b0;
        transmit_data_empty       <= 1'b1;
        receiver_overrun          <= 1'b0;
        receive_data_register     <= 8'h00;
        transmit_busy_clocks_left <= 16'd0;
    end else begin
        /* The transmitter becomes ready again one character time after a write. */
        if (transmit_busy_clocks_left != 16'd0) begin
            transmit_busy_clocks_left <= transmit_busy_clocks_left - 16'd1;

            if (transmit_busy_clocks_left == 16'd1)
                transmit_data_empty <= 1'b1;
        end

        /* A character arrives. If the previous one was not read yet it stays and the new one is lost (overrun). */
        if (new_character_arrives) begin
            if (receive_data_full) begin
                receiver_overrun <= 1'b1;
            end else begin
                receive_data_register <= received_byte;
                receive_data_full     <= 1'b1;
            end
        end

        /* Reading the data register empties it and clears the overrun. (Later in the clock than the arrival: a read in the same clock wins.) */
        if (data_register_read) begin
            receive_data_full <= 1'b0;
            receiver_overrun  <= 1'b0;
        end

        if (control_register_written) begin
            control_register <= write_data;

            /* Master reset: empty receiver, ready transmitter. */
            if (write_data[1:0] == COUNTER_DIVIDE_MASTER_RESET) begin
                receive_data_full         <= 1'b0;
                receiver_overrun          <= 1'b0;
                transmit_data_empty       <= 1'b1;
                transmit_busy_clocks_left <= 16'd0;
            end
        end

        /* The CPU sends a character: hand it on and keep the transmitter busy for one character time. */
        if (data_register_written) begin
            transmit_pulse            <= 1'b1;
            transmit_data_register    <= write_data;
            transmit_data_empty       <= 1'b0;
            transmit_busy_clocks_left <= CHARACTER_TIME_CLOCKS;
        end
    end
end

endmodule
