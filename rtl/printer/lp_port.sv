/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Centronics line printer port of the logic board: the CPU side of the parallel port.

      4F0000 (write)  printer data register: the byte to print
      4A0000 (write)  MCR bit 13 = LPSTROBE: the CPU writes the data register, then toggles the strobe 1 -> 0 (falling edge = "here is a byte")
      470000 (read)   status: D7 busy, D6 select (printer on line), D5 no paper, D4 error*
      LPINT           level 2 interrupt (shared with the disk): the printer raises it when it has taken the byte and is ready for the next one

    Handshake: the falling edge of LPSTROBE hands the byte to the printer; the printer is busy for ACK_CLOCKS (20 us at 20 MHz, a typical line
    printer's acknowledge time), then drops busy and raises LPINT. The interrupt is cleared by the next write of the data register and by either edge
    of the strobe. An off-line printer ignores the strobe: it is never busy and never acknowledges.

    What is behind the port is not part of the hardware: the "printer" is the virtual one in lp_paper_text.sv. This module only talks to it through
    printed_byte / byte_ready / byte_taken and waits while it is busy clearing the paper (paper_clearing).
*/
module lp_port
#(
    parameter ACK_CLOCKS = 400                  /* busy time per byte: 20 us at 20 MHz */
)
(
    input            clk,
    input            rst,

    input            online,                    /* printer on line (OSD switch) */

    /* CPU side */
    input            wr_data,                   /* CPU writes the data register (one clock) */
    input      [7:0] data,                      /* the byte written */
    input            wr_mcr,                    /* CPU writes the MCR (one clock) */
    input            mcr13,                     /* MCR bit 13 as written: LPSTROBE */

    /* status lines and interrupt */
    output           busy,                      /* status D7 */
    output           select,                    /* status D6 */
    output           nopaper,                   /* status D5 (the virtual paper never runs out) */
    output reg       lpint = 1'b0,                     /* interrupt request */

    /* virtual printer side */
    output reg [7:0] printed_byte = 8'd0,              /* the byte in the data register */
    output reg       byte_ready = 1'b0,                /* a strobe came and the byte has not been taken yet */
    input            byte_taken,                /* the virtual printer takes the byte this clock */
    input            paper_clearing             /* the virtual printer is still clearing paper: no acknowledge yet */
);

reg        strobe_was_high = 1'b1;              /* MCR bit 13 as written the last time, to find its edges */
reg        busy_counting   = 1'b0;
reg [8:0]  busy_clocks_left = 9'd0;

assign busy    = busy_counting;
assign select  = online;
assign nopaper = 1'b0;

wire strobe_falling = strobe_was_high & ~mcr13;                 /* valid together with wr_mcr */
wire strobe_rising  = ~strobe_was_high & mcr13;

always @(posedge clk) begin
    if (wr_data) begin
        printed_byte <= data;
        lpint        <= 1'b0;                                   /* a new byte clears the acknowledge */
    end

    if (wr_mcr) begin
        strobe_was_high <= mcr13;
        if (strobe_falling) begin
            byte_ready <= 1'b1;                                 /* LPSTROBE: the byte is handed over */
            lpint      <= 1'b0;
        end
        if (strobe_rising)
            lpint <= 1'b0;
    end

    if (busy_counting) begin
        busy_clocks_left <= busy_clocks_left - 9'd1;
        if (busy_clocks_left == 9'd1 && !paper_clearing) begin
            busy_counting <= 1'b0;                              /* the printer is ready again ... */
            lpint         <= online;                            /* ... and says so */
        end
    end

    if (byte_taken) begin
        byte_ready <= 1'b0;
        if (online) begin
            busy_counting    <= 1'b1;
            busy_clocks_left <= ACK_CLOCKS;
        end
    end

    if (rst) begin
        byte_ready      <= 1'b0;
        busy_counting   <= 1'b0;
        lpint           <= 1'b0;
        strobe_was_high <= 1'b1;
    end
end

endmodule
