/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The LANCE's DMA word engine (inside the Am7990: its bus interface unit).

    The sequencer asks for one 16 bit memory access at a time (access_pending with the address, direction, data and byte enables). The engine waits
    for the bus grant, then drives the board's master port (m_req / m_we / m_addr / m_wdata / m_be) until the board answers with m_done, and reports
    the result (access_finished, access_read_data, access_error). The bus request (bus_request) stays up while an access is pending or running and for
    a few clocks after it, so that the descriptor and data accesses of a burst keep the bus.

    Byte order: descriptor and initialisation block words are natural 16 bit values whatever BSWP says. Only packet data follows BSWP: with BSWP set
    the two bytes of a data word are swapped on the way to and from the bus, and the byte enables with them.
*/
module lance_bus_master
(
    input         clk,
    input         rst,
    input         byte_swap,                    /* CSR3 BSWP */

    /* the sequencer's request */
    input         access_pending,               /* an access has been asked for and has not been given the bus yet */
    input         access_write,
    input  [23:1] access_address,
    input  [15:0] access_write_data,            /* LANCE word (before the byte swap) */
    input         access_is_packet_data,        /* packet data: the byte order follows BSWP */
    input  [1:0]  access_byte_enables,          /* bit 0 = low byte of the LANCE word */

    output        access_starts,                /* the access gets the bus in this clock: the sequencer clears access_pending */
    output        bus_request,
    output reg        access_finished = 1'b0,   /* one clock pulse: the access is done */
    output reg [15:0] access_read_data = 16'd0, /* LANCE word read */
    output reg        access_error = 1'b0,      /* the access ended with a bus error */

    /* the board's master port */
    input         bus_granted,
    output reg        m_req = 1'b0,
    output reg        m_we,
    output reg [23:1] m_addr,
    output reg [15:0] m_wdata,
    output reg [1:0]  m_be,                     /* bus byte enables: bit 1 = D15..D8 */
    input         m_done,
    input         m_fault,
    input  [15:0] m_rdata
);

localparam [7:0] BUS_HOLD_CLOCKS = 8'd6;        /* the bus request stays this many clocks after an access: bursts */

reg access_running           = 1'b0;
reg [7:0] hold_clocks_left   = 8'd0;

assign access_starts = access_pending && !access_running && bus_granted;
assign bus_request   = access_pending | access_running | (hold_clocks_left != 8'd0);

/* a data word as the LANCE sees it and as the bus sees it (the swap is its own inverse) */
function [15:0] swap_if_packet_data(input [15:0] word);
    swap_if_packet_data = (byte_swap & access_is_packet_data) ? {word[7:0], word[15:8]} : word;
endfunction

always @(posedge clk) begin
    access_finished <= 1'b0;

    if (hold_clocks_left != 8'd0 && !access_pending && !access_running)
        hold_clocks_left <= hold_clocks_left - 8'd1;

    if (access_starts) begin
        access_running <= 1'b1;
        m_req          <= 1'b1;
        m_we           <= access_write;
        m_addr         <= access_address;
        m_wdata        <= swap_if_packet_data(access_write_data);
        m_be           <= (byte_swap & access_is_packet_data) ? {access_byte_enables[0], access_byte_enables[1]} : access_byte_enables;
    end

    if (access_running && m_done) begin
        m_req            <= 1'b0;
        access_running   <= 1'b0;
        access_finished  <= 1'b1;
        hold_clocks_left <= BUS_HOLD_CLOCKS;
        access_read_data <= swap_if_packet_data(m_rdata);
        access_error     <= m_fault;
    end

    if (rst) begin
        access_running   <= 1'b0;
        m_req            <= 1'b0;
        hold_clocks_left <= 8'd0;
    end
end

endmodule
