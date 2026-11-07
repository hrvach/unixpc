/* Copyright (c) 2026 by Hrvoje Cavrak */
`timescale 1ns / 1ps

/*
    Bus arbiter, PAL 16R8A, IC 25B - sheet 2 (manual appendix A, "Arbitor"), equations as printed there. Registered outputs, active low, clocked on
    the falling edge of the processor clock on the board. In the FPGA the module is clocked with the system clock instead (clk, rising edge); the
    grants therefore settle a few times faster than on the board, nothing else changes. The 68010 is not part of the equations: it gets the bus only
    while no DMA device requests it. The FPGA CPU core needs a BR/BG handshake to let go of the bus, which the board wrapper does around this module
    (it feeds "no cycle in progress" in as nENRAS). Priority: disk > expansion 3 > refresh > expansion 2 > 1 > 0.
*/

module arbiter
(
   input clk,              /* arbitration clock (the board uses the falling edge of the processor clock) */
   input nRFRQ,            /* Refresh Request */
   input nEXP3RQ,          /* Expansion Board 3 Request */
   input nDKRQ,            /* Disk Bus Request */
   input nEXP2RQ,          /* Expansion Board 2 Request */
   input nEXP1RQ,          /* Expansion Board 1 Request */
   input nEXP0RQ,          /* Expansion Board 0 Request */
   input nBGACK,           /* Bus Grant Acknowledge */
   input nENRAS,           /* Enable Row Address Strobe */
   
   output reg OE,          /* Output Enable */
   output reg nNRFBGC,     /* Nonrefresh / Bus Grant Common */
   output reg nBGC,        /* Bus Grant Common (any bus master except 68010) */
   output reg nEXP0BG,     /* Expansion Board 0 Bus Grant */
   output reg nEXP1BG,     /* Expansion Board 1 Bus Grant */
   output reg nEXP2BG,     /* Expansion Board 2 Bus Grant */
   output reg nDKBG,       /* Disk Drive Bus Grant */
   output reg nEXP3BG,     /* Expansion Board 3 Bus Grant */
   output reg nRFBG        /* Refresh Bus Grant */
);

   /* [bus grant] = [bus request ] X [bus not in use] X [priority term] + [latch term] */

wire bus_not_in_use = nENRAS & nBGACK;

initial begin
   OE      = 1'b1;
   nNRFBGC = 1'b1;
   nBGC    = 1'b1;
   nEXP0BG = 1'b1;
   nEXP1BG = 1'b1;
   nEXP2BG = 1'b1;
   nDKBG   = 1'b1;
   nEXP3BG = 1'b1;
   nRFBG   = 1'b1;
end

always @(posedge clk) begin

nDKBG   <= ~((~nDKRQ & bus_not_in_use) | (~nDKBG & ~nBGACK));
nEXP3BG <= ~((~nEXP3RQ & bus_not_in_use & nDKRQ) | (~nEXP3BG & ~nBGACK));
nRFBG   <= ~((~nRFRQ & bus_not_in_use & nDKRQ & nEXP3RQ) | (~nRFBG & ~nBGACK));
nEXP2BG <= ~((~nEXP2RQ & bus_not_in_use & nDKRQ & nEXP3RQ & nRFRQ) | (~nEXP2BG & ~nBGACK));
nEXP1BG <= ~((~nEXP1RQ & bus_not_in_use & nDKRQ & nEXP3RQ & nEXP2RQ & nRFRQ) | (~nEXP1BG & ~nBGACK));
nEXP0BG <= ~((~nEXP0RQ & bus_not_in_use & nDKRQ & nEXP3RQ & nEXP2RQ & nEXP1RQ & nRFRQ) | (~nEXP0BG & ~nBGACK));

nBGC    <= ~((~nRFRQ | ~nEXP3RQ | ~nDKRQ | ~nEXP2RQ | ~nEXP1RQ | ~nEXP0RQ) & bus_not_in_use | (~nBGC & ~nBGACK));
nNRFBGC <= ~((~nEXP3RQ | ~nDKRQ | ~nEXP2RQ | ~nEXP1RQ | ~nEXP0RQ) & (bus_not_in_use & nRFRQ) | (~nNRFBGC & ~nBGACK));

end


endmodule
