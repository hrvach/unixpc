/*
    Copyright (c) 2026 by Hrvoje Cavrak

    16 KB boot ROM as the two 8 KB EPROMs of the board, 15C on the high byte of the bus and 14C on the low byte, 16 bit wide, 1 clock read latency.
    Each half is a RAM that starts with the contents of INIT_FILE when BUILT_IN is 1 (the simulators); its own write port lets the MiSTer core load
    the half from a file (rom_loader.sv). The MiSTer core is built with BUILT_IN = 0: the bitstream has no ROM in it and the RAM is empty until the
    files have arrived.
*/
module rom16k #(parameter INIT_FILE = "", parameter BUILT_IN = 1)
(
    input             clk,
    input      [12:0] addr,
    output reg [15:0] q,

    input             we_high,          /* write port of 15C */
    input      [12:0] waddr_high,
    input      [7:0]  wdata_high,
    input             we_low,           /* write port of 14C */
    input      [12:0] waddr_low,
    input      [7:0]  wdata_low
);
reg [7:0]  mem_high [0:8191];
reg [7:0]  mem_low  [0:8191];
reg [15:0] init_words [0:8191];
integer    word_index;

generate
    if (BUILT_IN) begin : g_built_in
        initial begin
            $readmemh(INIT_FILE, init_words);
            for (word_index = 0; word_index < 8192; word_index = word_index + 1) begin
                mem_high[word_index] = init_words[word_index][15:8];
                mem_low[word_index]  = init_words[word_index][7:0];
            end
        end
    end
endgenerate

always @(posedge clk) begin
    if (we_high) mem_high[waddr_high] <= wdata_high;
    if (we_low)  mem_low[waddr_low]   <= wdata_low;
    q <= {mem_high[addr], mem_low[addr]};
end
endmodule
