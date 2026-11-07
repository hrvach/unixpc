/*
    Copyright (c) 2026 by Hrvoje Cavrak

    32 KB bitmap RAM (720x348, 45 words per line). Port A: CPU, port B: scan-out.

    Two implementations with the same behaviour (registered outputs, a read in the same cycle as a write returns the old data): simulation
    (Verilator): one 16 bit array `mem` (read by the testbenches for screenshots) Quartus (ALTERA_RESERVED_QIS) or -DSYNTH_RAM: four simple dual port
    byte RAMs, a copy per read port (A and B) and per byte lane, so that each
       one maps onto M10K blocks. The single array with byte enable part-select writes and two read ports was not inferred as RAM (262k registers).
       Both copies of a lane get the same writes, so the B copies hold the picture.
*/
`ifdef ALTERA_RESERVED_QIS
 `define VRAM_M10K
`endif
`ifdef SYNTH_RAM
 `define VRAM_M10K
`endif

module vram16
(
    input             clk,
    input  [13:0]     a_addr,
    input             a_we,
    input  [1:0]      a_be,
    input  [15:0]     a_wdata,
    output reg [15:0] a_q,
    input  [13:0]     b_addr,
    output reg [15:0] b_q
);
`ifdef VRAM_M10K
wire [15:0] aq, bq;
vram_sdp8 ra_hi
(
    .clk   (clk),
    .we    (a_we & a_be[1]),
    .waddr (a_addr),
    .wdata (a_wdata[15:8]),
    .raddr (a_addr),
    .q     (aq[15:8])
);
vram_sdp8 ra_lo
(
    .clk   (clk),
    .we    (a_we & a_be[0]),
    .waddr (a_addr),
    .wdata (a_wdata[7:0]),
    .raddr (a_addr),
    .q     (aq[7:0])
);
vram_sdp8 rb_hi
(
    .clk   (clk),
    .we    (a_we & a_be[1]),
    .waddr (a_addr),
    .wdata (a_wdata[15:8]),
    .raddr (b_addr),
    .q     (bq[15:8])
);
vram_sdp8 rb_lo
(
    .clk   (clk),
    .we    (a_we & a_be[0]),
    .waddr (a_addr),
    .wdata (a_wdata[7:0]),
    .raddr (b_addr),
    .q     (bq[7:0])
);
always @* begin
    a_q = aq;
    b_q = bq;
end
`else
reg [15:0] mem [0:16383] /*verilator public_flat_rw*/;
always @(posedge clk) begin
    if (a_we) begin
        if (a_be[1]) mem[a_addr][15:8] <= a_wdata[15:8];
        if (a_be[0]) mem[a_addr][7:0]  <= a_wdata[7:0];
    end
    a_q <= mem[a_addr];
    b_q <= mem[b_addr];
end
`endif
endmodule

`ifdef VRAM_M10K
/*
    16384 x 8 simple dual port RAM, one clock, registered read. A read of the address being written returns the old data in simulation; the hardware
    result for that case is not defined (no_rw_check), which does not matter here.
*/
module vram_sdp8
(
    input             clk,
    input             we,
    input  [13:0]     waddr,
    input  [7:0]      wdata,
    input  [13:0]     raddr,
    output reg [7:0]  q
);
(* ramstyle = "M10K, no_rw_check" *) reg [7:0] mem [0:16383];
always @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
    q <= mem[raddr];
end
endmodule
`endif
