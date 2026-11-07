/*
    Copyright (c) 2026 by Hrvoje Cavrak

    UNIX PC video scan-out: 720x348 monochrome bitmap, 20 MHz pixel clock (= the board clock, one pixel per clock).

    Word time 16 px = 800 ns, line = 56 word times = 896 px (44.8 us: 45 visible words + 11 word times of horizontal retrace, reference manual 2-80),
    field = 372 lines (24 blank lines: the refresh counter runs 3D2C..4163 = 1080 words = 24 lines of 45 words, manual 2-85), i.e. 20 MHz / (896 x
    372) = 60.00 Hz, the "60 fields per second" of the manual (2-78). The bitmap is 45 words per line, word 0 = upper left, pixel 0 of a word = bit 0
    (LSB first). vid_addr / vid_data is the read port of the video RAM (1 clock read latency, vram16.b_*).

    Visible pixels sit in hcnt 16..735 (the first word is fetched at the end of the preceding word time), all syncs are positive pulses.
*/
module video_gen
(
    input         clk,
    input         rst,
    output [13:0] vid_addr,
    input  [15:0] vid_data,
    input         invert,       /* 0: bit 1 = lit pixel */

    output reg    pix,          /* lit pixel (valid while ~blank) */
    output reg    hsync,
    output reg    vsync,
    output reg    hblank,
    output reg    vblank,
    output reg    vbl_start     /* one clk pulse at the start of vertical blank */
);

localparam H_TOT = 896, V_TOT = 372, V_ACT = 348;
localparam H_ACT0 = 16, H_ACT1 = 736;                  /* visible hcnt range [16, 736) */

reg  [9:0]  hcnt = 10'd0;
reg  [8:0]  vcnt = 9'd0;
reg  [13:0] rowbase = 14'd0;                           /* vcnt * 45 */
reg  [15:0] sr = 16'd0;

assign vid_addr = rowbase + {8'd0, hcnt[9:4]};         /* loaded at hcnt[3:0] == 15 for the word shown next */

wire hlast = (hcnt == H_TOT - 1);
wire vlast = (vcnt == V_TOT - 1);

always @(posedge clk) begin
    vbl_start <= 1'b0;
    if (rst) begin
        hcnt    <= 10'd0;
        vcnt    <= 9'd0;
        rowbase <= 14'd0;
    end else begin
        hcnt <= hlast ? 10'd0 : hcnt + 10'd1;
        if (hlast) begin
            vcnt    <= vlast ? 9'd0 : vcnt + 9'd1;
            rowbase <= vlast ? 14'd0 : rowbase + 14'd45;
            if (vcnt == V_ACT - 1) vbl_start <= 1'b1;
        end
    end

    /* pixel shifter */
    if (hcnt[3:0] == 4'd15) sr <= vid_data;
    else                    sr <= {1'b0, sr[15:1]};
    pix <= sr[0] ^ invert;

    /* blanking / sync, registered together with the pixel (one clock after hcnt) */
    hblank <= ~((hcnt >= H_ACT0) && (hcnt < H_ACT1));
    vblank <= (vcnt >= V_ACT);
    hsync  <= (hcnt >= 10'd760) && (hcnt < 10'd840);
    vsync  <= (vcnt >= 9'd366) && (vcnt < 9'd370);        /* after the 16 lines below the picture (lines 348..363, not shown) */
end

endmodule
