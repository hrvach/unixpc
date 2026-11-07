/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The brightness effects of the CRT look for the 720x348 monochrome picture, racing the beam (one 4 bit level per pixel and two line buffers, no
    frame processing). The stages, in order:

      crt_persistence  phosphor persistence: a 4 bit level per pixel (the logarithm of the light) is kept in a block RAM; a lit pixel resets it
                       to 15 and every frame it drops by two levels, an exponential fade over 8 levels, dark after 8 frames (about 133 ms)
      crt_glow         glow: 5 tap horizontal and 3 tap vertical binomial blur of the brightness, added back (glow_mode 1 = low, 2 = high, 0 = off);
                       one line of delay, because the vertical blur needs the line below
      crt_noise        the noise of the lit pixels and the slow flicker of the picture

    The geometry (barrel distortion, line wobble), the glass, the screen edges, the rounded corners and the rolling band are not here: the canvas
    (rtl/video/video.sv) does them at the resolution of the output, where they do not turn into stair steps.

    The line period counter that the glow's line buffers share is here, and so is the pixel noise LFSR, which the glow and the noise stage use.

    Output: lum 0..255 and de / hs / vs delayed by the same latency (one line plus a few clocks). The caller colours lum. Pixel n of a frame is
    addressed n = y * 720 + x; the counters restart at vertical blank, so only visible pixels touch the RAM.
*/
module crt_fx
(
    input        clk,
    input        pix,
    input        hblank,
    input        vblank,
    input        hsync,
    input        vsync,
    input        pers_on,           /* phosphor persistence on */
    input  [1:0] glow_mode,         /* 0 off, 1 low, 2 mid, 3 high */
    /* random variation of the glow (halo) brightness per pixel: 0 off, 1 low (+-25 %), 2 mid (+-37 %), 3 high (+-50 %) */
    input  [1:0] glow_noise,
    /* random brightness variation of every pixel, new every pixel and frame: 0 off, 1 low (+-6 %), 2 mid (+-9 %), 3 high (+-12 %) */
    input  [1:0] noise,

    output [7:0] lum,
    output       de,
    output       hs,
    output       vs
);

/* pixel noise: a free running LFSR, one step per pixel clock */
reg [15:0] pixel_noise = 16'h1ACE;

always @(posedge clk)
    pixel_noise <= {pixel_noise[14:0], pixel_noise[15] ^ pixel_noise[13] ^ pixel_noise[12] ^ pixel_noise[10]};

/*
    The line period counter that the glow stage uses: it counts the clocks of one line of the scan-out (LINE_CLOCKS = 896 pixel clocks, see
    video_gen.sv), free running. The glow keeps one line of data in circular buffers of exactly one line period; it reads the buffer at line_column
    and writes it one clock later, at line_column_delayed, so that a read before the write gives the previous line.
*/
localparam LINE_CLOCKS = 896;

reg [9:0] line_column         = 10'd0;
reg [9:0] line_column_delayed = 10'd0;

always @(posedge clk) begin
    line_column         <= (line_column == LINE_CLOCKS - 1) ? 10'd0 : line_column + 10'd1;
    line_column_delayed <= line_column;
end

/* ---- phosphor persistence */
wire [7:0] persistence_brightness;
wire       persistence_de;
wire       persistence_hs;
wire       persistence_vs;

crt_persistence persistence
(
    .clk            (clk),
    .pix            (pix),
    .hblank         (hblank),
    .vblank         (vblank),
    .hsync          (hsync),
    .vsync          (vsync),
    .persistence_on (pers_on),
    .brightness     (persistence_brightness),
    .de             (persistence_de),
    .hs             (persistence_hs),
    .vs             (persistence_vs)
);

/* ---- glow */
wire [7:0] glow_lum;
wire       glow_de;
wire       glow_hs;
wire       glow_vs;

crt_glow glow
(
    .clk            (clk),
    .brightness     (persistence_brightness),
    .de             (persistence_de),
    .hs             (persistence_hs),
    .vs             (persistence_vs),
    .glow_mode      (glow_mode),
    .glow_noise     (glow_noise),
    .pixel_noise    (pixel_noise),
    .column         (line_column),
    .column_delayed (line_column_delayed),
    .lum            (glow_lum),
    .de_out         (glow_de),
    .hs_out         (glow_hs),
    .vs_out         (glow_vs)
);

/* ---- one clock of delay (the latency the curvature stage, removed, was replaced by) */
reg [7:0] glow_lum_delayed = 8'd0;
reg       glow_de_delayed = 1'b0;
reg       glow_hs_delayed = 1'b0;
reg       glow_vs_delayed = 1'b0;

always @(posedge clk) begin
    glow_lum_delayed <= glow_lum;
    glow_de_delayed  <= glow_de;
    glow_hs_delayed  <= glow_hs;
    glow_vs_delayed  <= glow_vs;
end

/* ---- noise and flicker */
crt_noise noise_stage
(
    .clk         (clk),
    .lum_in      (glow_lum_delayed),
    .de_in       (glow_de_delayed),
    .hs_in       (glow_hs_delayed),
    .vs_in       (glow_vs_delayed),
    .noise       (noise),
    .pixel_noise (pixel_noise),
    .lum         (lum),
    .de          (de),
    .hs          (hs),
    .vs          (vs)
);

endmodule
