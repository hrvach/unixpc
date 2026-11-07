/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Everything between the video RAM and the HDMI pins: the raster that reads the video RAM, the notice shown while there is no boot ROM, the CRT
    brightness effects, the virtual line printer with its paper, and the 1280x720 canvas (video.sv) that draws the picture, the paper, the monitor
    frame and the LEDs. Alt+F1 / Alt+F2 (hotkeys.sv) choose between the screen and the paper.
*/
module video_output
(
    input         clk_sys,  /* 20 MHz: the machine */
    input         clk_hd,  /* 75 MHz: the video canvas */
    input         reset,
    input         boot_rom_ready,  /* low: the notice is shown instead of the machine */
    input  [10:0] ps2_key,  /* for the Alt+F1 / Alt+F2 hotkeys */
    output [13:0] video_ram_address,
    input  [15:0] video_ram_data,
    input         lp_wr_data,  /* the parallel printer port of the board */
    input  [7:0]  lp_data,
    input         lp_wr_mcr,
    input         lp_mcr13,
    output        lp_busy,
    output        lp_select,
    output        lp_nopaper,
    output        lp_int,
    input  [3:0]  front_panel_leds,  /* 0 = lit */
    input         osd_tear_off_paper,
    input         osd_invert_video,
    input         osd_phosphor_on,
    input  [1:0]  osd_glow_mode,
    input  [1:0]  osd_glow_noise,
    input  [1:0]  osd_noise,
    input         osd_bezel_on,
    input         osd_curvature_on,
    input  [1:0]  osd_line_wobble,
    input  [1:0]  osd_screen_colour,
    input  [7:0]  osd_glass_floor_level,
    input         osd_edges_on,
    input         osd_round_corners,
    input  [1:0]  osd_roll_band,
    output [7:0]  VGA_R,
    output [7:0]  VGA_G,
    output [7:0]  VGA_B,
    output        VGA_HS,
    output        VGA_VS,
    output        VGA_DE,
    output        CLK_VIDEO,
    output        CE_PIXEL
);

/* ---- video */
wire pix;
wire hsync;
wire vsync;
wire hblank;
wire vblank;
wire vbl_start;

wire machine_pixel = pix;

/* the picture of the machine, or the notice that there is no boot ROM */
wire        notice_pixel;
wire [9:0]  notice_font_address;
wire [9:0]  printer_font_address;
wire [7:0]  shared_font_row;

boot_rom_notice notice
(
    .clk          (clk_sys),
    .hblank       (hblank),
    .vblank       (vblank),
    .notice_pixel (notice_pixel),
    .font_address (notice_font_address),
    .font_row     (shared_font_row)
);

wire picture_pixel = boot_rom_ready ? machine_pixel : notice_pixel;

video_gen raster
(
    .clk       (clk_sys),
    .rst       (reset & boot_rom_ready),           /* the raster runs while the notice is shown */
    .vid_addr  (video_ram_address),
    .vid_data  (video_ram_data),
    .invert    (osd_invert_video),
    .pix       (pix),
    .hsync     (hsync),
    .vsync     (vsync),
    .hblank    (hblank),
    .vblank    (vblank),
    .vbl_start (vbl_start)
);

/* ---- the video output: the screen of the tube or the printer's paper on the 1008x720 canvas of video.sv */
wire show_paper;

hotkeys keys
(
    .clk        (clk_sys),
    .ps2_key    (ps2_key),
    .paper_view (show_paper)
);

wire [7:0] effects_brightness;
wire       effects_de;
wire       effects_hs;
wire       effects_vs;

/* the brightness effects at the machine's resolution (no curvature: the canvas does the geometry) */
crt_fx effects
(
    .clk        (clk_sys),
    .pix        (picture_pixel),
    .hblank     (hblank),
    .vblank     (vblank),
    .hsync      (hsync),
    .vsync      (vsync),
    .pers_on    (osd_phosphor_on),
    .glow_mode  (osd_glow_mode),
    .glow_noise (osd_glow_noise),
    .noise      (osd_noise),
    .lum        (effects_brightness),
    .de         (effects_de),
    .hs         (effects_hs),
    .vs         (effects_vs)
);

/* the parallel printer: a virtual Centronics printer with a paper view (always on line); its paper pixels come two clocks after the timing */
wire [23:0] paper_pixel;

lp_printer printer
(
    .clk     (clk_sys),
    .rst     (reset),
    .online  (1'b1),
    .tear    (osd_tear_off_paper),
    .wr_data (lp_wr_data),
    .data    (lp_data),
    .wr_mcr  (lp_wr_mcr),
    .mcr13   (lp_mcr13),
    .busy    (lp_busy),
    .select  (lp_select),
    .nopaper (lp_nopaper),
    .lpint   (lp_int),
    .de      (effects_de),
    .vs      (effects_vs),
    .rgb     (paper_pixel),
    .font_address (printer_font_address),
    .font_row     (shared_font_row)
);

/* the one font ROM: the printer's, and the notice's while there is no boot ROM (the printer is idle then) */
lp_font shared_font
(
    .clk  (clk_sys),
    .addr (boot_rom_ready ? printer_font_address : notice_font_address),
    .q    (shared_font_row)
);

video canvas
(
    .clk_machine   (clk_sys),
    .brightness    (effects_brightness),
    .picture_de    (effects_de),
    .picture_vs    (effects_vs),
    .paper_pixel   (paper_pixel),
    .show_paper    (show_paper),
    .bezel_on      (osd_bezel_on),
    .curvature_strength (osd_curvature_on ? 3'd0 : 3'd7),       /* 0.031 or off */
    .wobble        (osd_line_wobble),
    .screen_colour (osd_screen_colour),
    .glass_floor   (osd_glass_floor_level),
    .edges_on      (osd_edges_on),
    .round_corners (osd_round_corners),
    .roll_band     (osd_roll_band),
    .leds_lit      (~front_panel_leds),
    .clk_pixel     (clk_hd),
    .de            (VGA_DE),
    .hs            (VGA_HS),
    .vs            (VGA_VS),
    .rgb           ({VGA_R, VGA_G, VGA_B})
);

assign CLK_VIDEO = clk_hd;
assign CE_PIXEL  = 1'b1;

endmodule
