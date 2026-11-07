/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Phosphor persistence of the CRT effects (FPGA only).

    A 4 bit level per pixel is kept in a block RAM (one entry for each of the 720 x 348 pixels, addressed by the pixel number n = y * 720 + x; the
    counter restarts at vertical blank, so only visible pixels touch the RAM). The level is the logarithm of the light: a lit pixel sets it to 15 and
    every frame it drops by two levels (15 13 11 9 7 5 3, then 0). Dropping the level by a constant multiplies the light by a constant, an exponential
    fade over 8 levels (brightness 255 198 153 118 92 71 55 0), dark after 8 frames (about 133 ms). A fast moving mouse pointer leaves a short tail,
    not a row of footprints.

    Pipeline: the RAM is read at stage 0 (data at stage 1), the new level is worked out at stage 1 and written at stage 2, where the brightness of the
    pixel is also output. The brightness is 0 outside the picture so that the blur that follows does not pick up junk.
*/
module crt_persistence
(
    input             clk,
    input             pix,                  /* lit pixel at the input (valid while not blanked) */
    input             hblank,
    input             vblank,
    input             hsync,
    input             vsync,
    input             persistence_on,

    output reg [7:0]  brightness = 8'd0,    /* 0..255, stage 2 */
    output reg        de = 1'b0,            /* visible pixel */
    output reg        hs = 1'b0,
    output reg        vs = 1'b0
);

/* the brightness of a level: a gamma curve (power 2.2) of the 4 bit level, 16 entries */
localparam [127:0] GAMMA_TABLE = 128'hffe0c6ae998776685c51473f37302b00;

function [7:0] crt_gamma(input [3:0] level);
    crt_gamma = GAMMA_TABLE[level * 8 +: 8];
endfunction

localparam PIXELS_PER_FRAME = 720 * 348;
localparam [3:0] LEVEL_LIT          = 4'd15;
localparam [3:0] LEVEL_DROP_PER_FRAME = 4'd2;
localparam [3:0] LEVEL_LAST_VISIBLE = 4'd3;         /* a level of 3 or less fades to 0 */

wire visible = ~hblank & ~vblank;

/* ---- stage 0: the number of the pixel at the input */
reg [17:0] pixel_number = 18'd0;

always @(posedge clk) begin
    if (vblank)
        pixel_number <= 18'd0;
    else if (visible)
        pixel_number <= pixel_number + 18'd1;
end

/* ---- the persistence RAM: read at stage 0 (data at stage 1), written at stage 2 */
reg [3:0]  levels [0:PIXELS_PER_FRAME - 1];
reg [3:0]  level_read = 4'd0;
reg        write_enable = 1'b0;
reg [17:0] write_address = 18'd0;
reg [3:0]  write_level = 4'd0;

always @(posedge clk) begin
    if (write_enable)
        levels[write_address] <= write_level;

    level_read <= levels[pixel_number];
end

/* ---- stage 1 */
reg pix_1 = 1'b0;
reg visible_1 = 1'b0;
reg hsync_1 = 1'b0;
reg vsync_1 = 1'b0;

always @(posedge clk) begin
    pix_1     <= pix;
    visible_1 <= visible;
    hsync_1   <= hsync;
    vsync_1   <= vsync;
end

/* one frame of decay: a lit pixel is set to the top level, any other drops by two levels, or goes dark when persistence is off */
reg [3:0] level_new;

always @* begin
    if (pix_1)
        level_new = LEVEL_LIT;
    else if (persistence_on && level_read > LEVEL_LAST_VISIBLE)
        level_new = level_read - LEVEL_DROP_PER_FRAME;
    else
        level_new = 4'd0;
end

/* the brightness of the pixel: the gamma curve of its level, or plain on / off when persistence is off */
reg [7:0] pixel_brightness;

always @* begin
    if (!visible_1)
        pixel_brightness = 8'd0;
    else if (persistence_on)
        pixel_brightness = crt_gamma(level_new);
    else if (pix_1)
        pixel_brightness = 8'd255;
    else
        pixel_brightness = 8'd0;
end

/* ---- stage 2: store the new level, output the brightness of the pixel */
reg [17:0] pixel_number_1 = 18'd0;

always @(posedge clk) begin
    pixel_number_1 <= pixel_number;
    write_enable   <= visible_1;
    write_address  <= pixel_number_1;
    write_level    <= level_new;
    brightness     <= pixel_brightness;
    de             <= visible_1;
    hs             <= hsync_1;
    vs             <= vsync_1;
end

endmodule
