/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The notice that is shown while the core has no boot ROM (FPGA only): the boot ROM is not part of the core, it is loaded from a file, and until it
    arrives the machine is held in reset and the screen would stay black. One line of text, in the middle of the picture, says so.

    The picture position comes from hblank / vblank of video_gen, the same way as in dbg_overlay.sv. Each character is a cell of 16 x 16 pixels (the
    8 x 8 glyph of the printer's font, doubled in both directions). The 17 characters are 272 pixels wide, so the text starts at cell column 14
    (pixel 224, which centres it on the 720 pixels of the picture), in cell row 10 (lines 160 to 175 of the 348).

    The font is the ROM of the printer (lp_font.sv), shared. The glyph row comes out of the font ROM one clock after its address, so the column
    of the pixel and the "this pixel belongs to the text" flag are delayed by one clock to meet it.
*/
module boot_rom_notice
(
    input          clk,
    input          hblank,
    input          vblank,

    output         notice_pixel,                /* a lit pixel of the text */

    /* the font ROM of the printer (lp_font.sv), shared: the printer does not use it while there is no boot ROM */
    output   [9:0] font_address,
    input    [7:0] font_row
);

localparam [5:0] TEXT_COLUMNS = 6'd17;
localparam [5:0] FIRST_COLUMN = 6'd14;          /* in cells of 16 pixels */
localparam [4:0] TEXT_ROW     = 5'd10;

/* exactly 17 characters: a longer literal would lose its first letters */
localparam [8 * 17 - 1:0] MESSAGE = "NO BOOT ROM FOUND";

reg [9:0] x = 10'd0;                            /* pixel in the line */
reg [8:0] y = 9'd0;                             /* line in the frame */
reg       hblank_before = 1'b1;

always @(posedge clk) begin
    hblank_before <= hblank;
    x <= hblank ? 10'd0 : x + 10'd1;

    if (vblank)
        y <= 9'd0;
    else if (hblank && !hblank_before)          /* the end of a visible line */
        y <= y + 9'd1;
end

/* which character is under this pixel, if the pixel is in the text at all */
wire [5:0] cell_column = x[9:4];
wire [4:0] cell_row = y[8:4];
wire [5:0] text_column = cell_column - FIRST_COLUMN;

wire in_text = (cell_column >= FIRST_COLUMN) && (text_column < TEXT_COLUMNS) && (cell_row == TEXT_ROW);

wire [7:0] character = MESSAGE[8 * (TEXT_COLUMNS - 6'd1 - text_column) +: 8];

/* the row of the glyph: 16 lines of the cell show the 8 rows of the glyph twice */
assign font_address = {character[6:0], y[3:1]};

/* the column of the glyph, and whether the pixel is in the text, one clock later, when the glyph row is there */
reg [2:0] glyph_column = 3'd0;
reg       in_text_delayed = 1'b0;

always @(posedge clk) begin
    glyph_column    <= x[3:1];
    in_text_delayed <= in_text & ~hblank & ~vblank;
end

assign notice_pixel = in_text_delayed & font_row[3'd7 - glyph_column];        /* the leftmost pixel of the glyph is bit 7 */

endmodule
