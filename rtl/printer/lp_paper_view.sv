/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The paper on the screen (FPGA only): fan-fold printer paper with green bars and sprocket holes, drawn instead of the machine's own picture while
    the OSD shows the printer.

    The module follows the picture it is part of: `de` is high for the visible pixels of a line, `vs` pulses at the start of the vertical blanking.
    Pixels are counted from that, one pixel per clock, 720 pixels per line:

        x = 0 ......... 39 | 40 ......................... 679 | 680 ......... 719
            sprocket strip |  the sheet: 80 columns of 8 pixels | sprocket strip

    the sheet shows 32 text lines of a 10 pixel pitch (an 8 pixel glyph and 2 blank lines) starting at picture line 14; the text comes from
    lp_paper_text.sv: the ring row of screen line r is r + head + 1 (the oldest line at the top, the line being typed at the bottom); paper is white,
    with a green bar every other three text lines; the bars move up with the text (line_feed_phase), black type; every 32 picture lines a round
    sprocket hole, 14 pixels across and 10 lines high (round on the canvas), is punched into each strip. The glyph row of the character in the NEXT 8 pixel cell is fetched while the current
    cell is drawn (text RAM, then font ROM, then a shift register).
*/
module lp_paper_view
import paper_palette_pkg::*;
(
    input             clk,

    /* picture timing of the output (1 = visible pixel / sync pulse) */
    input             de,
    input             vs,

    /* paper state from lp_paper_text.sv */
    input      [4:0]  head_row,
    input      [2:0]  line_feed_phase,
    output reg [11:0] text_read_address = 12'd0,    /* {row, column} of the character wanted */
    input      [6:0]  text_read_data,                /* the character, one clock later */

    /* the font ROM (lp_font.sv, outside: the core shares it with the notice for a missing boot ROM), the glyph row comes one clock after the address */
    output     [9:0]  font_address,
    input      [7:0]  font_row,

    output reg [23:0] rgb = 24'd0
);

/*
    ---- geometry and colours
*/
localparam SHEET_LEFT_X        = 40;            /* first pixel of the sheet (the sprocket strip is to the left of it) */
localparam SHEET_RIGHT_X       = 680;           /* first pixel right of the sheet */
localparam FIRST_SHEET_CELL    = 5;             /* the sheet starts at cell 5 (= pixel 40); column 0 is cell 5 */
localparam LAST_SHEET_CELL     = 84;
localparam LAST_BLANK_LINE_BEFORE_TEXT = 13;    /* the next picture line is the first text line (line 14) */
localparam TEXT_LINES          = 32;
localparam LINE_PITCH          = 10;            /* 8 glyph rows + 2 blank lines */
localparam GLYPH_ROWS          = 8;

/*
    ---- where in the picture we are
*/
reg [9:0] pixel_x = 10'd0;                      /* 0..719 while de is high */
reg [8:0] picture_line = 9'd0;                  /* counted from the vertical sync */
reg       de_previous = 1'b0;
reg       in_text_area = 1'b0;                  /* the current picture line belongs to one of the 32 text lines */
reg [3:0] line_in_pitch = 4'd0;                 /* 0..9 inside a text line's pitch; the glyph rows are 0..7 */
reg [4:0] screen_line = 5'd0;                   /* text line 0..31, top to bottom */

always @(posedge clk) begin
    de_previous <= de;
    pixel_x     <= de ? pixel_x + 10'd1 : 10'd0;

    if (vs) begin
        picture_line  <= 9'd0;
        in_text_area  <= 1'b0;
        line_in_pitch <= 4'd0;
        screen_line   <= 5'd0;
    end
    else if (de_previous && !de) begin              /* end of a visible line */
        picture_line <= picture_line + 9'd1;
        if (picture_line == LAST_BLANK_LINE_BEFORE_TEXT) begin
            in_text_area  <= 1'b1;
            line_in_pitch <= 4'd0;
            screen_line   <= 5'd0;
        end
        else if (in_text_area) begin
            if (line_in_pitch == LINE_PITCH - 1) begin
                line_in_pitch <= 4'd0;
                screen_line   <= screen_line + 5'd1;
                if (screen_line == TEXT_LINES - 1)
                    in_text_area <= 1'b0;
            end
            else
                line_in_pitch <= line_in_pitch + 4'd1;
        end
    end
end

/*
    ---- the glyph pipeline
*/
wire [6:0] cell_number   = pixel_x[9:3];                    /* 8 pixel cell of the picture, 0..89 */
wire [2:0] pixel_in_cell = pixel_x[2:0];

/* the NEXT cell is cell_number + 1, and cell 5 is column 0: column = cell_number + 1 - 5 */
wire [6:0] next_cell_column = cell_number - 7'd4;
wire       fetching_glyph_row = in_text_area && (line_in_pitch < GLYPH_ROWS) &&
                                (cell_number >= FIRST_SHEET_CELL - 1) && (cell_number < LAST_SHEET_CELL);
wire [4:0] ring_row = screen_line + head_row + 5'd1;        /* the oldest line is at the top, the line being typed at the bottom */

reg        fetching_q  = 1'b0;
reg [7:0]  next_glyph_row = 8'd0, current_glyph_row = 8'd0;
reg        next_glyph_valid = 1'b0, current_glyph_valid = 1'b0;
assign font_address = {text_read_data, line_in_pitch[2:0]};

always @(posedge clk) begin
    if (pixel_in_cell == 3'd0) begin                        /* ask the text RAM for the character of the next cell */
        text_read_address <= {ring_row, next_cell_column};
        fetching_q        <= fetching_glyph_row;
    end
    if (pixel_in_cell == 3'd3) begin                        /* the character has arrived (phase 2), the font's glyph row arrives now */
        next_glyph_row   <= fetching_q ? font_row : 8'd0;
        next_glyph_valid <= fetching_q;
    end
    if (pixel_in_cell == 3'd7) begin                        /* the next cell becomes the current one */
        current_glyph_row   <= next_glyph_row;
        current_glyph_valid <= next_glyph_valid;
    end
end

/*
    ---- green bars and ink
    A bar covers three text lines, then three lines are white; the pattern repeats every six lines and moves up with each line feed.
*/
wire [3:0] bar_position_sum = {1'b0, screen_line % 6} + {1'b0, line_feed_phase};
wire [2:0] bar_position     = (bar_position_sum >= 4'd6) ? bar_position_sum[2:0] - 3'd6 : bar_position_sum[2:0];
wire       in_green_bar     = in_text_area && (bar_position >= 3'd3);

wire in_sheet = (pixel_x >= SHEET_LEFT_X) && (pixel_x < SHEET_RIGHT_X);
wire ink      = in_sheet && current_glyph_valid && current_glyph_row[3'd7 - pixel_in_cell];     /* the glyph's left pixel is bit 7 */

/*
    ---- sprocket holes
    Holes that look round on the screen, one every 32 lines, centred in each strip. A pixel of the picture is 1.4 times wider and 2.07 times taller
    than it is wide on the canvas, so a round hole is 14 pixels across and 10 lines high. The hole is drawn row by row: its half width depends on
    the row's distance from the centre line of the hole (which lies between two picture lines), a quarter of that ellipse in a small table.
*/
wire [4:0] hole_row_distance = picture_line[4] ? (picture_line[4:0] - 5'd16) : (5'd15 - picture_line[4:0]);     /* 0 at the centre */

reg [2:0] hole_half_width;                              /* in pixels */
always @* begin
    case (hole_row_distance)
        5'd0, 5'd1:       hole_half_width = 3'd7;
        5'd2:             hole_half_width = 3'd6;
        5'd3:             hole_half_width = 3'd5;
        5'd4:             hole_half_width = 3'd2;
        default:          hole_half_width = 3'd0;       /* outside the hole */
    endcase
end

/* distance of the pixel from the hole's centre line, which lies between two pixels (x = 19/20 in the left strip, 699/700 in the right one) */
wire [9:0] left_hole_distance  = (pixel_x >= 10'd20)  ? pixel_x - 10'd20  : 10'd19  - pixel_x;
wire [9:0] right_hole_distance = (pixel_x >= 10'd700) ? pixel_x - 10'd700 : 10'd699 - pixel_x;

wire in_left_hole  = (pixel_x < SHEET_LEFT_X) && (left_hole_distance < {7'd0, hole_half_width});
wire in_right_hole = (pixel_x >= SHEET_RIGHT_X) && (pixel_x < 10'd720) && (right_hole_distance < {7'd0, hole_half_width});

/*
    ---- colour of the pixel
*/
reg [23:0] pixel_color = COLOR_BLANKING;

always @(posedge clk) begin
    if (!de)
        pixel_color <= COLOR_BLANKING;
    else if (in_sheet) begin
        if (ink)
            pixel_color <= COLOR_INK;
        else if (in_green_bar)
            pixel_color <= COLOR_GREEN_BAR;
        else
            pixel_color <= COLOR_PAPER;
    end
    else if (in_left_hole | in_right_hole)
        pixel_color <= COLOR_HOLE;
    else if (pixel_x == SHEET_LEFT_X - 1 || pixel_x == SHEET_RIGHT_X)
        pixel_color <= COLOR_PERFORATION;
    else
        pixel_color <= COLOR_PAPER;

    rgb <= pixel_color;                                 /* one more register stage, to line up with the glyph pipeline */
end

endmodule
