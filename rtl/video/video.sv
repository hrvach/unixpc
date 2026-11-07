/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The video output: the machine's picture on a 1280x720 canvas made from the 75 MHz video clock, locked to the machine's own timing. The picture
    (1008x720, the shape of the tube) is in the middle, on top of the monitor's frame: a 4 bit grey picture of the top left quarter (161x360 pixels,
    in a ROM made by tools/gen_bezel.py) mirrored to the other three quarters. The frame shows on both sides of the picture and wherever the picture
    is not (its curved edge, its corners). The picture shows either the screen of the tube or, while the printer is shown, the printer's paper.

    The brightness effects (phosphor persistence, glow, noise and flicker) run at the machine's own resolution in crt_fx and deliver
    an 8 bit brightness stream of the foreground, on a true black background. That stream is written line by line into a ring of 64 lines. The canvas
    reads the ring, and everything that is geometry is done here, at 720p, where it does not turn into stair steps:

      - barrel distortion of the foreground: every canvas pixel is traced back to the position in the source picture it shows, with a radial bow term
      - line wobble: one source line, picked at random every third frame, jumps sideways by a few sixteenths of a pixel
      - bilinear sampling horizontally, nearest line vertically

    The glass is not part of that: the dark level of the tube face is a flat layer of its own, added after the foreground, so it is not warped. It
    fades a little towards the edges and the canvas corners are rounded by a few pixels. The foreground is darkened where the bow pulls the picture
    away from the edge of the canvas.

    Every stage of the pipeline does one step of arithmetic (a multiplication, an addition and a comparison, a table lookup), so that every path
    between two registers is short; the latency of about 21 canvas clocks does not matter for a video output.

    Locked timing. Both clocks come from one PLL (600 MHz divided by 30 and by 8), so the ratio is exact. A machine line is 896 clocks of 20 MHz =
    44.8 us, a canvas line is 1680 clocks of 75 MHz = 22.4 us: two canvas lines per machine line. A machine frame is 372 lines, a canvas frame is 744
    lines, both 16.6656 ms. The canvas counters are restarted from the machine's vertical sync only when they have drifted (at power up and after a
    reset), so there is no tearing and no frame buffer.

    The canvas frame starts 48 canvas rows (24 machine lines) after the machine's vertical sync. The canvas shows 348 source lines in 720 rows (2.07
    rows per line, 1008 columns for 720 pixels: the shape of the real tube, 1.4). With the barrel a canvas row shows lines up to 10.4 lines away from
    the nominal one (strength 0.125 is the most), so the canvas must run at least 35 canvas rows behind the machine; because it shows lines a little
    slower than the machine makes them, the ring has to hold up to 40 lines: 64 are enough.

    Ring: two banks of 8 bit pixels, one for the even source columns and one for the odd ones, so that the two neighbours of a bilinear sample are
    read in the same clock. A line is addressed by its number in the frame modulo 64, a pixel by its column divided by two.
*/
/* the bezel files are read by bare name; the Quartus project sets the macros to roms/ in 3B1.qsf */
`ifndef BEZEL_MEM_FILE
`define BEZEL_MEM_FILE "bezel.mem"
`endif
module video
import paper_palette_pkg::*;
#(
    parameter BEZEL_FILE = `BEZEL_MEM_FILE
)
(
    /* the machine's picture after the brightness effects (machine clock) */
    input         clk_machine,
    input  [7:0]  brightness,
    input         picture_de,
    input         picture_vs,

    /* the printer's paper (machine clock): the colour of the pixel two clocks after picture_de, from lp_printer */
    input  [23:0] paper_pixel,
    input         show_paper,               /* the paper instead of the screen (changes rarely) */
    input         bezel_on,                 /* the monitor's frame on both sides of the picture (changes rarely) */

    /* settings */
    input  [2:0]  curvature_strength,       /* 0 .. 6: k = 2/64 1/64 3/64 4/64 5/64 6/64 8/64; 7 = off */
    input  [1:0]  wobble,                   /* 0 off, 1 low, 2 mid, 3 high */
    input  [3:0]  leds_lit,                 /* the four front panel LEDs, left to right, 1 = lit (clk_machine domain, changes rarely) */
    input         edges_on,                 /* the foreground fades towards the edges of the screen */
    input         round_corners,            /* the corners of the screen are rounded by 28 pixels (otherwise by 4) */
    input  [1:0]  roll_band,                /* the rolling bright band: 0 off, 1 low (3 levels), 2 mid (7), 3 high (15) */
    input  [1:0]  screen_colour,            /* 0 white, 1 green, 2 amber, 3 mint */
    input  [7:0]  glass_floor,              /* brightness code of the dark level of the glass (0 = true black) */

    /* the canvas */
    input         clk_pixel,                /* 75 MHz */
    output reg    de = 1'b0,
    output reg    hs = 1'b0,
    output reg    vs = 1'b0,
    output reg [23:0] rgb = 24'd0
);

/* ---- the ring of lines: written at the machine clock, read at the pixel clock */
localparam SOURCE_HEIGHT = 348;

(* ramstyle = "M10K" *) reg [7:0] bank_even [0:32767];
(* ramstyle = "M10K" *) reg [7:0] bank_odd  [0:32767];

reg        picture_de_previous = 1'b0;
reg        picture_vs_previous = 1'b0;
reg [9:0]  write_x = 10'd0;
reg [5:0]  write_line = 6'd0;

always @(posedge clk_machine) begin
    picture_de_previous <= picture_de;
    picture_vs_previous <= picture_vs;

    if (picture_vs && !picture_vs_previous) begin
        write_x    <= 10'd0;
        write_line <= 6'd0;
    end
    else begin
        write_x <= picture_de ? write_x + 10'd1 : 10'd0;

        if (picture_de_previous && !picture_de)
            write_line <= write_line + 6'd1;
    end

    if (picture_de) begin
        if (!write_x[0])
            bank_even[{write_line, write_x[9:1]}] <= brightness;
        else
            bank_odd[{write_line, write_x[9:1]}] <= brightness;
    end
end

reg [14:0] read_address_even = 15'd0;
reg [14:0] read_address_odd = 15'd0;
reg [7:0]  value_even = 8'd0;
reg [7:0]  value_odd = 8'd0;

always @(posedge clk_pixel) begin
    value_even <= bank_even[read_address_even];
    value_odd  <= bank_odd[read_address_odd];
end

/*
    The paper has a ring of its own, in the same lines and columns as the screen's: the number of the colour of each pixel (three bits, see
    paper_palette_pkg). The paper pixel arrives two clocks after the timing signals, so the address and the write enable of the screen's write are
    delayed by two clocks. The paper is read without interpolation: it is flat and has sharp edges.
*/
(* ramstyle = "M10K" *) reg [2:0] paper_ring [0:65535];

reg [15:0] paper_write_address_1 = 16'd0;
reg [15:0] paper_write_address_2 = 16'd0;
reg        paper_write_1 = 1'b0;
reg        paper_write_2 = 1'b0;

always @(posedge clk_machine) begin
    paper_write_address_1 <= {write_line, write_x};
    paper_write_address_2 <= paper_write_address_1;
    paper_write_1         <= picture_de;
    paper_write_2         <= paper_write_1;

    if (paper_write_2)
        paper_ring[paper_write_address_2] <= paper_colour_number(paper_pixel);
end

reg [15:0] paper_read_address = 16'd0;
reg [2:0]  paper_read_number = 3'd0;

always @(posedge clk_pixel)
    paper_read_number <= paper_ring[paper_read_address];

/* the paper shown, brought into the pixel domain; the paper is drawn flat and steady: no barrel distortion, no wobble */
reg paper_shown_1 = 1'b0;
reg paper_shown = 1'b0;

always @(posedge clk_pixel) begin
    paper_shown_1 <= show_paper;
    paper_shown   <= paper_shown_1;
end

/* ---- canvas timing (1280x720 active, 1680 x 744 total: 75 MHz / 1249920 = 60.0 Hz, as the machine) */
localparam [10:0] H_TOTAL = 11'd1680;
localparam [10:0] H_ACTIVE = 11'd1280;
localparam [10:0] HS_START = 11'd1320;
localparam [10:0] HS_END = 11'd1360;
localparam [10:0] PICTURE_WIDTH = 11'd1008;
localparam [10:0] BEZEL_WIDTH = (H_ACTIVE - PICTURE_WIDTH) / 2;            /* the frame on each side of the picture: 136 columns */
localparam [9:0]  V_TOTAL = 10'd744;
localparam [9:0]  V_ACTIVE = 10'd720;
localparam [9:0]  VS_START = 10'd725;
localparam [9:0]  VS_END = 10'd730;
localparam [9:0]  CANVAS_DELAY_ROWS = 10'd48;                  /* the canvas frame starts this many rows after the machine's vertical sync */
localparam [9:0]  RESTART_ROW = V_TOTAL - CANVAS_DELAY_ROWS;

reg [10:0] column = 11'd0;
reg [9:0]  row = 10'd0;

/* the machine's vertical sync, brought into the pixel domain (the clocks are phase locked, so the delay is the same every frame) */
reg vsync_synchroniser_1 = 1'b0;
reg vsync_synchroniser_2 = 1'b0;
reg vsync_synchroniser_3 = 1'b0;

wire vsync_event = vsync_synchroniser_2 & ~vsync_synchroniser_3;

/* the counters were restarted at the previous event, so they are back at the restart position now (give or take a clock) when they are locked */
wire at_restart_position = ((row == RESTART_ROW) && (column <= 11'd4)) || ((row == RESTART_ROW - 10'd1) && (column >= H_TOTAL - 11'd4));

always @(posedge clk_pixel) begin
    vsync_synchroniser_1 <= picture_vs;
    vsync_synchroniser_2 <= vsync_synchroniser_1;
    vsync_synchroniser_3 <= vsync_synchroniser_2;

    if (vsync_event && !at_restart_position) begin
        column <= 11'd0;
        row    <= RESTART_ROW;
    end
    else if (column == H_TOTAL - 11'd1) begin
        column <= 11'd0;
        row    <= (row == V_TOTAL - 10'd1) ? 10'd0 : row + 10'd1;
    end
    else
        column <= column + 11'd1;
end

/* ---- per frame state: the wobbling line */
reg [15:0] frame_random = 16'hBEEF;
reg [1:0]  frame_divider = 2'd0;
reg [8:0]  wobble_line = 9'd0;
reg signed [27:0] wobble_offset = 28'sd0;        /* in 1/65536 of a source pixel */

wire [1:0] wobble_in_use = paper_shown ? 2'd0 : wobble;
wire [3:0] wobble_amount = (wobble_in_use == 2'd1) ? 4'd3 : (wobble_in_use == 2'd2) ? 4'd6 : 4'd10;     /* sixteenths of a pixel */
wire signed [27:0] wobble_size = $signed({24'd0, wobble_amount}) <<< 12;                   /* amount * 4096 */

always @(posedge clk_pixel) begin
    if (column == 11'd0 && row == 10'd0) begin
        frame_random  <= {frame_random[14:0], frame_random[15] ^ frame_random[13] ^ frame_random[12] ^ frame_random[10]};
        frame_divider <= (frame_divider == 2'd2) ? 2'd0 : frame_divider + 2'd1;

        if (frame_divider == 2'd0) begin
            wobble_line <= (frame_random[8:0] < SOURCE_HEIGHT) ? frame_random[8:0] : frame_random[8:0] - SOURCE_HEIGHT;

            if (wobble_in_use == 2'd0)
                wobble_offset <= 28'sd0;
            else if (frame_random[9])
                wobble_offset <= wobble_size;
            else
                wobble_offset <= -wobble_size;
        end
    end
end

/*
    ---- the rolling band: a bright band that drifts down the picture, one source line per frame, drawn here in canvas rows so that it is straight and
    always the same size (the barrel does not touch it). It is a triangle 32 source lines (66 rows) wide, rising over the first 33 rows and falling
    over the next 33, added to the brightness of the glass and of the foreground alike. The position is kept in 1/256 row; the band starts 48 source
    lines above the top and is restarted when it has left the bottom.
*/
localparam signed [23:0] BAND_STEP = 24'sd530;                  /* 2.069 rows per frame, in 1/256 row */
localparam signed [23:0] BAND_RESTART = -24'sd25426;            /* -48 source lines */
localparam signed [23:0] BAND_WRAP = 24'sd183800;               /* 347 source lines */

reg signed [23:0] band_start_q8 = BAND_RESTART;

always @(posedge clk_pixel) begin
    if (column == 11'd0 && row == 10'd0) begin
        if (band_start_q8 >= BAND_WRAP)
            band_start_q8 <= BAND_RESTART;
        else
            band_start_q8 <= band_start_q8 + BAND_STEP;
    end
end

wire signed [15:0] band_start_row = band_start_q8[23:8];

/* ---- the barrel: source position = centre + normalised canvas position * (a + b * r^2) * half size of the source */
/*
    the barrel strength k = n / 64 from the OSD code: a = 65536 / (1 + k), b = a * k; the middle of each edge of the canvas shows the edge of the
    picture
*/
reg signed [17:0] barrel_a;
reg signed [17:0] barrel_b;

always @* begin
    case (paper_shown ? 3'd7 : curvature_strength)
        3'd0:    begin barrel_a = 18'sd63550; barrel_b = 18'sd1986; end     /* 2/64 = 0.0312 */
        3'd1:    begin barrel_a = 18'sd64528; barrel_b = 18'sd1008; end     /* 1/64 = 0.0156 */
        3'd2:    begin barrel_a = 18'sd62602; barrel_b = 18'sd2934; end     /* 3/64 = 0.0469 */
        3'd3:    begin barrel_a = 18'sd61681; barrel_b = 18'sd3855; end     /* 4/64 = 0.0625 */
        3'd4:    begin barrel_a = 18'sd60787; barrel_b = 18'sd4749; end     /* 5/64 = 0.0781 */
        3'd5:    begin barrel_a = 18'sd59919; barrel_b = 18'sd5617; end     /* 6/64 = 0.0938 */
        3'd6:    begin barrel_a = 18'sd58254; barrel_b = 18'sd7282; end     /* 8/64 = 0.1250 */
        default: begin barrel_a = 18'sd65536; barrel_b = 18'sd0; end             /* off */
    endcase
end

/* stage 1: normalised position (Q16: +-1.0 is the edge of the canvas) */
reg signed [19:0] nx_1 = 20'sd0;
reg signed [19:0] ny_1 = 20'sd0;
reg               in_window_1 = 1'b0;

wire signed [11:0] canvas_dx = $signed({1'b0, column}) - 12'sd640;       /* the middle of the canvas */
wire signed [11:0] canvas_dy = $signed({2'b00, row}) - 12'sd360;

wire signed [19:0] canvas_dx_wide = canvas_dx;
wire signed [19:0] canvas_dy_wide = canvas_dy;

always @(posedge clk_pixel) begin
    nx_1        <= (canvas_dx_wide <<< 7) + (canvas_dx_wide <<< 1);                                                      /* x * 130 = 65536 / 504 */
    ny_1        <= (canvas_dy_wide <<< 8) - (canvas_dy_wide <<< 6) - (canvas_dy_wide <<< 3) - (canvas_dy_wide <<< 1);    /* y * 182 = 65536 / 360 */
    in_window_1 <= (column >= BEZEL_WIDTH) && (column < BEZEL_WIDTH + PICTURE_WIDTH) && (row < V_ACTIVE);
end

/* stage 2: the squares */
reg signed [19:0] nx_2 = 20'sd0;
reg signed [19:0] ny_2 = 20'sd0;
reg [17:0]        nx_squared_2 = 18'd0;
reg [17:0]        ny_squared_2 = 18'd0;
reg               in_window_2 = 1'b0;

wire signed [39:0] nx_square_full = nx_1 * nx_1;
wire signed [39:0] ny_square_full = ny_1 * ny_1;

always @(posedge clk_pixel) begin
    nx_2         <= nx_1;
    ny_2         <= ny_1;
    nx_squared_2 <= nx_square_full[33:16];
    ny_squared_2 <= ny_square_full[33:16];
    in_window_2  <= in_window_1;
end

/* stage 3: r squared */
reg signed [19:0] nx_3 = 20'sd0;
reg signed [19:0] ny_3 = 20'sd0;
reg [18:0]        radius_squared_3 = 19'd0;
reg               in_window_3 = 1'b0;

always @(posedge clk_pixel) begin
    nx_3             <= nx_2;
    ny_3             <= ny_2;
    radius_squared_3 <= {1'b0, nx_squared_2} + {1'b0, ny_squared_2};
    in_window_3      <= in_window_2;
end

/* stage 4: b * r^2 */
reg signed [19:0] nx_4 = 20'sd0;
reg signed [19:0] ny_4 = 20'sd0;
reg signed [39:0] barrel_product_4 = 40'sd0;
reg               in_window_4 = 1'b0;

always @(posedge clk_pixel) begin
    nx_4             <= nx_3;
    ny_4             <= ny_3;
    barrel_product_4 <= barrel_b * $signed({1'b0, radius_squared_3});
    in_window_4      <= in_window_3;
end

/* stage 5: the scale factor a + b * r^2 */
reg signed [19:0] nx_5 = 20'sd0;
reg signed [19:0] ny_5 = 20'sd0;
reg signed [19:0] scale_5 = 20'sd0;
reg               in_window_5 = 1'b0;

always @(posedge clk_pixel) begin
    nx_5        <= nx_4;
    ny_5        <= ny_4;
    scale_5     <= barrel_a + barrel_product_4[35:16];
    in_window_5 <= in_window_4;
end

/* stage 6: the bowed normalised position */
reg signed [19:0] bowed_x_6 = 20'sd0;
reg signed [19:0] bowed_y_6 = 20'sd0;
reg               in_window_6 = 1'b0;

wire signed [39:0] bowed_x_full = nx_5 * scale_5;
wire signed [39:0] bowed_y_full = ny_5 * scale_5;

always @(posedge clk_pixel) begin
    bowed_x_6   <= bowed_x_full[35:16];
    bowed_y_6   <= bowed_y_full[35:16];
    in_window_6 <= in_window_5;
end

/* stage 7: the position in the source picture, in 1/65536 of a pixel (u: of the texel centres, v: of the lines) */
reg signed [27:0] source_u_7 = 28'sd0;
reg signed [27:0] source_v_7 = 28'sd0;
reg               in_window_7 = 1'b0;

wire signed [27:0] bowed_x_wide = bowed_x_6;
wire signed [27:0] bowed_y_wide = bowed_y_6;

wire signed [27:0] bowed_x_scaled = (bowed_x_wide <<< 8) + (bowed_x_wide <<< 7) - (bowed_x_wide <<< 4) - (bowed_x_wide <<< 3);     /* x * 360 */
wire signed [27:0] bowed_y_scaled = (bowed_y_wide <<< 7) + (bowed_y_wide <<< 6) - (bowed_y_wide <<< 4) - (bowed_y_wide <<< 1);     /* y * 174 */

/*
    The margin of the pixel inside the picture, in 1/65536 of the half width (or half height, whichever is nearer): positive inside the curved
    picture, zero on its edge, negative outside. It darkens the foreground towards its edge and blanks it beyond. The absolute values are formed in
    stage 7 and the margin from them in stage 8.
*/
wire signed [20:0] bowed_x_magnitude = (bowed_x_6 < 0) ? -bowed_x_6 : bowed_x_6;
wire signed [20:0] bowed_y_magnitude = (bowed_y_6 < 0) ? -bowed_y_6 : bowed_y_6;
reg signed [20:0] bow_x_abs_7 = 21'sd0;
reg signed [20:0] bow_y_abs_7 = 21'sd0;

always @(posedge clk_pixel) begin
    bow_x_abs_7 <= bowed_x_magnitude;
    bow_y_abs_7 <= bowed_y_magnitude;
    source_u_7  <= 28'sd23560192 + bowed_x_scaled;          /* 360.0 - 0.5 pixels: half the width of the source, in texel centres */
    source_v_7  <= 28'sd11403264 + bowed_y_scaled;          /* 174.0 lines: half the height of the source */
    in_window_7 <= in_window_6;
end

/* stage 8: the wobble, the texel numbers, the fractions, and whether the position is inside the picture */
wire [8:0] line_8 = source_v_7[24:16];
wire signed [27:0] u_wobbled = (line_8 == wobble_line) ? source_u_7 + wobble_offset : source_u_7;
wire        inside_8 = in_window_7 && (source_v_7 >= 28'sd0) && (source_v_7 < 28'sd22806528) && (u_wobbled >= -28'sd32768) &&
                       (u_wobbled < 28'sd47153152);

reg [9:0]  texel_0_8 = 10'd0;
reg [9:0]  texel_1_8 = 10'd0;
reg [7:0]  across_8 = 8'd0;
reg [8:0]  line_number_8 = 9'd0;
reg        valid_8 = 1'b0;

always @(posedge clk_pixel) begin
    valid_8       <= inside_8;
    line_number_8 <= line_8;

    if (u_wobbled < 28'sd0) begin
        texel_0_8 <= 10'd0;
        texel_1_8 <= 10'd0;
        across_8  <= 8'd0;
    end
    else begin
        texel_0_8 <= u_wobbled[25:16];
        texel_1_8 <= (u_wobbled[25:16] == 10'd719) ? 10'd719 : u_wobbled[25:16] + 10'd1;
        across_8  <= u_wobbled[15:8];
    end
end

/* stage 9: the ring slot of the line (its number modulo 32) */
reg [14:0] row_offset_9 = 15'd0;
reg [9:0]  texel_0_9 = 10'd0;
reg [9:0]  texel_1_9 = 10'd0;
reg [7:0]  across_9 = 8'd0;
reg        valid_9 = 1'b0;

always @(posedge clk_pixel) begin
    row_offset_9  <= {line_number_8[5:0], 9'd0};
    texel_0_9     <= texel_0_8;
    texel_1_9     <= texel_1_8;
    across_9      <= across_8;
    valid_9       <= valid_8;
end

/* stage 10: the two addresses (the even texel and the odd texel of the pair are in different banks) */
reg        texel_0_odd_10 = 1'b0;
reg [7:0]  across_10 = 8'd0;
reg        valid_10 = 1'b0;

always @(posedge clk_pixel) begin
    if (!texel_0_9[0]) begin
        read_address_even <= {row_offset_9[14:9], texel_0_9[9:1]};
        read_address_odd  <= {row_offset_9[14:9], texel_1_9[9:1]};
    end
    else begin
        read_address_odd  <= {row_offset_9[14:9], texel_0_9[9:1]};
        read_address_even <= {row_offset_9[14:9], texel_1_9[9:1]};
    end

    paper_read_address <= {row_offset_9[14:9], texel_0_9};

    texel_0_odd_10 <= texel_0_9[0];
    across_10      <= across_9;
    valid_10       <= valid_9;
end

/* stage 11: the pixels arrive */
reg        texel_0_odd_11 = 1'b0;
reg [7:0]  across_11 = 8'd0;
reg        valid_11 = 1'b0;

always @(posedge clk_pixel) begin
    texel_0_odd_11 <= texel_0_odd_10;
    across_11      <= across_10;
    valid_11       <= valid_10;
end

/* stage 12: the left and the right texel */
reg [7:0]  left_12 = 8'd0;
reg [7:0]  right_12 = 8'd0;
reg [7:0]  across_12 = 8'd0;
reg        valid_12 = 1'b0;

always @(posedge clk_pixel) begin
    left_12        <= texel_0_odd_11 ? value_odd : value_even;
    right_12       <= texel_0_odd_11 ? value_even : value_odd;
    across_12      <= across_11;
    valid_12       <= valid_11;
end

/* stage 13: the horizontal blend */
reg [7:0]  blended_13 = 8'd0;
reg        valid_13 = 1'b0;

wire signed [9:0]  texel_difference = $signed({2'b00, right_12}) - $signed({2'b00, left_12});
wire signed [18:0] blend_product = texel_difference * $signed({1'b0, across_12});

always @(posedge clk_pixel) begin
    blended_13     <= left_12 + (blend_product >>> 8);
    valid_13       <= valid_12;
end

/* stage 14: the blended brightness travels on (the fall off and the edge below are formed in parallel) */
reg [7:0]  blended_14 = 8'd0;
reg        valid_14 = 1'b0;

always @(posedge clk_pixel) begin
    blended_14 <= blended_13;
    valid_14   <= valid_13;
end

/*
    stage 14: the fall off of the foreground towards the edges of the screen (the squared radius times 40: 84 % at the middle of an edge, 69 % in a
    corner); stage 15: it is combined with the anti-aliasing of the edge of the bowed picture into one gain
*/
reg [7:0] edge_gain_14 = 8'd255;
reg [7:0] fade_gain_15 = 8'd0;

wire [24:0] radius_squared_14 = radius_squared_pipe[13];
wire [24:0] edge_darkening = (radius_squared_14 << 5) + (radius_squared_14 << 3);      /* r^2 * 40 */

always @(posedge clk_pixel) begin
    edge_gain_14 <= edges_on ? 8'd255 - edge_darkening[23:16] : 8'd255;
end

/* stage 15: the brightness of the foreground, waiting for the gain of the edge */
reg [7:0]  foreground_15 = 8'd0;
reg        valid_15 = 1'b0;

/*
    The corners of the bowed picture, rounded by 28 canvas pixels with the Round corners setting. In the normalised position of the picture (1.0 is
    the edge) the straight part of an edge ends 28 pixels before the corner: 28/504 of the half width across and 28/360 of the half height down. The
    distance past those points is turned into canvas pixels (stage 15, 16), squared (stage 17) and the corner circle is tested (stage 18).
*/
localparam signed [20:0] CORNER_START_X = 21'sd61895;       /* 65536 - 65536 * 28 / 504 */
localparam signed [20:0] CORNER_START_Y = 21'sd60438;       /* 65536 - 65536 * 28 / 360 */

/* the distance past the start of the corner, in 1/65536 of the half width / height (a 28 pixel corner is 3641 / 5097) */
reg [12:0] corner_x_15 = 13'd0;
reg [12:0] corner_y_15 = 13'd0;
reg [5:0]  corner_x_pixels_16 = 6'd0;
reg [5:0]  corner_y_pixels_16 = 6'd0;
reg [11:0] corner_x_squared_17 = 12'd0;
reg [11:0] corner_y_squared_17 = 12'd0;
reg        round_cut_18 = 1'b0;

wire signed [20:0] corner_dx = bow_x_abs_pipe[14] - CORNER_START_X;
wire signed [20:0] corner_dy = bow_y_abs_pipe[14] - CORNER_START_Y;
wire [21:0] corner_x_wide = corner_x_15;
wire [21:0] corner_y_wide = corner_y_15;
wire [21:0] corner_x_scaled = (corner_x_wide << 9) - (corner_x_wide << 3);                                         /* x * 504 */
wire [21:0] corner_y_scaled = (corner_y_wide << 8) + (corner_y_wide << 7) - (corner_y_wide << 4) - (corner_y_wide << 3);     /* y * 360 */

always @(posedge clk_pixel) begin
    corner_x_15 <= (corner_dx > 21'sd0) ? ((corner_dx > 21'sd8191) ? 13'd8191 : corner_dx[12:0]) : 13'd0;
    corner_y_15 <= (corner_dy > 21'sd0) ? ((corner_dy > 21'sd8191) ? 13'd8191 : corner_dy[12:0]) : 13'd0;
    corner_x_pixels_16 <= corner_x_scaled[21:16];
    corner_y_pixels_16 <= corner_y_scaled[21:16];
    corner_x_squared_17 <= corner_x_pixels_16 * corner_x_pixels_16;
    corner_y_squared_17 <= corner_y_pixels_16 * corner_y_pixels_16;
    round_cut_18 <= round_corners && (({1'b0, corner_x_squared_17} + {1'b0, corner_y_squared_17}) > 13'd784);
end

localparam signed [20:0] AA_WIDTH = 21'sd204;               /* 0.003 of the half width: about a pixel and a half */

wire signed [20:0] margin_14 = margin_pipe[14];
/* 0 at the edge, 255 a pixel and a half inside (margin * 5 / 4) */
wire [7:0]  aa_gain_14 = (margin_14 >= AA_WIDTH) ? 8'd255 : ((margin_14[11:0] << 2) + margin_14[11:0]) >> 2;
wire [15:0] fade_gain_product = edge_gain_14 * aa_gain_14;

always @(posedge clk_pixel) begin
    foreground_15 <= blended_14;
    fade_gain_15 <= fade_gain_product[15:8];
    valid_15     <= valid_14;
end

/* the colour number of the paper pixel travels along with the pixel, from the stage where it arrives (11) to the stage of the overlay (19) */
reg [2:0] paper_number_pipe [12:19];
integer   paper_stage;

always @(posedge clk_pixel) begin
    paper_number_pipe[12] <= paper_read_number;

    for (paper_stage = 13; paper_stage <= 19; paper_stage = paper_stage + 1)
        paper_number_pipe[paper_stage] <= paper_number_pipe[paper_stage - 1];
end

/* the margin (from stage 8) and the canvas position (from stage 1) travel along with the pixel */
localparam PIPELINE_STAGES = 21;

reg signed [20:0] margin_pipe [8:16];
reg signed [20:0] bow_x_abs_pipe [8:14];
reg signed [20:0] bow_y_abs_pipe [8:14];
reg [18:0]        radius_squared_pipe [3:16];
reg [10:0]        column_pipe [1:PIPELINE_STAGES];
reg [9:0]         row_pipe [1:PIPELINE_STAGES];
integer           carry;

always @(posedge clk_pixel) begin
    margin_pipe[8]  <= 21'sd65536 - ((bow_x_abs_7 > bow_y_abs_7) ? bow_x_abs_7 : bow_y_abs_7);
    bow_x_abs_pipe[8] <= bow_x_abs_7;
    bow_y_abs_pipe[8] <= bow_y_abs_7;
    radius_squared_pipe[3] <= radius_squared_3;
    column_pipe[1]  <= column;
    row_pipe[1]     <= row;

    for (carry = 9; carry <= 16; carry = carry + 1)
        margin_pipe[carry] <= margin_pipe[carry - 1];

    for (carry = 9; carry <= 14; carry = carry + 1) begin
        bow_x_abs_pipe[carry] <= bow_x_abs_pipe[carry - 1];
        bow_y_abs_pipe[carry] <= bow_y_abs_pipe[carry - 1];
    end

    for (carry = 4; carry <= 16; carry = carry + 1)
        radius_squared_pipe[carry] <= radius_squared_pipe[carry - 1];

    for (carry = 2; carry <= PIPELINE_STAGES; carry = carry + 1) begin
        column_pipe[carry] <= column_pipe[carry - 1];
        row_pipe[carry]    <= row_pipe[carry - 1];
    end
end

/*
    stage 16: the edge of the bowed picture is anti-aliased over about a pixel and a half (no stair steps, and no band that would show as a line);
    outside it everything is black
*/

reg signed [12:0] band_distance_16 = 13'sd0;
reg [7:0]  faded_16 = 8'd0;
reg        inside_16 = 1'b0;

wire signed [20:0] fade_margin = margin_pipe[15];
/* 0 at the edge, 255 at the end of the band */
/* 0 at the edge, 255 a pixel and a half inside */
wire [15:0] faded_full = foreground_15 * fade_gain_15;

/*
    the glass darkening towards the corners, kept in 27 bits (the squared radius times the fixed gradient 100) and applied with full precision in
    stage 17: no steps, no rings
*/
reg [26:0] darkening_16 = 27'd0;
wire [26:0] radius_squared_15 = radius_squared_pipe[15];

always @(posedge clk_pixel) begin
    band_distance_16 <= $signed({3'd0, row_pipe[15]}) - band_start_row;
    inside_16    <= valid_15 && (fade_margin >= 21'sd0);
    /* the glass is lightest in the middle: 100 % there, 76 % at the middle of an edge, 53 % in a corner */
    darkening_16  <= (radius_squared_15 << 6) + (radius_squared_15 << 5) + (radius_squared_15 << 2);       /* r^2 * 100 */
    faded_16     <= (valid_15 && fade_margin >= 21'sd0) ? faded_full[15:8] : 8'd0;
end

/*
    stage 17: the glass (flat inside the bowed picture) and the foreground raised by it: glass + foreground * (255 - glass) / 256, in 8.8 fixed point
*/

wire [34:0] glass_loss = glass_floor * darkening_16;
wire [34:0] glass_level = {19'd0, glass_floor, 8'd0} - {8'd0, glass_floor} - (glass_loss >> 16);     /* floor * (255 - darkening), 8.8 fixed point */
wire [15:0] glass_full = (inside_16 && !glass_level[34]) ? glass_level[15:0] : 16'd0;
wire [15:0] foreground_full = faded_16 * (8'd255 - glass_floor);

/*
    The corners of the picture are rounded by 4 pixels (by 28 for the paper, with the Round corners setting). The test is a circle: the distance of
    the pixel from the straight part of the edge, across and down, squared and added, against the squared radius. It takes three stages: the
    distances (17), the squares and their sum (18), the comparison (19). The column is counted from the left edge of the picture.
*/
localparam [10:0] CORNER_RADIUS = 11'd4;
localparam [10:0] PAPER_CORNER_RADIUS = 11'd28;      /* the paper is flat: its corners are circles in canvas pixels */

wire [10:0] picture_column_16 = column_pipe[16] - BEZEL_WIDTH;
wire [10:0] paper_radius = round_corners ? PAPER_CORNER_RADIUS : CORNER_RADIUS;

function [10:0] corner_distance_across(input [10:0] x, input [10:0] radius);
    corner_distance_across = (x < radius) ? radius - x : ((x > PICTURE_WIDTH - 11'd1 - radius) ? x - (PICTURE_WIDTH - 11'd1 - radius) : 11'd0);
endfunction

function [10:0] corner_distance_down(input [10:0] y, input [10:0] radius);
    corner_distance_down = (y < radius) ? radius - y : ((y > V_ACTIVE - 11'd1 - radius) ? y - (V_ACTIVE - 11'd1 - radius) : 11'd0);
endfunction

/* the band: the triangle of the distance into it (0 .. 65 rows), scaled by the setting */
wire [6:0] band_ramp = (band_distance_16 < 13'sd33) ? band_distance_16[6:0] : 7'd66 - band_distance_16[6:0];
wire [12:0] band_ramp_wide = band_ramp;
wire [12:0] band_ramp_scaled = (band_ramp_wide << 6) - (band_ramp_wide << 1);      /* ramp * 62 */
wire [7:0] band_shape = (band_distance_16 >= 13'sd0 && band_distance_16 < 13'sd66) ? band_ramp_scaled[10:3] : 8'd0;

reg [7:0] band_add_17 = 8'd0;

always @(posedge clk_pixel) begin
    if (!inside_16)
        band_add_17 <= 8'd0;
    else begin
        case (roll_band)
            2'd1:    band_add_17 <= {6'd0, band_shape[7:6]};
            2'd2:    band_add_17 <= {5'd0, band_shape[7:5]};
            2'd3:    band_add_17 <= {4'd0, band_shape[7:4]};
            default: band_add_17 <= 8'd0;
        endcase
    end
end

/* stage 17: the glass and the foreground (8.8 fixed point), and the distances for the corner cuts */
reg [15:0] glass_17 = 16'd0;
reg [15:0] foreground_17 = 16'd0;
reg [10:0] cut_across_17 = 11'd0;
reg [10:0] cut_down_17 = 11'd0;
reg [10:0] paper_across_17 = 11'd0;
reg [10:0] paper_down_17 = 11'd0;
reg        inside_17 = 1'b0;

always @(posedge clk_pixel) begin
    glass_17        <= glass_full;
    foreground_17   <= foreground_full;
    cut_across_17   <= corner_distance_across(picture_column_16, CORNER_RADIUS);
    cut_down_17     <= corner_distance_down({1'b0, row_pipe[16]}, CORNER_RADIUS);
    paper_across_17 <= corner_distance_across(picture_column_16, paper_radius);
    paper_down_17   <= corner_distance_down({1'b0, row_pipe[16]}, paper_radius);
    inside_17       <= inside_16;
end

/* stage 18: the total brightness (8.8 fixed point), the dither value of the pixel (random) and the sums of squares for the corner cuts */
reg [16:0] total_18 = 17'd0;
reg [7:0]  dither_18 = 8'd0;
reg [7:0]  dither_second_18 = 8'd0;     /* a second, independent random value: two of them add up to a triangular dither */
reg [21:0] cut_squares_18 = 22'd0;
reg [21:0] paper_squares_18 = 22'd0;
reg        inside_18 = 1'b0;

/*
    A random dither value for every pixel and frame, from a 32 bit LFSR that advances 8 steps per clock (so neighbouring pixels are not shifted copies
    of each other). A fixed pattern would be averaged away by the scaler's filters and the contours of the gradient would come back; random noise of
    one level or less survives them and keeps the gradient smooth.
*/
reg [31:0] dither_state = 32'h1ACEB00C;

function [31:0] lfsr_advance_8(input [31:0] state);
    integer step;
    reg [31:0] next;

    begin
        next = state;

        for (step = 0; step < 8; step = step + 1)
            next = {next[30:0], next[31] ^ next[21] ^ next[1] ^ next[0]};

        lfsr_advance_8 = next;
    end
endfunction

always @(posedge clk_pixel) begin
    total_18          <= {1'b0, glass_17} + {1'b0, foreground_17} + {1'b0, band_add_17, 8'd0};
    dither_state      <= lfsr_advance_8(dither_state);
    dither_18         <= dither_state[15:8];
    dither_second_18  <= dither_state[23:16];
    cut_squares_18    <= cut_across_17 * cut_across_17 + cut_down_17 * cut_down_17;
    paper_squares_18  <= paper_across_17 * paper_across_17 + paper_down_17 * paper_down_17;
    inside_18         <= inside_17;
end

/* the colour of the phosphor */
reg [7:0] phosphor_red;
reg [7:0] phosphor_green;
reg [7:0] phosphor_blue;

always @* begin
    case (screen_colour)
        2'd0: begin
            phosphor_red   = 8'hf0;
            phosphor_green = 8'hf0;
            phosphor_blue  = 8'hf0;
        end

        2'd1: begin
            phosphor_red   = 8'h00;
            phosphor_green = 8'hf0;
            phosphor_blue  = 8'h00;
        end

        2'd2: begin
            phosphor_red   = 8'hff;
            phosphor_green = 8'hb0;
            phosphor_blue  = 8'h00;
        end

        default: begin
            phosphor_red   = 8'd108;
            phosphor_green = 8'd235;
            phosphor_blue  = 8'd194;
        end
    endcase
end

wire [15:0] total_clamped = total_18[16] ? 16'hffff : total_18[15:0];

/*
    Triangular dither: the sum of two random values, so the rounding value is spread over two levels (from -1 to +1) and not one. With one random
    value the noise of a pixel is strongest where the brightness lies half way between two levels and absent where it lies on one, so a smooth
    gradient shows rings of clean and noisy texture, one for every level (measured: the noise swings by a quarter). With the triangular spread the
    noise has the same strength at every brightness and the gradient has no rings. Half a level is taken off again, so the average is still exact.
    Black stays black.
*/
wire [16:0] triangular_dither = {9'd0, dither_18} + {9'd0, dither_second_18};         /* 0 .. 510, in 1/256 of a level */
localparam [24:0] HALF_LEVEL = 25'd32768;                                              /* half a level in the 16 fractional bits of the product */


/*
    The four front panel LEDs, drawn in the right strip of the monitor's frame, near the bottom (on the screen and on the paper alike): thin upright
    strips of 4 x 10 pixels, red, green, yellow, red like on the board, dark brown when off and in their colour when lit. They are drawn over the
    finished pixel, so the glass dither does not touch them.
*/
/* the row of four is 28 pixels wide, in the grey of the right strip of the frame (centred on column 1209, 3 pixels left of the middle) */
localparam [10:0] LED_LEFT   = 11'd1195;
localparam [9:0]  LED_TOP    = 10'd695;       /* the bottom of the LEDs is 15 pixels above the bottom of the canvas */
localparam [9:0]  LED_HEIGHT = 10'd10;
localparam [23:0] LED_COLOUR_RED    = 24'hb02418;
localparam [23:0] LED_COLOUR_GREEN  = 24'h20a834;
localparam [23:0] LED_COLOUR_YELLOW = 24'hb89a18;
localparam [23:0] LED_COLOUR_OFF    = 24'h3a281c;

reg [3:0] leds_lit_sync_1 = 4'd0;
reg [3:0] leds_lit_sync_2 = 4'd0;

always @(posedge clk_pixel) begin
    leds_lit_sync_1 <= leds_lit;
    leds_lit_sync_2 <= leds_lit_sync_1;
end

wire [10:0] led_offset = column_pipe[18] - LED_LEFT;                /* 8 pixels per LED: 4 lit, 4 gap */
wire [1:0]  led_number = led_offset[4:3];
wire        led_hit    = (column_pipe[18] >= LED_LEFT) && (led_offset < 11'd32) && !led_offset[2] &&
                         (row_pipe[18] >= LED_TOP) && (row_pipe[18] < LED_TOP + LED_HEIGHT);

reg [23:0] led_lit_colour;

always @* begin
    case (led_number)
        2'd1:    led_lit_colour = LED_COLOUR_GREEN;
        2'd2:    led_lit_colour = LED_COLOUR_YELLOW;
        default: led_lit_colour = LED_COLOUR_RED;
    endcase
end

wire [23:0] led_colour = leds_lit_sync_2[led_number] ? led_lit_colour : LED_COLOUR_OFF;

/*
    ---- the monitor's frame around the picture
    The frame is drawn first and the picture on top of it: the frame shows wherever the picture does not (the margins on both sides, the corners, and
    the curved edge of the bowed picture, so the curve of the frame can reach past the leftmost and rightmost points of the picture).
    The top left quarter of the frame is in a ROM of 161 x 360 pixels of 4 bits, an index into a palette of 16 grey levels (both made by
    tools/gen_bezel.py from a picture). The right side is the left one mirrored and the bottom half the top half mirrored: the column and the row are
    counted from the nearer edge of the canvas. The ROM address is row * 161 + column, made with shifts (161 = 128 + 32 + 1). Stage 14 forms the row
    start and the column, stage 15 the address, stage 16 reads the ROM, stage 17 the palette; stage 18 holds the grey level for the pixel.
*/
localparam [10:0] BEZEL_COLUMNS = 11'd161;
localparam BEZEL_PIXELS = 161 * 360;

(* ramstyle = "M10K" *) reg [3:0] bezel_rom [0:BEZEL_PIXELS - 1];

/* The 16 grey levels the 4-bit shades of the bezel picture stand for (tools/gen_bezel.py prints this table together with bezel.mem) */
function [7:0] bezel_palette(input [3:0] shade);
    case (shade)
        4'd0:  bezel_palette = 8'h00;
        4'd1:  bezel_palette = 8'h04;
        4'd2:  bezel_palette = 8'h25;
        4'd3:  bezel_palette = 8'h31;
        4'd4:  bezel_palette = 8'h3d;
        4'd5:  bezel_palette = 8'h49;
        4'd6:  bezel_palette = 8'h4e;
        4'd7:  bezel_palette = 8'h51;
        4'd8:  bezel_palette = 8'h52;
        4'd9:  bezel_palette = 8'h53;
        4'd10: bezel_palette = 8'h54;
        4'd11: bezel_palette = 8'h57;
        4'd12: bezel_palette = 8'h65;
        4'd13: bezel_palette = 8'h90;
        4'd14: bezel_palette = 8'hb8;
        default: bezel_palette = 8'hd7;
    endcase
endfunction

initial $readmemh(BEZEL_FILE, bezel_rom);

reg bezel_shown_1 = 1'b0;
reg bezel_shown = 1'b0;

always @(posedge clk_pixel) begin
    bezel_shown_1 <= bezel_on;
    bezel_shown   <= bezel_shown_1;
end

wire [10:0] bezel_column_13 = (column_pipe[13] < H_ACTIVE / 2) ? column_pipe[13] : (H_ACTIVE - 11'd1 - column_pipe[13]);
wire [9:0]  bezel_row_13 = (row_pipe[13] < V_ACTIVE / 2) ? row_pipe[13] : (V_ACTIVE - 10'd1 - row_pipe[13]);

reg [16:0] bezel_row_start_14 = 17'd0;
reg [7:0]  bezel_column_14 = 8'd0;
reg        bezel_in_rom_14 = 1'b0;
reg [16:0] bezel_address_15 = 17'd0;
reg        bezel_in_rom_15 = 1'b0;
reg [3:0]  bezel_shade_16 = 4'd0;
reg        bezel_in_rom_16 = 1'b0;
reg [7:0]  bezel_grey_17 = 8'd0;
reg [7:0]  bezel_grey_18 = 8'd0;

always @(posedge clk_pixel) begin
    bezel_row_start_14 <= {bezel_row_13, 7'd0} + {bezel_row_13, 5'd0} + {7'd0, bezel_row_13};
    bezel_column_14    <= bezel_column_13[7:0];
    bezel_in_rom_14    <= bezel_column_13 < BEZEL_COLUMNS;

    bezel_address_15   <= bezel_in_rom_14 ? bezel_row_start_14 + {9'd0, bezel_column_14} : 17'd0;
    bezel_in_rom_15    <= bezel_in_rom_14;

    bezel_shade_16     <= bezel_rom[bezel_address_15];
    bezel_in_rom_16    <= bezel_in_rom_15;

    bezel_grey_17      <= bezel_in_rom_16 ? bezel_palette(bezel_shade_16) : 8'd0;      /* beyond the ROM the frame is dark glass */
    bezel_grey_18      <= bezel_grey_17;
end

/*
    stage 19: the products of the total with the colour of the phosphor, plus the dither (the multiplications, nothing else); the corner cuts
    (the sums of squares against the squared radius); the LEDs, the paper's colour number and the frame's grey level for the overlay.
    stage 20: half a level is taken off the products (the rounding); the overlay: what is drawn instead of the screen, if anything.
    stage 21: the products are limited to 8 bits and the pixel is chosen: the overlay or the screen.
*/
wire [21:0] paper_radius_squared = round_corners ? 22'd784 : 22'd16;      /* the squared radius of the corners of the paper */

reg [24:0] red_sum_19 = 25'd0;
reg [24:0] green_sum_19 = 25'd0;
reg [24:0] blue_sum_19 = 25'd0;
reg        total_zero_19 = 1'b0;
reg        cut_19 = 1'b0;
reg        paper_cut_19 = 1'b0;
reg        inside_19 = 1'b0;
reg        led_hit_19 = 1'b0;
reg [23:0] led_colour_19 = 24'd0;
reg [7:0]  bezel_grey_19 = 8'd0;

always @(posedge clk_pixel) begin
    red_sum_19    <= total_clamped * phosphor_red   + {triangular_dither, 8'd0};
    green_sum_19  <= total_clamped * phosphor_green + {triangular_dither, 8'd0};
    blue_sum_19   <= total_clamped * phosphor_blue  + {triangular_dither, 8'd0};
    total_zero_19 <= (total_18 == 17'd0);

    cut_19        <= (cut_squares_18 > CORNER_RADIUS * CORNER_RADIUS) || round_cut_18;
    paper_cut_19  <= paper_squares_18 > paper_radius_squared;
    inside_19     <= inside_18;

    led_hit_19    <= led_hit;
    led_colour_19 <= led_colour;
    bezel_grey_19 <= bezel_grey_18;
end

/* the frame shows where the picture does not: outside the window of the picture, and where the picture is not (its edge, its corners) */
wire        in_window_19 = (column_pipe[19] >= BEZEL_WIDTH) && (column_pipe[19] < BEZEL_WIDTH + PICTURE_WIDTH);
wire        picture_visible_19 = in_window_19 && inside_19 && !(paper_shown ? paper_cut_19 : cut_19);
wire [23:0] bezel_pixel_19 = bezel_shown ? {bezel_grey_19, bezel_grey_19, bezel_grey_19} : 24'd0;
wire [23:0] paper_colour_19 = paper_colour_of(paper_colour_t'(paper_number_pipe[19]));

reg [24:0] red_product_20 = 25'd0;
reg [24:0] green_product_20 = 25'd0;
reg [24:0] blue_product_20 = 25'd0;
reg        overlay_valid_20 = 1'b0;
reg [23:0] overlay_pixel_20 = 24'd0;

always @(posedge clk_pixel) begin
    red_product_20   <= (total_zero_19 || red_sum_19   < HALF_LEVEL) ? 25'd0 : red_sum_19   - HALF_LEVEL;
    green_product_20 <= (total_zero_19 || green_sum_19 < HALF_LEVEL) ? 25'd0 : green_sum_19 - HALF_LEVEL;
    blue_product_20  <= (total_zero_19 || blue_sum_19  < HALF_LEVEL) ? 25'd0 : blue_sum_19  - HALF_LEVEL;

    overlay_valid_20 <= led_hit_19 || !picture_visible_19 || paper_shown;

    if (led_hit_19)
        overlay_pixel_20 <= led_colour_19;
    else if (!picture_visible_19)
        overlay_pixel_20 <= bezel_pixel_19;
    else
        overlay_pixel_20 <= paper_colour_19;
end

/* the screen's pixel: the glass and the foreground in the colour of the phosphor, rounded with the dither */
wire [23:0] screen_colour_20 = {red_product_20[24]   ? 8'hff : red_product_20[23:16],
                                green_product_20[24] ? 8'hff : green_product_20[23:16],
                                blue_product_20[24]  ? 8'hff : blue_product_20[23:16]};

reg [23:0] pixel_21 = 24'd0;

always @(posedge clk_pixel)
    pixel_21 <= overlay_valid_20 ? overlay_pixel_20 : screen_colour_20;

/* ---- the sync signals, delayed by the same twenty-one stages; the output register adds the last clock */
reg [2:0] sync_pipeline [0:PIPELINE_STAGES - 1];
integer stage;

wire canvas_de = (column < H_ACTIVE) && (row < V_ACTIVE);
wire canvas_hs = (column >= HS_START) && (column < HS_END);
wire canvas_vs = (row >= VS_START) && (row < VS_END);

always @(posedge clk_pixel) begin
    sync_pipeline[0] <= {canvas_de, canvas_hs, canvas_vs};

    for (stage = 1; stage < PIPELINE_STAGES; stage = stage + 1)
        sync_pipeline[stage] <= sync_pipeline[stage - 1];

    {de, hs, vs} <= sync_pipeline[PIPELINE_STAGES - 1];
    rgb          <= sync_pipeline[PIPELINE_STAGES - 1][2] ? pixel_21 : 24'd0;
end

endmodule
