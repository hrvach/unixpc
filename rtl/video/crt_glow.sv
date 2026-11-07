/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Glow of the CRT effects (FPGA only): the light of a lit pixel spills into its neighbours.

    The brightness is blurred horizontally with 5 taps [1 4 6 4 1] / 16 and vertically with 3 taps [1 2 1] / 4 (a binomial blur), and the result is
    added back to the sharp brightness, scaled by the glow mode: 0 = off, 1 = low (half), 2 = mid (three quarters), 3 = high (all of it).

    The vertical blur needs the lines above, so two circular line buffers of exactly one line period hold the previous line ({de, hs, vs, sharp
    brightness, horizontally blurred}) and the line before ({horizontally blurred}); a read before the write gives the previous line. The vertical
    centre is the line y - 1, so this stage delays the picture by one line.

    Glow noise: the brightness of every pixel's halo is raised or lowered by 1/4, 3/8 or 1/2 of itself (low, mid, high), random on / off and random
    sign, from the pixel noise LFSR. A strong halo next to a large lit area (112 or more) gets a quarter of the variation: the grain belongs to thin
    strokes, not to the edge of an area.
*/
module crt_glow
(
    input             clk,
    input  [7:0]      brightness,           /* from the persistence stage */
    input             de,
    input             hs,
    input             vs,
    input  [1:0]      glow_mode,            /* 0 off, 1 low (half), 2 mid (three quarters), 3 high (all of it) */
    input  [1:0]      glow_noise,           /* 0 off, 1 low, 2 mid, 3 high */
    input  [15:0]     pixel_noise,          /* free running LFSR, one step per pixel clock */
    input  [9:0]      column,               /* line period counter, see crt_fx.sv */
    input  [9:0]      column_delayed,

    output reg [7:0]  lum = 8'd0,
    output reg        de_out = 1'b0,
    output reg        hs_out = 1'b0,
    output reg        vs_out = 1'b0
);

localparam LINE_CLOCKS = 896;

/* ---- part 1: horizontal 5 tap blur [1 4 6 4 1] / 16; the centre sample (the sharp pixel) is sample_2 */
reg [7:0] sample_0 = 8'd0;
reg [7:0] sample_1 = 8'd0;
reg [7:0] sample_2 = 8'd0;
reg [7:0] sample_3 = 8'd0;
reg [7:0] sample_4 = 8'd0;
reg [2:0] timing_pipe_0 = 3'd0;            /* {de, hs, vs} aligned with the samples, to find the ones that belong to the centre */
reg [2:0] timing_pipe_1 = 3'd0;
reg [2:0] timing_pipe_2 = 3'd0;
reg [7:0] sharp_3 = 8'd0;
reg [7:0] blurred_h_3 = 8'd0;
reg       de_3 = 1'b0;
reg       hs_3 = 1'b0;
reg       vs_3 = 1'b0;

/* 1 4 6 4 1: the centre is 2 + 4 times the sample */
wire [11:0] horizontal_sum = {4'd0, sample_0}
                           + {2'd0, sample_1, 2'd0}
                           + {3'd0, sample_2, 1'd0} + {2'd0, sample_2, 2'd0}
                           + {2'd0, sample_3, 2'd0}
                           + {4'd0, sample_4};

always @(posedge clk) begin
    sample_0 <= brightness;
    sample_1 <= sample_0;
    sample_2 <= sample_1;
    sample_3 <= sample_2;
    sample_4 <= sample_3;

    timing_pipe_0 <= {de, hs, vs};
    timing_pipe_1 <= timing_pipe_0;
    timing_pipe_2 <= timing_pipe_1;

    /* sample_2 is the centre of the five samples, so the signals that belong to it are the ones delayed by three clocks */
    blurred_h_3 <= horizontal_sum[11:4];
    sharp_3     <= sample_2;
    {de_3, hs_3, vs_3} <= timing_pipe_2;
end

/*
    ---- part 2: the line buffers. At this point current_line (line y), previous_line (line y - 1) and line_before (line y - 2) belong to the same
    column.
*/
reg [18:0] previous_line_buffer [0:LINE_CLOCKS - 1];        /* {de, hs, vs, sharp, blurred} */
reg [7:0]  line_before_buffer   [0:LINE_CLOCKS - 1];        /* blurred */
reg [18:0] previous_line = 19'd0;
reg [7:0]  line_before = 8'd0;
reg [18:0] current_line = 19'd0;
reg [7:0]  current_blurred_h = 8'd0;

always @(posedge clk) begin
    previous_line <= previous_line_buffer[column];
    line_before   <= line_before_buffer[column];
    current_line  <= {de_3, hs_3, vs_3, sharp_3, blurred_h_3};
    previous_line_buffer[column_delayed] <= current_line;
    line_before_buffer[column_delayed]   <= previous_line[7:0];

    current_blurred_h <= blurred_h_3;
end

wire [9:0] vertical_sum = {2'd0, current_blurred_h} + {1'd0, previous_line[7:0], 1'd0} + {2'd0, line_before};
wire [7:0] blurred      = vertical_sum[9:2];

/* the glow that is added back: nothing, half of the blur or all of it */
reg [8:0] glow;

always @* begin
    case (glow_mode)
        2'd3:    glow = {1'b0, blurred};
        2'd2:    glow = {1'b0, blurred - (blurred >> 2)};
        2'd1:    glow = {1'b0, blurred >> 1};
        default: glow = 9'd0;
    endcase
end

/* glow noise: the size of the random change of the halo */
reg [8:0] noise_size;

always @* begin
    if (!pixel_noise[4]) begin
        noise_size = 9'd0;
    end else begin
        case (glow_noise)
            2'd1:    noise_size = glow >> 2;
            2'd2:    noise_size = (glow >> 2) + (glow >> 3);
            2'd3:    noise_size = glow >> 1;
            default: noise_size = 9'd0;
        endcase
    end
end

wire [8:0] noise_change     = (glow >= 9'd112) ? (noise_size >> 2) : noise_size;
wire [9:0] glow_with_noise  = pixel_noise[5] ? {1'b0, glow} + {1'b0, noise_change} : {1'b0, glow} - {1'b0, noise_change};
wire [9:0] sharp_plus_glow  = {2'b00, previous_line[15:8]} + glow_with_noise;

always @(posedge clk) begin
    lum <= (sharp_plus_glow[9] | sharp_plus_glow[8]) ? 8'd255 : sharp_plus_glow[7:0];
    {de_out, hs_out, vs_out} <= previous_line[18:16];
end

endmodule
