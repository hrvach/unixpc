/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The last stage of the CRT effects (FPGA only): the noise of the picture and its slow flicker. The geometry, the glass, the edges, the rolling band
    and the line wobble are done by the canvas (rtl/video/video.sv), at the resolution of the output.

      - noise: every pixel that carries light gets a random brightness variation, new for every pixel and frame, low +-6 %, mid +-9 %, high +-12 %. A
        pixel carries light when it is lit or a dark one next to a lit pixel (the glow has given it a luminance); the dark background stays quiet.
        Inside bright areas (this pixel and the two before it at 224 or more) the noise is cut to a quarter: grain looks good on text, not on large
        lit areas.
      - slow flicker: the whole lit picture breathes by about 1 to 2 % with a period of 64 frames (about a second): a counter that steps at every
        vertical sync, run through a triangle. It follows the noise setting (low, mid, high = 1, 2, 3 eighths per step); with noise off there is none.

    The brightness goes through four register stages (the factor, the product, two plain delays); de / hs / vs follow with the same latency.
*/
module crt_noise
(
    input             clk,
    input  [7:0]      lum_in,
    input             de_in,
    input             hs_in,
    input             vs_in,
    input  [1:0]      noise,                /* 0 off, 1 low, 2 mid, 3 high */
    input  [15:0]     pixel_noise,          /* a new random value for every pixel (crt_fx.sv) */

    output reg [7:0]  lum = 8'd0,
    output reg        de = 1'b0,
    output reg        hs = 1'b0,
    output reg        vs = 1'b0
);

localparam signed [10:0] FACTOR_UNITY = 11'sd256;       /* brightness factor 256 = unchanged */

/* ---- slow flicker */
reg        vertical_sync_previous = 1'b0;
reg [5:0]  flicker_phase = 6'd0;

always @(posedge clk) begin
    vertical_sync_previous <= vs_in;

    if (vs_in && !vertical_sync_previous)
        flicker_phase <= flicker_phase + 6'd1;
end

wire [4:0]         flicker_triangle = flicker_phase[5] ? ~flicker_phase[4:0] : flicker_phase[4:0];             /* 0 up to 31 and back */
wire signed [6:0]  flicker_centred  = $signed({2'b00, flicker_triangle}) - 7'sd16;                             /* -16 .. 15 */
wire signed [10:0] flicker_term     = (flicker_centred * $signed({1'b0, noise})) >>> 3;                        /* in 1/256 of the brightness */

/* ---- noise: nibble - 8 is -8..7 sixteenths of 1/16, i.e. +-3 % of the factor 256 */
wire signed [5:0]  noise_sample   = $signed({2'b00, pixel_noise[3:0]}) - 6'sd8;
wire signed [10:0] noise_extended = {{5{noise_sample[5]}}, noise_sample};
reg  signed [10:0] noise_term;

always @* begin
    case (noise)
        2'd1:    noise_term = noise_extended <<< 1;                         /* low +-6 % */
        2'd2:    noise_term = (noise_extended <<< 1) + noise_extended;      /* mid +-9 % */
        2'd3:    noise_term = noise_extended <<< 2;                         /* high +-12 % */
        default: noise_term = 11'sd0;
    endcase
end

reg  [7:0] lum_1 = 8'd0;
reg  [7:0] lum_2 = 8'd0;
wire       flat_area = (lum_in >= 8'd224) && (lum_1 >= 8'd224) && (lum_2 >= 8'd224);

always @(posedge clk) begin
    lum_1 <= lum_in;
    lum_2 <= lum_1;
end

/* the factor of the brightness: 256 = unchanged; only pixels that carry light are touched */
reg signed [10:0] factor;

always @* begin
    if (lum_in == 8'd0)
        factor = FACTOR_UNITY;
    else if (flat_area)
        factor = FACTOR_UNITY + (noise_term >>> 2) + flicker_term;
    else
        factor = FACTOR_UNITY + noise_term + flicker_term;
end

/* ---- stage 1: the brightness (times 255 / 256) and the factor are registered */
wire [15:0] scaled_lum = {8'd0, lum_in} * 16'd255;

reg [7:0]  lum_1_stage = 8'd0;
reg signed [10:0] factor_1_stage = 11'sd0;
reg        de_1_stage = 1'b0;
reg        hs_1_stage = 1'b0;
reg        vs_1_stage = 1'b0;

always @(posedge clk) begin
    lum_1_stage    <= scaled_lum[15:8];
    factor_1_stage <= factor;
    de_1_stage     <= de_in;
    hs_1_stage     <= hs_in;
    vs_1_stage     <= vs_in;
end

/* ---- stage 2: the product with the factor, limited to white (with the noise off: the brightness as it is) */
wire [18:0] noisy_product = {11'd0, lum_1_stage} * {8'd0, factor_1_stage[10:0]};

reg [7:0] lum_2_stage = 8'd0;
reg       de_2_stage = 1'b0;
reg       hs_2_stage = 1'b0;
reg       vs_2_stage = 1'b0;

always @(posedge clk) begin
    if (noise == 2'd0)
        lum_2_stage <= lum_1_stage;
    else if (noisy_product[18:16] != 3'd0)
        lum_2_stage <= 8'd255;
    else
        lum_2_stage <= noisy_product[15:8];

    de_2_stage <= de_1_stage;
    hs_2_stage <= hs_1_stage;
    vs_2_stage <= vs_1_stage;
end

/* ---- stages 3 and 4: two plain delays (the latency of the screen stage this one replaced) */
reg [7:0] lum_3_stage = 8'd0;
reg       de_3_stage = 1'b0;
reg       hs_3_stage = 1'b0;
reg       vs_3_stage = 1'b0;

always @(posedge clk) begin
    lum_3_stage <= lum_2_stage;
    de_3_stage  <= de_2_stage;
    hs_3_stage  <= hs_2_stage;
    vs_3_stage  <= vs_2_stage;

    lum <= lum_3_stage;
    de  <= de_3_stage;
    hs  <= hs_3_stage;
    vs  <= vs_3_stage;
end

endmodule
