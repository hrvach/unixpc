/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Core hotkeys from the key events of hps_io (FPGA only; ps2_key is generated on clk_sys by hps_io, so there is no clock crossing):
      Alt+F1   show the screen
      Alt+F2   show the printer paper (the printer itself is always on line)
    The Alt key can be either of the two: the left one is scancode 11, the right one is the same code with the extended flag.
*/
module hotkeys
(
    input         clk,
    input  [10:0] ps2_key,                      /* {toggle, pressed, extended, scancode} */

    output reg    paper_view = 1'b0
);

localparam [7:0] SCANCODE_ALT = 8'h11;
localparam [7:0] SCANCODE_F1  = 8'h05;
localparam [7:0] SCANCODE_F2  = 8'h06;

wire       key_toggle = ps2_key[10];
wire       key_down   = ps2_key[9];
wire       key_extended = ps2_key[8];
wire [7:0] key_scancode = ps2_key[7:0];

reg key_toggle_before = 1'b0;
reg left_alt_down     = 1'b0;
reg right_alt_down    = 1'b0;

wire new_key_event = (key_toggle != key_toggle_before);

always @(posedge clk) begin
    key_toggle_before <= key_toggle;

    if (new_key_event && key_scancode == SCANCODE_ALT) begin
        if (key_extended)
            right_alt_down <= key_down;
        else
            left_alt_down <= key_down;
    end

    if (new_key_event && key_down && !key_extended && (left_alt_down | right_alt_down)) begin
        if (key_scancode == SCANCODE_F1)
            paper_view <= 1'b0;

        if (key_scancode == SCANCODE_F2)
            paper_view <= 1'b1;
    end
end

endmodule
