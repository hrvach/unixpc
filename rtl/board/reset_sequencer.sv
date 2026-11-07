/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Reset of the machine (FPGA only): any reset request holds the board in reset for a time, and the reset lasts further until the host time has been loaded
    into the real time clock (rtc_conv.sv). The page map RAM is cleared while the board is in reset, which needs 1024 clocks or more; the counter gives
    65535 clocks (about 3 ms).

    Mounting a hard disk image is a reset request too (the disk changes under the running system), as is a boot ROM download.
*/
module reset_sequencer
(
    input  clk,
    input  reset_request,                       /* any source: button, OSD, PLL not locked, disk mounted, boot ROM loading */
    input  rtc_ready,                           /* the real time clock has the host time */

    output reg reset_stretched = 1'b1,          /* the request, held for the reset time; also the reset of the clock converter */
    output     reset                            /* the reset of the whole machine */
);

reg [15:0] clocks_left = 16'hffff;

always @(posedge clk) begin
    if (reset_request)
        clocks_left <= 16'hffff;
    else if (clocks_left != 16'd0)
        clocks_left <= clocks_left - 16'd1;

    reset_stretched <= (clocks_left != 16'd0);
end

assign reset = reset_stretched | ~rtc_ready;

endmodule
