/*
    Copyright (c) 2026 by Hrvoje Cavrak

    "Eject floppy" of the OSD (FPGA only): the drive reports not ready until another image is mounted. The HPS keeps the old image mounted, which the core
    simply stops using. The OSD entry is a trigger: its status bit toggles to 1 for the click.
*/
module floppy_eject
(
    input  clk,
    input  eject_request,                       /* OSD trigger */
    input  floppy_image_mounted,                /* hps_io: a floppy image has just been mounted (one clock pulse) */
    input  floppy_image_present,                /* an image is mounted in the floppy drive */

    output floppy_present,                      /* the drive has a floppy: an image, and not ejected since */
    output reset_after_swap                     /* one clock pulse: an image was mounted after an eject, so the machine restarts and boots from it */
);

reg ejected = 1'b0;
reg eject_request_before = 1'b0;

always @(posedge clk) begin
    eject_request_before <= eject_request;

    if (eject_request && !eject_request_before)
        ejected <= 1'b1;

    if (floppy_image_mounted)               /* a new image ends the eject; it wins over a request in the same clock */
        ejected <= 1'b0;
end

assign floppy_present = floppy_image_present & ~ejected;

/*
    The boot ROM looks for a floppy only when the machine starts. Ejecting and mounting another image is the swap of the boot floppy, so the new image
    restarts the machine; mounting an image that did not follow an eject (the first image at core start, or a floppy for the running system) does not.
*/
assign reset_after_swap = floppy_image_mounted & ejected;

endmodule
