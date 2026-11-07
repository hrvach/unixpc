/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Disk control register (write only, address 4E0000), the latch on the logic board that selects and resets the two disk controllers (manual: disk
    control register).

    Bits, as the kernel and the diagnostic use them:
      D7      floppy controller (WD2797) reset, active low:   0 = held in reset
      D4      hard disk controller (WD1010) reset, active low: 0 = held in reset
      D3      drive select: 0 = floppy drive, 1 = hard disk (the PAL's own direction input is derived from the running command instead, see
              disk_ctrl_pins.sv)
      D2:D0   hard disk head select; the WD1010's SDH register takes these three bits from the same CPU write (wd1010_controller.sv)

    What it deliberately does not model: the motor, side and precompensation bits (nothing in the FPGA looks at them).
*/
module disk_control_register
import unixpc_registers_pkg::*;
(
    input        clk,
    input        rst,                       /* system reset: the register clears (both controllers in reset) */
    input        write,                     /* the CPU writes the register (selected by the address decoder) */
    input  [7:0] write_data,                /* D7:D0 of the CPU data bus */

    output       floppy_controller_enabled, /* D7: 1 = WD2797 out of reset */
    output       hard_disk_controller_enabled, /* D4: 1 = WD1010 out of reset */
    output [2:0] head_select                   /* D2:D0: the head the drive has selected */
);

disk_control_reg_t register_value = 8'h00;       /* layout: unixpc_registers_pkg.sv */

assign floppy_controller_enabled     = register_value.FDRST;
assign hard_disk_controller_enabled  = register_value.HDRST;
assign head_select                   = register_value.HDHSEL;

always @(posedge clk) begin
    if (write) begin
        register_value <= write_data;
    end

    if (rst) begin
        register_value <= 8'h00;
    end
end

endmodule
