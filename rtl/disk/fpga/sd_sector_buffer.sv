/*
    Copyright (c) 2026 by Hrvoje Cavrak

    One 512 byte sector buffer of the disk image store (FPGA only): 256 words of 16 bit as a true dual port RAM with a byte write enable per port. It
    becomes M10K block RAM, one block per byte lane.

      port A  the HPS side (hps_io sd_buff_*): the sector arrives from or goes to the image file on the SD card, 16 bits at a time
      port B  the bridge's state machine (sd_image_bridge.sv): the 64 bit words of the disk controllers are read and written 16 bits at a time

    Both ports read with one clock of delay (registered read). A write and a read of the same address on the two ports in the same clock is not
    resolved (no_rw_check): the two sides never do that, because the HPS only touches a sector buffer while the bridge waits for it.
*/
module sd_sector_buffer
(
    input             clk,

    input      [7:0]  hps_address,
    input      [15:0] hps_write_data,
    input      [1:0]  hps_byte_write,           /* bit 0: low byte, bit 1: high byte */
    output reg [15:0] hps_read_data,

    input      [7:0]  bridge_address,
    input      [15:0] bridge_write_data,
    input      [1:0]  bridge_byte_write,
    output reg [15:0] bridge_read_data
);

(* ramstyle = "M10K, no_rw_check" *) reg [7:0] low_byte_ram  [0:255];
(* ramstyle = "M10K, no_rw_check" *) reg [7:0] high_byte_ram [0:255];

always @(posedge clk) begin
    if (hps_byte_write[0])
        low_byte_ram[hps_address] <= hps_write_data[7:0];

    hps_read_data[7:0] <= low_byte_ram[hps_address];
end

always @(posedge clk) begin
    if (hps_byte_write[1])
        high_byte_ram[hps_address] <= hps_write_data[15:8];

    hps_read_data[15:8] <= high_byte_ram[hps_address];
end

always @(posedge clk) begin
    if (bridge_byte_write[0])
        low_byte_ram[bridge_address] <= bridge_write_data[7:0];

    bridge_read_data[7:0] <= low_byte_ram[bridge_address];
end

always @(posedge clk) begin
    if (bridge_byte_write[1])
        high_byte_ram[bridge_address] <= bridge_write_data[15:8];

    bridge_read_data[15:8] <= high_byte_ram[bridge_address];
end

endmodule
