/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The boot ROM is not in the core: it is the two 8 KB EPROMs of the board, 15C (high byte) and 14C (low byte), each loaded from a file by
    rom_loader.sv. Main_MiSTer sends boot0.rom (index 0) and boot1.rom (index 0x40) from the core's folder at start. The machine stays in reset while a file is coming, and until both halves have arrived.
*/
module boot_rom_files
#(
    parameter BUILT_IN = 0              /* 1: the ROM is in the design (simulators), no file is needed */
)
(
    input         clk_sys,
    input         ioctl_download,  /* hps_io file download */
    input         ioctl_wr,
    input  [26:0] ioctl_addr,
    input  [15:0] ioctl_dout,
    input  [15:0] ioctl_index,
    output        rom_high_write,  /* write port of 15C */
    output [12:0] rom_high_address,
    output [7:0]  rom_high_data,
    output        rom_low_write,  /* write port of 14C */
    output [12:0] rom_low_address,
    output [7:0]  rom_low_data,
    output        rom_loading,  /* a download or a clear is in progress: hold the machine in reset */
    output        boot_rom_ready  /* both halves are there (or the ROM is built in) */
);

wire        rom_high_loading;
wire        rom_high_loaded;
wire        rom_low_loading;
wire        rom_low_loaded;
assign rom_loading = rom_high_loading | rom_low_loading;
assign boot_rom_ready = BUILT_IN | (rom_high_loaded & rom_low_loaded);

/* the two EPROMs: 15C (high byte) is file index 0 and boot0.rom, 14C (low byte) is OSD index 1 and boot1.rom, which Main_MiSTer sends as index 0x40 */
rom_loader rom_15c_loader
(
    .clk            (clk_sys),
    .sel            (ioctl_index[7:0] == 8'h00),
    .ioctl_download (ioctl_download),
    .ioctl_wr       (ioctl_wr),
    .ioctl_addr     (ioctl_addr),
    .ioctl_dout     (ioctl_dout),
    .we             (rom_high_write),
    .waddr          (rom_high_address),
    .wdata          (rom_high_data),
    .loading        (rom_high_loading),
    .loaded         (rom_high_loaded)
);

rom_loader rom_14c_loader
(
    .clk            (clk_sys),
    .sel            (ioctl_index[7:0] == 8'h01 || ioctl_index[7:0] == 8'h40),
    .ioctl_download (ioctl_download),
    .ioctl_wr       (ioctl_wr),
    .ioctl_addr     (ioctl_addr),
    .ioctl_dout     (ioctl_dout),
    .we             (rom_low_write),
    .waddr          (rom_low_address),
    .wdata          (rom_low_data),
    .loading        (rom_low_loading),
    .loaded         (rom_low_loaded)
);

endmodule
