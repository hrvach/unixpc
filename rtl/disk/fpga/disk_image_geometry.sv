/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Where a sector is in the disk image file (FPGA only): the images are plain sector dumps, 512 bytes per sector, no headers.

      hard disk  byte offset = (cylinder x sectors per cylinder + head x sectors per track + sector number) x 512
                 hd_geom (OSD): 0 = 8 heads x 16 sectors (128 per cylinder), 1 = 17 sectors per track, 136 per cylinder, 2 = 17 sectors per track, 272
                 per cylinder
      floppy     LBA = (track x sides + side) x sectors per track + sector number - 1;  sectors per track 10 (800 KB-less 400 KB format) or 8 (OSD
                 setting)
*/
module disk_image_geometry
(
    input  [1:0]  hard_disk_geometry,        /* OSD: see above */
    input         eight_sector_floppy,       /* OSD: 1 = 8 sectors per track, 0 = 10 */

    /* the WD1010's task file */
    input  [10:0] cylinder,
    input  [2:0]  head,
    input  [7:0]  sector_number,

    /* the WD2797's registers */
    input  [7:0]  floppy_track,              /* where the head is */
    input         floppy_side,
    input  [7:0]  floppy_sector,

    output [27:0] hard_disk_sector_offset,   /* byte offset of the addressed sector */
    output [27:0] floppy_sector_offset,
    output [18:0] floppy_track_first_sector, /* LBA of the first sector of the track and side, for the format fill */
    /* 64 bit words in one track (8 or 10 sectors of 512 bytes, 8 bytes per word): the length of the format fill */
    output [15:0] floppy_track_words,
    output [4:0]  hard_disk_sectors_per_track,
    output [28:0] hard_disk_track_fill_base, /* image word address of the first word of the hard disk track (cylinder and head of the task file) */
    output [15:0] hard_disk_track_words,     /* 64 bit words in one hard disk track */
    output [28:0] floppy_track_fill_base     /* image word address of the first word of the floppy track */
);

wire [18:0] floppy_lba = eight_sector_floppy ? (16 * floppy_track + 8 * floppy_side + floppy_sector - 1)
                                              : (20 * floppy_track + 10 * floppy_side + floppy_sector - 1);

assign floppy_sector_offset      = {floppy_lba, 9'd0};
assign floppy_track_first_sector = eight_sector_floppy ? (16 * floppy_track + 8 * floppy_side)
                                                       : (20 * floppy_track + 10 * floppy_side);
assign floppy_track_words        = eight_sector_floppy ? 16'd512 : 16'd640;

reg [31:0] hard_disk_sector;          /* 32 bit like the arithmetic of the original: the shift below drops what does not fit */

always @* begin
    case (hard_disk_geometry)
        2'd1:    hard_disk_sector = cylinder * 136 + head * 17 + sector_number;
        2'd2:    hard_disk_sector = cylinder * 272 + head * 17 + sector_number;
        default: hard_disk_sector = cylinder * 128 + head * 16 + sector_number;
    endcase
end

assign hard_disk_sector_offset = hard_disk_sector << 9;

assign hard_disk_sectors_per_track = (hard_disk_geometry == 2'd0) ? 5'd16 : 5'd17;
assign hard_disk_track_words       = {2'd0, hard_disk_sectors_per_track, 6'd0} ;           /* sectors x 64 words of 8 bytes */

reg [31:0] hard_disk_track_first_sector;

always @* begin
    case (hard_disk_geometry)
        2'd1:    hard_disk_track_first_sector = cylinder * 136 + head * 17;
        2'd2:    hard_disk_track_first_sector = cylinder * 272 + head * 17;
        default: hard_disk_track_first_sector = cylinder * 128 + head * 16;
    endcase
end

wire [27:0] hard_disk_track_offset = hard_disk_track_first_sector << 9;

assign hard_disk_track_fill_base = {4'b0011, hard_disk_track_offset[27:3]};
assign floppy_track_fill_base    = {13'b0011_1111_1111_1, 16'(floppy_track_first_sector[12:0] * 16'd64)};

endmodule
