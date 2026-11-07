/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Boot ROM half from a file. The boot ROM of the real machine is two 8 KB EPROMs, 15C on the high byte of the 16 bit bus and 14C on the low byte; the
    core loads each from its own file (boot0.rom = 15C, boot1.rom = 14C in the core's folder, or the two "Boot ROM" entries of the OSD) and has two of
    these loaders, one per half.

    hps_io (16 bit mode) delivers a file as words with a byte address that grows by 2 (ioctl_addr, ioctl_dout, ioctl_wr), the first byte of the pair in
    the LOW half of the word. A word holds two bytes of the file and so two ROM bytes: the first is written at once, the second one clock later. A
    file shorter than 8 KB would leave the end of the old contents behind, so when the download ends the bytes above the last one written are cleared;
    the caller keeps the machine in reset for long enough (the 3B1 glue holds reset for 65535 clocks after every download) and the clear takes at most
    8192 clocks. Bytes beyond 8 KB are ignored.
*/
module rom_loader
(
    input             clk,
    input             sel,              /* this download is for this half (file index) */
    input             ioctl_download,
    input             ioctl_wr,
    input      [26:0] ioctl_addr,
    input      [15:0] ioctl_dout,
    output reg        we,
    output reg [12:0] waddr,
    output reg [7:0]  wdata,
    output            loading,          /* download or clear in progress */
    output reg        loaded = 1'b0     /* a file has been loaded (a download that wrote a byte has finished); low during the next one */
);
reg        download_before = 1'b0;          /* ioctl_download & sel one clock ago, to see the end of a download */
reg        received_data = 1'b0;            /* the file wrote at least one byte */
reg        clearing_rest = 1'b0;            /* the bytes after the file are being cleared */
reg        second_byte_waiting = 1'b0;      /* the second byte of the last word is written in this clock */
reg [7:0]  second_byte = 8'd0;
reg [12:0] second_byte_address = 13'd0;
reg [13:0] first_unwritten_byte = 14'd0;    /* first byte not written by the file */
wire       downloading = ioctl_download & sel;
assign loading = downloading | clearing_rest;

always @(posedge clk) begin
    we              <= 1'b0;
    download_before <= downloading;

    if (downloading & ~download_before) begin
        received_data        <= 1'b0;
        first_unwritten_byte <= 14'd0;
        clearing_rest        <= 1'b0;
        loaded               <= 1'b0;       /* a new file is coming. This is also what keeps `loaded` a register: one that is only ever set would be
                                               taken for a constant 1 by Quartus (power-up don't care) and the core would think it has a ROM */
    end

    if (download_before & ~downloading) begin      /* download finished: clear the rest */
        if (received_data && first_unwritten_byte < 14'd8192) clearing_rest <= 1'b1;
        if (received_data) loaded <= 1'b1;
    end

    if (second_byte_waiting) begin
        we                  <= 1'b1;
        waddr               <= second_byte_address;
        wdata               <= second_byte;
        second_byte_waiting <= 1'b0;
    end else if (downloading & ioctl_wr & (ioctl_addr < 27'd8192)) begin
        we                   <= 1'b1;
        waddr                <= ioctl_addr[12:0];
        wdata                <= ioctl_dout[7:0];
        second_byte_waiting  <= 1'b1;
        second_byte          <= ioctl_dout[15:8];
        second_byte_address  <= ioctl_addr[12:0] + 13'd1;
        received_data        <= 1'b1;
        first_unwritten_byte <= {1'b0, ioctl_addr[12:0]} + 14'd2;
    end else if (clearing_rest) begin
        we    <= 1'b1;
        waddr <= first_unwritten_byte[12:0];
        wdata <= 8'h00;
        if (first_unwritten_byte == 14'd8191) clearing_rest <= 1'b0;
        first_unwritten_byte <= first_unwritten_byte + 14'd1;
    end
end
endmodule
