/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The media side of a disk transfer (FPGA only): the disk image behind the controllers.

    The controllers move bytes (wd_dma_port.sv); the image store moves 64 bit words (the board's sd_image_bridge.sv: a one sector cache in front of
    the SD card, a sector change takes milliseconds). This module sits between the two:
      disk -> RAM  the next byte of the sector comes from an 8 byte chunk that is fetched from the image and kept; the byte pointer says which byte is
                   next.
                   A Read Address transfer has no image data, it serves the 6 bytes of the ID field.
      RAM -> disk  the bytes of the controller are assembled to words (the first byte of a word is the high byte); a word waits in `word_pending`
                   until the image store has taken it
                   (the controller model waits meanwhile, see media_wait in disk_dma_path.sv). A Write Track transfer only parses the format stream:
                   the first data byte after a data address mark FB / F8 is the fill byte of the track (the sequencer fills the track in the image
                   when the stream is over).

    Image address map on the store's port (64 bit word addresses), the layout the first engine (archived/disk_ctrl.sv) used and the images on the SD
    card follow:
      hard disk  {4'b0011, byte_offset[27:3]}                    floppy  {13'b0011_1111_1111_1, byte_offset[18:3]}
    The images are plain sector dumps; a word in the image holds two bytes of the machine's word swapped (the byte enables select the lane).
*/
module disk_media_access
(
    input         clk,
    input         rst,

    /* the transfer, from the sequencer */
    input         transfer_start,             /* pulse: a new transfer starts, everything restarts */
    input         transfer_running,           /* the controllers' strobes run and the engine has not ended the transfer */
    input         ram_to_disk,
    input         floppy_transfer,
    input         read_address_transfer,      /* Read Address: the bytes are the ID field */
    input         format_transfer,            /* Write Track: the stream is only parsed */
    input  [27:0] base_offset,                /* byte offset of the first byte in the image */
    input  [7:0]  id_track,                   /* the ID field of a Read Address */
    input  [7:0]  id_side,
    input  [7:0]  id_sector,
    input  [7:0]  id_size_code,
    input  [15:0] id_crc,

    /* the byte streams of the controller model */
    input         source_take,                /* disk -> RAM: the controller took the byte */
    output        source_valid,
    output [7:0]  source_byte,
    input         sink_valid,                 /* RAM -> disk: the controller delivers a byte */
    input  [7:0]  sink_byte,
    output        word_pending,               /* a word waits to go to the image */
    output        high_byte_held,             /* the first (high) byte of a word has been received (debug) */
    output [7:0]  high_byte,                  /* (debug) */
    output        image_idle,                 /* no image access in flight */
    output [2:0]  image_state,                /* 0 idle, 2 read request, 3 read wait, 4 write, 5 write wait (debug) */
    output [7:0]  format_fill_byte,           /* Write Track: the fill byte found in the stream */

    /* the image store (the 64 bit word port) */
    output reg        ddr_rd = 1'b0,
    output reg        ddr_we = 1'b0,
    output reg [28:0] ddr_addr,
    output reg [63:0] ddr_din,
    output reg [7:0]  ddr_be,
    input      [63:0] ddr_dout,
    input             ddr_ready,
    input             ddr_busy
);

typedef enum logic [2:0] {
    IMAGE_IDLE         = 3'd0,
    IMAGE_READ_REQUEST = 3'd2,   /* (state 1 is not used) */
    IMAGE_READ_WAIT    = 3'd3,
    IMAGE_WRITE        = 3'd4,
    IMAGE_WRITE_WAIT   = 3'd5
} image_access_state_t;

localparam [7:0] DATA_MARK_FB = 8'hfb;           /* data address marks of the format stream */
localparam [7:0] DATA_MARK_F8 = 8'hf8;
localparam [7:0] DEFAULT_FILL_BYTE = 8'he5;

image_access_state_t state = IMAGE_IDLE;
reg [18:0] byte_pointer = 19'd0;                  /* bytes moved to / from the media in this transfer */
reg [63:0] chunk = 64'd0;                         /* the 8 bytes around the next byte (disk -> RAM) */
reg [24:0] chunk_address = 25'h1ffffff;           /* the image word address of the chunk */
reg        chunk_valid = 1'b0;
reg        high_byte_received = 1'b0;
reg [7:0]  high_byte_register = 8'd0;
reg [15:0] word_from_controller = 16'd0;
reg        word_waiting_for_image = 1'b0;
reg        data_mark_seen = 1'b0;                 /* format stream: a data address mark was the last thing seen */
reg        fill_byte_found = 1'b0;
reg [7:0]  fill_byte = DEFAULT_FILL_BYTE;

assign word_pending     = word_waiting_for_image;
assign high_byte_held   = high_byte_received;
assign high_byte        = high_byte_register;
assign image_idle       = (state == IMAGE_IDLE);
assign image_state      = state;
/* a floppy format stream carries its fill byte, a hard disk format writes zeros */
assign format_fill_byte = floppy_transfer ? fill_byte : 8'h00;

/* ---- where the next byte is in the image */
wire [27:0] byte_offset         = base_offset + {9'd0, byte_pointer};
wire [28:0] hard_disk_address   = {4'b0011, byte_offset[27:3]};
wire [28:0] floppy_address      = {13'b0011_1111_1111_1, byte_offset[18:3]};
wire [28:0] current_address     = floppy_transfer ? floppy_address : hard_disk_address;
wire        chunk_is_current    = chunk_valid && (chunk_address == current_address[24:0]);

/* ---- the byte the controller gets next: from the chunk, or from the ID field of a Read Address */
reg [7:0] id_field_byte;

always @* begin
    case (byte_pointer[2:0])
        3'd0:    id_field_byte = id_track;
        3'd1:    id_field_byte = id_side;
        3'd2:    id_field_byte = id_sector;
        3'd3:    id_field_byte = id_size_code;
        3'd4:    id_field_byte = id_crc[15:8];
        default: id_field_byte = id_crc[7:0];
    endcase
end

assign source_valid = read_address_transfer ? 1'b1 : chunk_is_current;
assign source_byte  = read_address_transfer ? id_field_byte : chunk[{byte_offset[2:0], 3'b000} +: 8];

/* ---- the format stream: after a data address mark the next byte is the fill byte of the track (two bytes of a word, first the high byte) */
reg        scan_mark_seen;
reg        scan_fill_found;
reg [7:0]  scan_fill_byte;
reg [7:0]  scanned_byte;
integer    byte_in_word;

always @* begin
    scan_mark_seen  = data_mark_seen;
    scan_fill_found = fill_byte_found;
    scan_fill_byte  = fill_byte;
    scanned_byte    = 8'h00;
    for (byte_in_word = 0; byte_in_word < 2; byte_in_word = byte_in_word + 1) begin
        scanned_byte = (byte_in_word == 0) ? word_from_controller[15:8] : word_from_controller[7:0];
        if (scan_mark_seen && !scan_fill_found) begin
            scan_fill_byte  = scanned_byte;
            scan_fill_found = 1'b1;
            scan_mark_seen  = 1'b0;
        end
        else if (scanned_byte == DATA_MARK_FB || scanned_byte == DATA_MARK_F8) begin
            scan_mark_seen = 1'b1;
        end
    end
end

always @(posedge clk) begin
    /* ---- the controller's byte stream */
    if (source_take) begin
        byte_pointer <= byte_pointer + 19'd1;
    end

    if (sink_valid) begin
        if (!high_byte_received) begin
            high_byte_register <= sink_byte;
            high_byte_received <= 1'b1;
        end
        else begin
            word_from_controller   <= {high_byte_register, sink_byte};
            high_byte_received     <= 1'b0;
            word_waiting_for_image <= 1'b1;
        end
    end

    /* ---- image access: fetch the 8 bytes around the next byte (disk -> RAM), store a received word (RAM -> disk) */
    case (state)
        IMAGE_IDLE: begin
            if (transfer_running) begin
                if (!ram_to_disk && !read_address_transfer && !chunk_is_current) begin
                    ddr_rd   <= 1'b1;
                    ddr_addr <= current_address;
                    state    <= IMAGE_READ_REQUEST;
                end
                else if (ram_to_disk && word_waiting_for_image) begin
                    if (format_transfer) begin
                        /* Write Track: parse the format stream, nothing goes to the image yet */
                        data_mark_seen         <= scan_mark_seen;
                        fill_byte_found        <= scan_fill_found;
                        fill_byte              <= scan_fill_byte;
                        byte_pointer           <= byte_pointer + 19'd2;
                        word_waiting_for_image <= 1'b0;
                    end
                    else begin
                        ddr_din      <= {4{word_from_controller[7:0], word_from_controller[15:8]}};
                        ddr_be       <= 8'b11 << {byte_offset[2:1], 1'b0};
                        ddr_addr     <= current_address;
                        chunk_valid  <= 1'b0;
                        state        <= IMAGE_WRITE;
                    end
                end
            end
        end

        IMAGE_READ_REQUEST: begin
            if (~ddr_busy) begin
                ddr_rd <= 1'b0;
                state  <= IMAGE_READ_WAIT;
            end
        end

        IMAGE_READ_WAIT: begin
            if (ddr_ready) begin
                chunk         <= ddr_dout;
                chunk_address <= ddr_addr[24:0];
                chunk_valid   <= 1'b1;
                state         <= IMAGE_IDLE;
            end
        end

        IMAGE_WRITE: begin
            ddr_we <= 1'b1;
            state  <= IMAGE_WRITE_WAIT;
        end

        IMAGE_WRITE_WAIT: begin
            if (~ddr_busy) begin
                ddr_we                 <= 1'b0;
                byte_pointer           <= byte_pointer + 19'd2;
                word_waiting_for_image <= 1'b0;
                state                  <= IMAGE_IDLE;
            end
        end

        default: state <= IMAGE_IDLE;
    endcase

    /* ---- a new transfer starts */
    if (transfer_start) begin
        byte_pointer           <= 19'd0;
        chunk_valid            <= 1'b0;
        high_byte_received     <= 1'b0;
        word_waiting_for_image <= 1'b0;
        data_mark_seen         <= 1'b0;
        fill_byte_found        <= 1'b0;
        fill_byte              <= DEFAULT_FILL_BYTE;
    end

    if (rst) begin
        state                  <= IMAGE_IDLE;
        ddr_rd                 <= 1'b0;
        ddr_we                 <= 1'b0;
        high_byte_received     <= 1'b0;
        word_waiting_for_image <= 1'b0;
    end
end

endmodule
