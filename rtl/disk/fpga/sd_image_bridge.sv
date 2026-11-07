/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Disk image storage on the MiSTer virtual drives (hps_io sd_*), FPGA only. The 64 bit "DDR-style" port that the disk controllers use for their
    images is served from a one sector (512 byte) cache per drive; the sector comes from, and goes back to, the image file on the SD card through
    the HPS. Drive 0 is the hard disk, drive 1 the floppy.

    Address map seen on the port (64 bit word addresses):
      floppy      addr[28:16] == 13'b0011_1111_1111_1,                  byte offset = {addr[15:0], 3'b000}
      hard disk   addr[28:25] == 4'b0011 (and not the floppy region),   byte offset = {addr[24:0], 3'b000}
      anything else behaves like an empty drive.

    What happens on an access:
      - a read or write for a sector that is not in the cache first writes the old sector back if it is dirty, then loads the wanted one (sd_rd),
        then completes; reads return their data with ddr_ready, writes (byte enables as for a DDR bus) just complete;
      - a dirty sector is written back (sd_wr) when it is replaced, and after FLUSH_DELAY clocks without a further write to it, so a power cycle
        loses at most the last few tens of milliseconds of writes;
      - a drive without an image (never mounted, unmounted, or a sector beyond the end of the image) reads as zeros and ignores writes; a
        read-only image takes writes into the cache but never writes them back;
      - img_mounted for a drive drops the cache of that drive and takes the new size and read-only flag;
      - an HPS transfer that does not complete within WATCHDOG clocks is abandoned (the request completes as if the drive were empty), so a stuck
        HPS cannot hang the machine.
*/
module sd_image_bridge
#(
    parameter [19:0] FLUSH_DELAY = 20'd1000000,     /* about 50 ms at 20 MHz */
    parameter [25:0] WATCHDOG    = 26'd40000000     /* about 2 s at 20 MHz */
)
(
    input             clk,
    input             rst,

    /* the disk controllers' side (DDRAM-like) */
    input             ddr_rd,
    input             ddr_we,
    input      [28:0] ddr_addr,
    input      [63:0] ddr_din,
    input      [7:0]  ddr_be,
    output reg [63:0] ddr_dout,
    output reg        ddr_ready,
    output            ddr_busy,

    /* hps_io side: two virtual drives, 512 byte blocks, WIDE = 1 (16 bit buffer, low byte first) */
    output reg [31:0] sd_lba0,
    output reg [31:0] sd_lba1,
    output reg [1:0]  sd_rd,
    output reg [1:0]  sd_wr,
    input      [1:0]  sd_ack,
    input      [12:0] sd_buff_addr,
    input      [15:0] sd_buff_dout,
    input             sd_buff_wr,
    output     [15:0] sd_buff_din0,
    output     [15:0] sd_buff_din1,
    input      [1:0]  img_mounted,
    input      [63:0] img_size,
    input             img_readonly,

    output     [1:0]  read_only,        /* the image in drive 0 / 1 is mounted read-only (the floppy's write protect) */
    output     [1:0]  present,          /* an image is mounted in drive 0 / 1 */
    output            activity          /* an sd transfer or a write-back is pending (LED) */
);

localparam HARD_DISK = 1'b0;
localparam FLOPPY    = 1'b1;

/* ---- the two sector buffers: the HPS fills or empties them, the state machine below reads and writes them 16 bits at a time */
reg  [7:0]  buffer_address;
reg  [15:0] buffer_write_data;
reg  [1:0]  hard_disk_byte_write;
reg  [1:0]  floppy_byte_write;
wire [15:0] hard_disk_read_data;
wire [15:0] floppy_read_data;

wire [1:0]  hps_hard_disk_write = (sd_buff_wr & sd_ack[0]) ? 2'b11 : 2'b00;
wire [1:0]  hps_floppy_write    = (sd_buff_wr & sd_ack[1]) ? 2'b11 : 2'b00;

sd_sector_buffer hard_disk_buffer
(
    .clk               (clk),
    .hps_address       (sd_buff_addr[7:0]),
    .hps_write_data    (sd_buff_dout),
    .hps_byte_write    (hps_hard_disk_write),
    .hps_read_data     (sd_buff_din0),
    .bridge_address    (buffer_address),
    .bridge_write_data (buffer_write_data),
    .bridge_byte_write (hard_disk_byte_write),
    .bridge_read_data  (hard_disk_read_data)
);

sd_sector_buffer floppy_buffer
(
    .clk               (clk),
    .hps_address       (sd_buff_addr[7:0]),
    .hps_write_data    (sd_buff_dout),
    .hps_byte_write    (hps_floppy_write),
    .hps_read_data     (sd_buff_din1),
    .bridge_address    (buffer_address),
    .bridge_write_data (buffer_write_data),
    .bridge_byte_write (floppy_byte_write),
    .bridge_read_data  (floppy_read_data)
);

/* ---- state of the two drives (index 0: hard disk, 1: floppy) */
reg [1:0]  drive_mounted   = 2'b00;
reg [1:0]  drive_read_only = 2'b00;
reg [1:0]  cache_valid     = 2'b00;           /* the cache holds the sector named by cached_block */
reg [1:0]  cache_dirty     = 2'b00;           /* the cache holds writes the image file has not got yet */
reg [31:0] image_blocks     [0:1];            /* image size in 512 byte blocks */
reg [31:0] cached_block     [0:1];
reg [19:0] write_back_timer [0:1];            /* counts down to the idle write-back of a dirty sector */

initial begin
    image_blocks[0]     = 32'd0;
    image_blocks[1]     = 32'd0;
    cached_block[0]     = 32'd0;
    cached_block[1]     = 32'd0;
    write_back_timer[0] = 20'd0;
    write_back_timer[1] = 20'd0;
end

assign present   = drive_mounted;
assign read_only = drive_read_only;
assign activity  = (|sd_rd) | (|sd_wr) | (|cache_dirty);

/* ---- the state machine */
typedef enum logic [3:0] {
    IDLE                 = 4'd0,
    CHECK_CACHE          = 4'd1,
    START_LOAD           = 4'd2,   /* ask the HPS for the wanted sector */
    WAIT_LOAD_ACK        = 4'd3,
    WAIT_LOAD_DONE       = 4'd4,
    START_WRITE_BACK     = 4'd5,   /* give the dirty sector back to the HPS */
    WAIT_WRITE_BACK_ACK  = 4'd6,
    WAIT_WRITE_BACK_DONE = 4'd7,
    ACCESS_LANE          = 4'd8,   /* read: first clock of a 16 bit lane; write: decides to write */
    READ_LANE            = 4'd9,
    WRITE_LANE           = 4'd10,
    FINISH_EMPTY         = 4'd11   /* a request that no sector can serve */
} image_bridge_state_t;

image_bridge_state_t state = IDLE;

assign ddr_busy = (state != IDLE);

/* the request being served, copied from the port when it was accepted */
reg        request_is_write;
reg        request_drive;                         /* HARD_DISK or FLOPPY */
reg [31:0] request_block;
reg [5:0]  request_word;                          /* the 64 bit word in the sector */
reg [63:0] request_data;
reg [7:0]  request_byte_enables;
reg [1:0]  lane;                                  /* the 16 bit part of the 64 bit word */
reg        write_back_only;                       /* an idle write-back, no request is waiting */
reg [25:0] hps_wait;                              /* clocks since the HPS was asked, for the watchdog */

wire address_is_floppy   = (ddr_addr[28:16] == 13'b0011_1111_1111_1);
wire address_is_hard_disk = (ddr_addr[28:25] == 4'b0011) & ~address_is_floppy;

wire request_in_image    = drive_mounted[request_drive] && (request_block < image_blocks[request_drive]);
wire request_in_cache    = cache_valid[request_drive] && (cached_block[request_drive] == request_block);

wire [15:0] buffer_read_data = request_drive ? floppy_read_data : hard_disk_read_data;

/* the buffer port of the state machine: both buffers get the same address and data, only the selected drive's buffer is written */
always @* begin
    buffer_address       = {request_word, lane};
    buffer_write_data    = request_data[{lane, 4'b0000} +: 16];
    hard_disk_byte_write = 2'b00;
    floppy_byte_write    = 2'b00;

    if (state == WRITE_LANE) begin
        if (request_drive == FLOPPY)
            floppy_byte_write = request_byte_enables[{lane, 1'b0} +: 2];
        else
            hard_disk_byte_write = request_byte_enables[{lane, 1'b0} +: 2];
    end
end

/* when the transfer is over (or given up) the request is completed from the idle write-back or the cache load that was running */
function image_bridge_state_t state_after_write_back(input idle_write_back, input image_bridge_state_t otherwise);
    state_after_write_back = idle_write_back ? IDLE : otherwise;
endfunction

integer drive;

always @(posedge clk) begin
    ddr_ready <= 1'b0;
    hps_wait  <= hps_wait + 26'd1;

    for (drive = 0; drive < 2; drive = drive + 1) begin
        if (write_back_timer[drive] != 0 && cache_dirty[drive])
            write_back_timer[drive] <= write_back_timer[drive] - 20'd1;
    end

    /* an image is mounted or unmounted: take its size, drop the cache */
    for (drive = 0; drive < 2; drive = drive + 1) begin
        if (img_mounted[drive]) begin
            drive_mounted[drive]   <= (img_size != 64'd0);
            drive_read_only[drive] <= img_readonly;
            image_blocks[drive]    <= img_size[40:9];
            cache_valid[drive]     <= 1'b0;
            cache_dirty[drive]     <= 1'b0;
        end
    end

    if (rst) begin
        state           <= IDLE;
        sd_rd           <= 2'b00;
        sd_wr           <= 2'b00;
        write_back_only <= 1'b0;
    end
    else begin
        case (state)
            IDLE: begin
                hps_wait        <= 26'd0;
                lane            <= 2'd0;
                write_back_only <= 1'b0;

                if (ddr_rd | ddr_we) begin
                    request_is_write     <= ddr_we;
                    request_drive        <= address_is_floppy;
                    request_block        <= address_is_floppy ? {22'd0, ddr_addr[15:6]} : {13'd0, ddr_addr[24:6]};
                    request_word         <= ddr_addr[5:0];
                    request_data         <= ddr_din;
                    request_byte_enables <= ddr_be;

                    /* anything outside the two image regions behaves like an empty drive */
                    if (!address_is_floppy && !address_is_hard_disk) begin
                        request_drive <= HARD_DISK;
                        request_block <= 32'hffffffff;
                    end

                    state <= CHECK_CACHE;
                end
                else if (cache_dirty[0] && write_back_timer[0] == 0) begin
                    write_back_only <= 1'b1;
                    request_drive   <= HARD_DISK;
                    state           <= START_WRITE_BACK;
                end
                else if (cache_dirty[1] && write_back_timer[1] == 0) begin
                    write_back_only <= 1'b1;
                    request_drive   <= FLOPPY;
                    state           <= START_WRITE_BACK;
                end
            end

            CHECK_CACHE: begin
                if (!request_in_image)
                    state <= FINISH_EMPTY;                          /* empty drive: zeros, writes are ignored */
                else if (request_in_cache)
                    state <= ACCESS_LANE;
                else if (cache_dirty[request_drive])
                    state <= START_WRITE_BACK;
                else
                    state <= START_LOAD;
            end

            /* ---- write the cached sector back to the image file */
            START_WRITE_BACK: begin
                if (drive_read_only[request_drive]) begin           /* a read-only image: there is nothing to write back */
                    cache_dirty[request_drive] <= 1'b0;
                    state                      <= state_after_write_back(write_back_only, START_LOAD);
                end
                else begin
                    if (request_drive == FLOPPY) begin
                        sd_lba1  <= cached_block[1];
                        sd_wr[1] <= 1'b1;
                    end
                    else begin
                        sd_lba0  <= cached_block[0];
                        sd_wr[0] <= 1'b1;
                    end

                    hps_wait <= 26'd0;
                    state    <= WAIT_WRITE_BACK_ACK;
                end
            end

            WAIT_WRITE_BACK_ACK: begin
                if (sd_ack[request_drive]) begin
                    sd_wr <= 2'b00;
                    state <= WAIT_WRITE_BACK_DONE;
                end
                else if (hps_wait == WATCHDOG) begin
                    sd_wr                      <= 2'b00;
                    cache_dirty[request_drive] <= 1'b0;
                    state                      <= state_after_write_back(write_back_only, FINISH_EMPTY);
                end
            end

            WAIT_WRITE_BACK_DONE: begin
                if (!sd_ack[request_drive]) begin
                    cache_dirty[request_drive] <= 1'b0;
                    state                      <= state_after_write_back(write_back_only, START_LOAD);
                end
                else if (hps_wait == WATCHDOG) begin
                    cache_dirty[request_drive] <= 1'b0;
                    state                      <= state_after_write_back(write_back_only, FINISH_EMPTY);
                end
            end

            /* ---- load the wanted sector from the image file */
            START_LOAD: begin
                cache_valid[request_drive] <= 1'b0;

                if (request_drive == FLOPPY) begin
                    sd_lba1  <= request_block;
                    sd_rd[1] <= 1'b1;
                end
                else begin
                    sd_lba0  <= request_block;
                    sd_rd[0] <= 1'b1;
                end

                hps_wait <= 26'd0;
                state    <= WAIT_LOAD_ACK;
            end

            WAIT_LOAD_ACK: begin
                if (sd_ack[request_drive]) begin
                    sd_rd <= 2'b00;
                    state <= WAIT_LOAD_DONE;
                end
                else if (hps_wait == WATCHDOG) begin
                    sd_rd <= 2'b00;
                    state <= FINISH_EMPTY;
                end
            end

            WAIT_LOAD_DONE: begin
                if (!sd_ack[request_drive]) begin
                    cache_valid[request_drive]    <= 1'b1;
                    cached_block[request_drive]   <= request_block;
                    lane                          <= 2'd0;
                    state                         <= ACCESS_LANE;
                end
                else if (hps_wait == WATCHDOG) begin
                    state <= FINISH_EMPTY;
                end
            end

            /* ---- access the cached sector: a read takes two clocks per 16 bit lane, a write one */
            ACCESS_LANE: begin
                if (request_is_write) begin
                    /* the buffers' byte write enables are driven from WRITE_LANE only: write the four lanes one by one */
                    lane  <= 2'd0;
                    state <= WRITE_LANE;
                end
                else begin
                    state <= READ_LANE;
                end
            end

            READ_LANE: begin
                ddr_dout[{lane, 4'b0000} +: 16] <= buffer_read_data;
                lane                            <= lane + 2'd1;

                if (lane == 2'd3) begin
                    ddr_ready <= 1'b1;
                    state     <= IDLE;
                end
                else begin
                    state <= ACCESS_LANE;
                end
            end

            WRITE_LANE: begin
                lane <= lane + 2'd1;

                if (lane == 2'd3) begin
                    cache_dirty[request_drive]      <= 1'b1;
                    write_back_timer[request_drive] <= FLUSH_DELAY;
                    state                           <= IDLE;
                end
            end

            /* ---- a request that cannot be served: zeros for a read, a write is dropped */
            FINISH_EMPTY: begin
                if (!request_is_write) begin
                    ddr_dout  <= 64'd0;
                    ddr_ready <= 1'b1;
                end

                state <= IDLE;
            end

            default: begin
                state <= IDLE;
            end
        endcase
    end
end

endmodule
