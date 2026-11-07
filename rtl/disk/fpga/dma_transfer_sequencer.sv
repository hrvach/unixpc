/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The transfer sequencer (FPGA only): one disk DMA transfer from the controller's command to its interrupt.

      IDLE        wait for a command that moves data. The controllers announce it with transfer_request in the clock of the command write; it is kept
                  in `request_pending`
                  for the clock in which the sequencer takes it (start).
      RUN         the controllers' strobes run (run = 1). The kernel reads and writes several sectors with one command and says how many words in the
                  DMA count register
                  (the controller's sector count register is not used); a Read Address moves the 6 bytes of the ID field. The transfer ends when the
                  count register has run out (terminal count), the 6 bytes are moved, or a memory fault happened, once the whole chain is quiet: no
                  bus request, no window, no memory cycle, nothing waiting for the image. For RAM -> disk the word that was fetched last must have
                  been taken by the controller too (word_waiting).
      END         the transfer is over: the controllers raise their interrupt (transfer_ending). A Write Track first fills the track in the image with
                  the fill byte of the stream.
      FILL_TRACK  write one 64 bit word of the track to the image
      FILL_WAIT   wait until the image store has taken it

    The two controllers are independent chips: a command that does not move data (restore, seek, force interrupt) is accepted by one controller while
    the other one transfers, and gives its interrupt; only the controller that owns the engine is busy, and a transfer command needs the engine free
    (a floppy seek that lands in the milliseconds of a hard disk transfer must not be lost, the driver would wait for its interrupt forever).
*/
module dma_transfer_sequencer
(
    input         clk,
    input         rst,

    /* commands of the controllers (in the clock of the write) */
    input         hd_transfer_request,
    input         hd_transfer_is_write,
    input         hd_transfer_is_format,
    input  [15:0] hd_transfer_total_bytes,
    input         fd_transfer_request,
    input         fd_transfer_is_write,
    input         fd_transfer_is_read_address,
    input         fd_transfer_is_write_track,
    input  [15:0] fd_transfer_total_bytes,

    /* where the data is in the image, at the moment the transfer starts */
    input  [27:0] hard_disk_sector_offset,
    input  [27:0] floppy_sector_offset,
    input  [7:0]  format_fill_byte,
    /* the track fill: image word address of the first word and number of words (the format of a floppy or hard disk track) */
    input  [28:0] fill_base_address,
    input  [15:0] fill_words,

    input         abort_transfer,            /* Force Interrupt of the floppy controller: stop the running transfer */

    /* the chain */
    input         chain_quiet,               /* no bus request, no window, no memory cycle, no image access, no word pending, no strobe in progress */
    input         total_bytes_done,          /* the 6 bytes of an ID field are moved */
    input         memory_fault,
    input         terminal_count,
    input         word_waiting,              /* RAM -> disk: the word buffer still holds bytes the controller has not taken */

    /* to the engine and the controllers */
    output [2:0]  state,                     /* 0 idle, 1 run, 2 end, 3 fill track, 4 fill wait (debug) */
    output reg    run = 1'b0,                /* the controllers' DMA strobes run */
    output reg    ram_to_disk = 1'b0,
    output reg    floppy_transfer = 1'b0,
    output reg    read_address_transfer = 1'b0,
    output reg    format_transfer = 1'b0,
    output reg [27:0] base_offset = 28'd0,   /* byte offset of the first byte of the transfer in the image */
    /* bytes the strobes move before the transfer ends by itself (6 for an ID field, the sector table of a format), 0 = until the DMA count ends it */
    output reg [15:0] total_bytes = 16'd0,
    output reg    fault = 1'b0,              /* a memory fault ended the transfer */
    output        transfer_start,            /* pulse: a transfer starts in this clock */
    output        transfer_running,          /* state RUN */
    output        transfer_ending,           /* pulse: the transfer is over in this clock (the controllers raise their interrupt) */
    output        engine_idle,               /* no transfer running or starting */
    output        busy_with_hard_disk,
    output        busy_with_floppy,
    output        start_ram_to_disk,         /* the pending command's direction and drive (debug) */
    output        start_floppy,
    output        filling,                   /* the track fill drives the image port */

    /* the image store, track fill part */
    input         ddr_busy,
    output reg        fill_we = 1'b0,
    output reg [28:0] fill_addr,
    output reg [63:0] fill_data,
    output reg [7:0]  fill_be
);

typedef enum logic [2:0] {
    IDLE       = 3'd0,
    RUN        = 3'd1,
    END        = 3'd2,
    FILL_TRACK = 3'd3,
    FILL_WAIT  = 3'd4
} sequencer_state_t;

sequencer_state_t sequencer_state = IDLE;
reg        request_pending = 1'b0;
reg        request_ram_to_disk = 1'b0;
reg        request_floppy = 1'b0;
reg        request_read_address = 1'b0;
reg        request_write_track = 1'b0;
reg [15:0] request_total_bytes = 16'd0;
reg        aborting = 1'b0;           /* a Force Interrupt stopped the transfer: wait for the chain to settle, then end it */
reg        fill_done = 1'b0;
reg [15:0] fill_index = 16'd0;

assign state             = sequencer_state;
assign transfer_start    = (sequencer_state == IDLE) && request_pending;
assign transfer_running  = (sequencer_state == RUN);
assign transfer_ending   = (sequencer_state == END) && !(format_transfer && !fill_done);
assign engine_idle       = (sequencer_state == IDLE) && !request_pending;
assign start_ram_to_disk = request_ram_to_disk;
assign start_floppy      = request_floppy;
assign filling           = (sequencer_state == FILL_TRACK) || (sequencer_state == FILL_WAIT);

/* which controller owns the engine: the running transfer's, or the one that is about to start */
wire owner_is_floppy = (sequencer_state != IDLE) ? floppy_transfer : request_floppy;

assign busy_with_hard_disk = !engine_idle && !owner_is_floppy;
assign busy_with_floppy    = !engine_idle && owner_is_floppy;

always @(posedge clk) begin
    /* ---- a memory fault ends the transfer */
    if (memory_fault) begin
        fault <= 1'b1;
    end

    case (sequencer_state)
        IDLE: begin
            if (request_pending) begin
                request_pending       <= 1'b0;
                ram_to_disk           <= request_ram_to_disk;
                floppy_transfer       <= request_floppy;
                fault                 <= 1'b0;
                base_offset           <= request_floppy ? floppy_sector_offset : hard_disk_sector_offset;
                total_bytes           <= request_total_bytes;
                read_address_transfer <= request_read_address;
                format_transfer       <= request_write_track;
                fill_done             <= 1'b0;
                request_read_address  <= 1'b0;
                request_write_track   <= 1'b0;
                run                   <= 1'b1;
                sequencer_state       <= RUN;
            end
        end

        RUN: begin
            if (abort_transfer && floppy_transfer) begin
                run      <= 1'b0;                    /* the strobes stop at once, what is in flight still completes */
                aborting <= 1'b1;
            end
            if (chain_quiet && (aborting || total_bytes_done || fault || (terminal_count && !(ram_to_disk && word_waiting)))) begin
                run             <= 1'b0;
                aborting        <= 1'b0;
                sequencer_state <= END;
            end
        end

        END: begin
            if (format_transfer && !fill_done) begin
                fill_index      <= 16'd0;
                sequencer_state <= FILL_TRACK;        /* fill the formatted track in the image first */
            end
            else begin
                format_transfer       <= 1'b0;
                read_address_transfer <= 1'b0;
                sequencer_state       <= IDLE;
            end
        end

        FILL_TRACK: begin
            if (fill_index == fill_words) begin
                fill_done       <= 1'b1;
                sequencer_state <= END;
            end
            else begin
                fill_addr       <= fill_base_address + {13'd0, fill_index};
                fill_data       <= {8{format_fill_byte}};
                fill_be         <= 8'hff;
                fill_we         <= 1'b1;
                sequencer_state <= FILL_WAIT;
            end
        end

        FILL_WAIT: begin
            if (~ddr_busy) begin
                fill_we         <= 1'b0;
                fill_index      <= fill_index + 16'd1;
                sequencer_state <= FILL_TRACK;
            end
        end

        default: sequencer_state <= IDLE;
    endcase

    /* ---- a command of one of the controllers (set after the clear above: a command is only accepted while the engine is idle) */
    if (hd_transfer_request || fd_transfer_request) begin
        request_pending     <= 1'b1;
        request_ram_to_disk <= hd_transfer_request ? hd_transfer_is_write : fd_transfer_is_write;
        request_floppy      <= fd_transfer_request;
        request_total_bytes <= hd_transfer_request ? hd_transfer_total_bytes : fd_transfer_total_bytes;
        if (fd_transfer_request) begin
            request_read_address <= fd_transfer_is_read_address;
            request_write_track  <= fd_transfer_is_write_track;
        end
        else begin
            request_write_track  <= hd_transfer_is_format;
        end
    end

    if (rst) begin
        sequencer_state       <= IDLE;
        aborting              <= 1'b0;
        run                   <= 1'b0;
        request_pending       <= 1'b0;
        request_read_address  <= 1'b0;
        request_write_track   <= 1'b0;
        read_address_transfer <= 1'b0;
        format_transfer       <= 1'b0;
        fill_done             <= 1'b0;
        fill_we               <= 1'b0;
    end
end

endmodule
