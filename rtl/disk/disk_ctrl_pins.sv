/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Disk subsystem with the DMA path at the pin level (the board's disk subsystem; the previous word mover is in archived/).

    The two controllers (WD1010 hard disk, WD2797 floppy) are behavioural; the way their data reaches memory is the chain of the real board:
      controller strobes -> disk PAL -> DMA data IC (count register) -> bus request / grant -> DMA address IC -> one memory cycle per word
    and the way the disk images are reached is FPGA plumbing: bytes of the controllers <-> 64 bit words of the image store (the MiSTer virtual drives,
    sd_image_bridge.sv).

      disk_control_register      4E0000    drive select and controller resets
      wd1010_controller          E0000x    hard disk task file and commands
      wd2797_controller          E1000x    floppy registers and commands
      disk_dma_path                        controller strobes, disk PAL, DMA data IC, DMA address IC (the chips of the board)
      dma_memory_cycle                     the memory cycle of one DMA word on the board's memory port
      dma_transfer_sequencer               one transfer from the command to the interrupt
      disk_media_access                    the bytes of the controllers <-> the image store
      disk_image_geometry                  sector -> byte offset in the image
      disk_dma_debug                       overlay, trace (observes only)

    CPU register map handled here (effective address):
      460000          R/W  DMA count
      4D00FF / 4D7FFF W    DMA address
      4E0000          W    disk control
      E0000x               WD1010
      E1000x               WD2797
*/
module disk_ctrl_pins
#(
    parameter DMA_TRACE = 0         /* 1 builds disk_dma_debug (tb/sim/models), which the simulator uses to check every DMA transfer; the core has none */
)
(
    input             clk,
    input             rst,

    /* configuration */
    input      [1:0]  hd_geom,       /* 0: 8 heads x 16 spt, 1: 17 spt (136/cyl), 2: 17 spt (272/cyl) */
    input             fd_sect8,      /* 1: 8 sectors/track floppy, 0: 10 */
    input             fd_present,    /* a floppy is in the drive (0: commands report "not ready") */
    input             fd_write_protected, /* the floppy image is read-only (write commands end with the write protect status) */

    /* CPU register access */
    input      [23:1] a,             /* effective address */
    input             wr_go,
    input             rd_go,
    input      [15:0] wdata,
    output reg [15:0] rdata,
    output            hit,

    /* interrupts to the interrupt controller / the line printer status register */
    output            fdint,
    output            hdint,

    /* DMA bus handshake */
    output            br,            /* DKRQ*: bus request */
    input             bg,            /* disk bus grant, gated with "the CPU has let go of the bus" */
    output            dma_active,    /* the window of one DMA cycle (BGACK to the board) */

    /* DMA memory port (logical word address) */
    output            m_req,
    output            m_we,
    output     [23:1] m_addr,
    output     [15:0] m_wdata,
    input             m_done,
    input             m_fault,
    input      [15:0] m_rdata,

    /* disk image storage (the 64 bit word port of sd_image_bridge.sv) */
    output            ddr_rd,
    output            ddr_we,
    output     [28:0] ddr_addr,
    output     [63:0] ddr_din,
    output     [7:0]  ddr_be,
    input      [63:0] ddr_dout,
    input             ddr_ready,
    input             ddr_busy,

    /* simulation self-check (tb/sim/models/disk_dma_debug.sv; zero unless DMA_TRACE = 1) */
    output            dbg_tog,       /* toggles at the end of every DMA transfer */
    output     [27:0] dbg_off,
    output     [15:0] dbg_n,
    output     [23:0] dbg_addr,
    output     [31:0] dbg_hash,
    output     [2:0]  dbg_flags,     /* {fault, floppy, ram->disk} */
    /* the last 8 CPU writes to the DMA count / address registers and to the controllers' command registers, newest first */
    output [127:0]    dbg_log
);

/* ------------------------------------------------------------------ address decode */
wire [7:0] address_high = a[23:16];
wire sel_cnt  = (address_high == 8'h46);       /* DMA count register */
wire sel_dadr = (address_high == 8'h4d);       /* DMA address register */
wire sel_dctl = (address_high == 8'h4e);       /* disk control register */
wire sel_hd   = (address_high == 8'he0);       /* WD1010 */
wire sel_fd   = (address_high == 8'he1);       /* WD2797 */

assign hit = sel_cnt | sel_hd | sel_fd;

/* ------------------------------------------------------------------ the sequencer and the signals everyone shares */
wire [2:0]  sequencer_state;
wire        run, ram_to_disk, floppy_transfer, read_address_transfer, format_transfer;
wire [27:0] base_offset;
wire        fault;
wire        transfer_start, transfer_running, transfer_ending;
wire        engine_idle, busy_with_hard_disk, busy_with_floppy;
wire        start_ram_to_disk, start_floppy;
wire        filling;
wire        fill_we;
wire [28:0] fill_addr;
wire [63:0] fill_data;
wire [7:0]  fill_be;

/* ------------------------------------------------------------------ the disk control register and the two controller chips */
wire        dctl_write = wr_go && sel_dctl;
wire        floppy_controller_enabled, hard_disk_controller_enabled;
wire [2:0]  selected_head;
wire        hd_interrupt, fd_interrupt;
wire        hd_transfer_request, hd_transfer_is_write, hd_transfer_is_format, hd_command_ignored;
wire [15:0] hd_transfer_total_bytes, fd_transfer_total_bytes, transfer_total_bytes;
wire        fd_transfer_request, fd_transfer_is_write, fd_transfer_is_read_address, fd_transfer_is_write_track;
wire [15:0] hd_read_data, fd_read_data;
wire [10:0] hd_cylinder;
wire [2:0]  hd_head;
wire [7:0]  hd_sector_number;
wire [7:0]  fd_track, fd_sector;
wire        fd_side;
wire        fd_data_request, fd_abort_transfer;
wire [7:0]  id_track, id_side, id_sector, id_size_code;
wire [15:0] id_crc;
wire [3:0]  last_hard_disk_command;

assign fdint = fd_interrupt;
assign hdint = hd_interrupt;

disk_control_register disk_control
(
    .clk                          (clk),
    .rst                          (rst),
    .write                        (dctl_write),
    .write_data                   (wdata[7:0]),
    .floppy_controller_enabled    (floppy_controller_enabled),
    .hard_disk_controller_enabled (hard_disk_controller_enabled),
    .head_select                  (selected_head)
);

wd1010_controller hard_disk_controller
(
    .clk                  (clk),
    .rst                  (rst),
    .controller_enabled   (hard_disk_controller_enabled),
    .head_select_load     (dctl_write),
    .head_select_data     (wdata[2:0]),
    .selected_head        (selected_head),
    .sectors_per_track    (hard_disk_sectors_per_track),
    .selected             (sel_hd),
    .register_select      (a[3:1]),
    .write                (wr_go),
    .read                 (rd_go),
    .write_data           (wdata[7:0]),
    .read_data            (hd_read_data),
    .engine_idle          (engine_idle),
    .busy_with_hard_disk  (busy_with_hard_disk),
    .transfer_done        (transfer_ending && !floppy_transfer),
    .transfer_request     (hd_transfer_request),
    .transfer_is_write    (hd_transfer_is_write),
    .transfer_is_format   (hd_transfer_is_format),
    .transfer_total_bytes (hd_transfer_total_bytes),
    .cylinder             (hd_cylinder),
    .head                 (hd_head),
    .sector_number        (hd_sector_number),
    .interrupt            (hd_interrupt),
    .command_ignored      (hd_command_ignored),
    .last_command_code    (last_hard_disk_command)
);

wd2797_controller floppy_controller
(
    .clk                      (clk),
    .rst                      (rst),
    .controller_enabled       (floppy_controller_enabled),
    .disk_present             (fd_present),
    .write_protected          (fd_write_protected),
    .data_request             (fd_data_request),
    .abort_transfer           (fd_abort_transfer),
    .eight_sector_tracks      (fd_sect8),
    .selected                 (sel_fd),
    .register_select          (a[2:1]),
    .write                    (wr_go),
    .read                     (rd_go),
    .write_data               (wdata[7:0]),
    .read_data                (fd_read_data),
    .engine_idle              (engine_idle),
    .busy_with_floppy         (busy_with_floppy),
    .transfer_done            (transfer_ending && floppy_transfer),
    .transfer_request         (fd_transfer_request),
    .transfer_is_write        (fd_transfer_is_write),
    .transfer_is_read_address (fd_transfer_is_read_address),
    .transfer_is_write_track  (fd_transfer_is_write_track),
    .transfer_total_bytes     (fd_transfer_total_bytes),
    .physical_track           (fd_track),
    .side                     (fd_side),
    .sector_register          (fd_sector),
    .id_track                 (id_track),
    .id_side                  (id_side),
    .id_sector                (id_sector),
    .id_size_code             (id_size_code),
    .id_crc                   (id_crc),
    .interrupt                (fd_interrupt)
);

/* ------------------------------------------------------------------ where the sectors are in the images */
wire [27:0] hard_disk_sector_offset, floppy_sector_offset;
wire [18:0] floppy_track_first_sector;
wire [15:0] floppy_track_words;
wire [4:0]  hard_disk_sectors_per_track;
wire [28:0] hard_disk_track_fill_base, floppy_track_fill_base;
wire [15:0] hard_disk_track_words;

disk_image_geometry geometry
(
    .hard_disk_geometry        (hd_geom),
    .eight_sector_floppy       (fd_sect8),
    .cylinder                  (hd_cylinder),
    .head                      (hd_head),
    .sector_number             (hd_sector_number),
    .floppy_track              (fd_track),
    .floppy_side               (fd_side),
    .floppy_sector             (fd_sector),
    .hard_disk_sector_offset   (hard_disk_sector_offset),
    .floppy_sector_offset      (floppy_sector_offset),
    .floppy_track_first_sector (floppy_track_first_sector),
    .floppy_track_words        (floppy_track_words),
    .hard_disk_sectors_per_track (hard_disk_sectors_per_track),
    .hard_disk_track_fill_base (hard_disk_track_fill_base),
    .hard_disk_track_words     (hard_disk_track_words),
    .floppy_track_fill_base    (floppy_track_fill_base)
);

/* ------------------------------------------------------------------ the DMA path of the board */
wire        bus_request, window_open;
wire [21:1] dma_address;
wire        dma_read;
wire [15:0] dma_write_word;
wire [15:0] count_register;
wire        underrun_overrun, word_waiting;
wire        source_valid, source_take, sink_valid, strobes_busy, total_bytes_done;
wire [7:0]  source_byte, sink_byte;
wire        word_pending;
wire        memory_timing_idle;
wire [15:0] memory_read_data;
wire        terminal_count = (count_register[13:0] == 14'h3fff);       /* the count register runs out at 3FFF: no more bus requests */

disk_dma_path dma_path
(
    .clk                     (clk),
    .rst                     (rst),
    .write_count_register    (wr_go && sel_cnt),
    .write_address_register  (wr_go && sel_dadr),
    .cpu_write_data          (wdata),
    .cpu_write_address       (a[21:1]),
    .bus_grant               (bg),
    .memory_timing_idle      (memory_timing_idle),
    .memory_read_data        (memory_read_data),
    .bus_request             (bus_request),
    .window_open             (window_open),
    .dma_address             (dma_address),
    .dma_read                (dma_read),
    .dma_write_word          (dma_write_word),
    .count_register          (count_register),
    .underrun_overrun        (underrun_overrun),
    .word_waiting            (word_waiting),
    .run                     (run),
    .floppy_transfer         (floppy_transfer),
    .ram_to_disk             (ram_to_disk),
    /* a Read Address moves the 6 bytes of the ID field, a hard disk format its sector table, everything else runs until the count ends */
    .total_bytes             (transfer_total_bytes),
    .media_wait              (ram_to_disk && word_pending),
    .source_valid            (source_valid),
    .source_byte             (source_byte),
    .source_take             (source_take),
    .sink_valid              (sink_valid),
    .sink_byte               (sink_byte),
    .strobes_busy            (strobes_busy),
    .floppy_data_request     (fd_data_request),
    .total_bytes_done        (total_bytes_done)
);

assign br         = bus_request;
assign dma_active = window_open;                        /* the bus is owned from the grant until the memory cycle is over */
assign m_addr     = {2'b00, dma_address};

/* ------------------------------------------------------------------ the memory cycle of one DMA word */
wire        window_served;
wire        memory_cycle_starting, memory_fault;
wire [1:0]  memory_state;

dma_memory_cycle memory_cycle
(
    .clk                   (clk),
    .rst                   (rst),
    .window_open           (window_open),
    .dma_read              (dma_read),
    .write_data            (dma_write_word),
    .m_req                 (m_req),
    .m_we                  (m_we),
    .m_wdata               (m_wdata),
    .m_done                (m_done),
    .m_fault               (m_fault),
    .m_rdata               (m_rdata),
    .read_data             (memory_read_data),
    .memory_timing_idle    (memory_timing_idle),
    .window_already_served (window_served),
    .cycle_starting        (memory_cycle_starting),
    .memory_fault          (memory_fault),
    .state_code            (memory_state)
);

/* ------------------------------------------------------------------ the media side */
wire        high_byte_held;
wire [7:0]  high_byte;
wire        image_idle;
wire [2:0]  image_state;
wire [7:0]  format_fill_byte;
wire        media_ddr_rd, media_ddr_we;
wire [28:0] media_ddr_addr;
wire [63:0] media_ddr_din;
wire [7:0]  media_ddr_be;

disk_media_access media
(
    .clk                    (clk),
    .rst                    (rst),
    .transfer_start         (transfer_start),
    .transfer_running       (transfer_running),
    .ram_to_disk            (ram_to_disk),
    .floppy_transfer        (floppy_transfer),
    .read_address_transfer  (read_address_transfer),
    .format_transfer        (format_transfer),
    .base_offset            (base_offset),
    .id_track               (id_track),
    .id_side                (id_side),
    .id_sector              (id_sector),
    .id_size_code           (id_size_code),
    .id_crc                 (id_crc),
    .source_take            (source_take),
    .source_valid           (source_valid),
    .source_byte            (source_byte),
    .sink_valid             (sink_valid),
    .sink_byte              (sink_byte),
    .word_pending           (word_pending),
    .high_byte_held         (high_byte_held),
    .high_byte              (high_byte),
    .image_idle             (image_idle),
    .image_state            (image_state),
    .format_fill_byte       (format_fill_byte),
    .ddr_rd                 (media_ddr_rd),
    .ddr_we                 (media_ddr_we),
    .ddr_addr               (media_ddr_addr),
    .ddr_din                (media_ddr_din),
    .ddr_be                 (media_ddr_be),
    .ddr_dout               (ddr_dout),
    .ddr_ready              (ddr_ready),
    .ddr_busy               (ddr_busy)
);

/* the image port is shared by the media access and the track fill of the sequencer (never at the same time) */
assign ddr_rd   = media_ddr_rd;
assign ddr_we   = media_ddr_we | fill_we;
assign ddr_addr = filling ? fill_addr : media_ddr_addr;
assign ddr_din  = filling ? fill_data : media_ddr_din;
assign ddr_be   = filling ? fill_be   : media_ddr_be;

/*
    ---- the transfer sequencer
    Nothing in flight: no request, no window, no memory cycle, nothing waiting for the image, no strobe in progress.
*/
wire chain_quiet = !bus_request && !window_open && memory_timing_idle && !window_served && image_idle && !word_pending && !strobes_busy;

dma_transfer_sequencer sequencer
(
    .clk                         (clk),
    .rst                         (rst),
    .hd_transfer_request         (hd_transfer_request),
    .hd_transfer_is_write        (hd_transfer_is_write),
    .hd_transfer_is_format       (hd_transfer_is_format),
    .hd_transfer_total_bytes     (hd_transfer_total_bytes),
    .fd_transfer_request         (fd_transfer_request),
    .fd_transfer_is_write        (fd_transfer_is_write),
    .fd_transfer_is_read_address (fd_transfer_is_read_address),
    .fd_transfer_is_write_track  (fd_transfer_is_write_track),
    .fd_transfer_total_bytes     (fd_transfer_total_bytes),
    .hard_disk_sector_offset     (hard_disk_sector_offset),
    .floppy_sector_offset        (floppy_sector_offset),
    .format_fill_byte            (format_fill_byte),
    .fill_base_address           (floppy_transfer ? floppy_track_fill_base : hard_disk_track_fill_base),
    .fill_words                  (floppy_transfer ? floppy_track_words : hard_disk_track_words),
    .abort_transfer              (fd_abort_transfer),
    .chain_quiet                 (chain_quiet),
    .total_bytes_done            (total_bytes_done),
    .memory_fault                (memory_fault),
    .terminal_count              (terminal_count),
    .word_waiting                (word_waiting),
    .state                       (sequencer_state),
    .run                         (run),
    .ram_to_disk                 (ram_to_disk),
    .floppy_transfer             (floppy_transfer),
    .read_address_transfer       (read_address_transfer),
    .format_transfer             (format_transfer),
    .base_offset                 (base_offset),
    .total_bytes                 (transfer_total_bytes),
    .fault                       (fault),
    .transfer_start              (transfer_start),
    .transfer_running            (transfer_running),
    .transfer_ending             (transfer_ending),
    .engine_idle                 (engine_idle),
    .busy_with_hard_disk         (busy_with_hard_disk),
    .busy_with_floppy            (busy_with_floppy),
    .start_ram_to_disk           (start_ram_to_disk),
    .start_floppy                (start_floppy),
    .filling                     (filling),
    .ddr_busy                    (ddr_busy),
    .fill_we                     (fill_we),
    .fill_addr                   (fill_addr),
    .fill_data                   (fill_data),
    .fill_be                     (fill_be)
);

/* ------------------------------------------------------------------ simulation trace (tb/sim/models/disk_dma_debug.sv, DMA_TRACE = 1) */
generate
    if (DMA_TRACE) begin : g_trace
    disk_dma_debug debug
    (
        .clk                        (clk),
        .wr_go                      (wr_go),
        .rd_go                      (rd_go),
        .a                          (a),
        .wdata                      (wdata),
        .sel_count_register         (sel_cnt),
        .sel_address_register       (sel_dadr),
        .sel_hard_disk              (sel_hd),
        .sel_floppy                 (sel_fd),
        .run                        (run),
        .engine_state               (sequencer_state),
        .image_state                (image_state),
        .memory_state               (memory_state),
        .bus_request                (bus_request),
        .window_open                (window_open),
        .word_pending               (word_pending),
        .ram_to_disk                (ram_to_disk),
        .floppy_transfer            (floppy_transfer),
        .fault                      (fault),
        .terminal_count             (terminal_count),
        .count_register             (count_register),
        .dma_error                  (underrun_overrun),
        .transfer_starting          (transfer_start),
        .transfer_ending            (transfer_ending),
        .transfer_offset            (base_offset),
        .start_ram_to_disk          (start_ram_to_disk),
        .start_floppy               (start_floppy),
        .hard_disk_interrupt        (hd_interrupt),
        .floppy_interrupt           (fd_interrupt),
        .hard_disk_command_ignored  (hd_command_ignored),
        .last_hard_disk_command     (last_hard_disk_command),
        .memory_cycle_starting      (memory_cycle_starting),
        .dma_read                   (dma_read),
        .dma_write_word             (dma_write_word),
        .memory_address             (m_addr),
        .memory_done                (m_done),
        .memory_fault               (m_fault),
        .controller_word_valid      (sink_valid && high_byte_held),
        .controller_word            ({high_byte, sink_byte}),
        .dbg_tog                    (dbg_tog),
        .dbg_off                    (dbg_off),
        .dbg_n                      (dbg_n),
        .dbg_addr                   (dbg_addr),
        .dbg_hash                   (dbg_hash),
        .dbg_flags                  (dbg_flags),
        .dbg_log                    (dbg_log)
    );
    end else begin : g_no_trace
        assign dbg_tog   = 1'b0;
        assign dbg_off   = 28'd0;
        assign dbg_n     = 16'd0;
        assign dbg_addr  = 24'd0;
        assign dbg_hash  = 32'd0;
        assign dbg_flags = 3'd0;
        assign dbg_log   = 128'd0;
    end
endgenerate

/* ------------------------------------------------------------------ what the CPU reads */
always @* begin
    rdata = 16'hffff;
    if (sel_cnt) begin
        /* D15 = U/OERR- (0: a disk DMA underrun or overrun happened, manual Table 2-20), D13:D0 the count */
        rdata = {~count_register[15], 1'b1, count_register[13:0]};
    end
    else if (sel_hd) begin
        rdata = hd_read_data;
    end
    else if (sel_fd) begin
        rdata = fd_read_data;
    end
end

endmodule
