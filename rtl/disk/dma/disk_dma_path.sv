/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The disk DMA path of the logic board: the chips between the two disk controllers and the memory bus.

      controller strobes (wd_dma_port.sv, a model of the WD1010 / WD2797 data side)
        -> disk PAL (disk_interface.sv, IC on sheet 11): turns the strobes into TFER* -> DMA data IC (dma_data_ic.sv, sheet 10): count register, word
        buffer, bus request -> arbiter (outside: unixpc_board.sv) -> bus grant -> DMA address IC (dma_address_ic.sv, sheet 1-2): address counter, the
        window of one DMA cycle

    Everything here runs on the 20 MHz system clock with a clock enable every other clock (ce): the chips of the board are clocked by 1PCK+, 10 MHz.
    The CPU writes of the count and address registers are held until the next clock enable, as the chips would latch them on their own clock edge.

    FPGA only, not on the board: the gate on TFER* (strobe_active, see below), the direction input of the PAL (derived from the running command),
    media_wait / byte_ok (a controller model that waits for the media and the DMA chip instead of overrunning them).
*/
module disk_dma_path
(
    input         clk,
    input         rst,

    /* CPU writes of the DMA registers (already decoded) */
    input         write_count_register,       /* 460000 */
    input         write_address_register,     /* 4D00FF / 4D7FFF */
    input  [15:0] cpu_write_data,
    input  [21:1] cpu_write_address,          /* the address register's data is on the address lines A21:A1 */

    /* the memory bus */
    input         bus_grant,                  /* disk bus grant (DKBG*), already gated with "the CPU has let go of the bus" */
    input         memory_timing_idle,         /* nENRAS: low while the memory cycle of the window runs */
    input  [15:0] memory_read_data,           /* the word the memory cycle read (RAM -> disk) */
    output        bus_request,                /* DKRQ*: the DMA data IC asks for the bus */
    output        window_open,                /* DKBGA*: the window of one DMA cycle (grant until the memory timing is over) */
    output [21:1] dma_address,                /* the address counter */
    output        dma_read,                   /* direction (count register D14): 1 = RAM -> disk */
    output [15:0] dma_write_word,             /* the word to write to RAM (disk -> RAM) */

    /* DMA count register (460000) as the data IC returns it */
    output [15:0] count_register,             /* {under/overrun error, direction, count[13:0]} */
    output        underrun_overrun,
    output        word_waiting,               /* RAM -> disk: the word buffer still holds bytes the controller has not taken */

    /* the controller side (the engine tells which transfer runs) */
    input         run,                        /* the controllers' DMA strobes run */
    input         floppy_transfer,            /* 1: WD2797, 0: WD1010 */
    input         ram_to_disk,
    input  [15:0] total_bytes,                /* bytes to move, 0 = until the count register ends it */
    input         media_wait,                 /* RAM -> disk: the previous word is not stored on the media yet */
    input         source_valid,               /* disk -> RAM: the next byte of the media is there */
    input  [7:0]  source_byte,
    output        source_take,                /* pulse: the byte has been taken */
    output        sink_valid,                 /* RAM -> disk: a byte for the media (pulse) */
    output [7:0]  sink_byte,
    output        floppy_data_request,        /* the floppy controller's DRQ pin */
    output        strobes_busy,               /* a byte strobe is in progress */
    output        total_bytes_done            /* total_bytes moved */
);

/* ------------------------------------------------------------------ clock enable (1PCK+, 10 MHz) */
reg ce = 1'b0;

always @(posedge clk) begin
    ce <= rst ? 1'b0 : ~ce;
end

/* ------------------------------------------------------------------ CPU writes, held until the chips' next clock enable */
reg         count_write_pending   = 1'b0;
reg         address_write_pending = 1'b0;
reg  [15:0] count_write_data      = 16'd0;
reg  [21:1] address_write_data    = 21'd0;

always @(posedge clk) begin
    if (write_count_register) begin
        count_write_pending <= 1'b1;
        count_write_data    <= cpu_write_data;
    end
    else if (ce) begin
        count_write_pending <= 1'b0;
    end

    if (write_address_register) begin
        address_write_pending <= 1'b1;
        address_write_data    <= cpu_write_address;
    end
    else if (ce) begin
        address_write_pending <= 1'b0;
    end
end

/* ------------------------------------------------------------------ the strobes of the controllers (model) and the disk data bus */
wire        nHDBCS, nHDRE, nHDWE, nFDRE, nFDWE;
wire        floppy_drq;
assign floppy_data_request = floppy_drq;
wire [7:0]  controller_data;                  /* DD7:DD0 driven by the controller (disk -> RAM) */
wire        controller_drives_data;
wire [7:0]  chip_data;                        /* DD7:DD0 driven by the DMA data IC (RAM -> disk) */
wire        chip_drives_data;
wire        byte_ok;                          /* the DMA data IC is ready for the next byte */

reg  [7:0]  disk_data_bus;

always @* begin
    if (chip_drives_data)
        disk_data_bus = chip_data;
    else if (controller_drives_data)
        disk_data_bus = controller_data;
    else
        disk_data_bus = 8'hff;               /* nobody drives the bus: pulled up */
end

wd_dma_port controller_strobes
(
    .clk         (clk),
    .ce          (ce),
    .rst         (rst),
    .media_wait  (media_wait),
    .byte_ok     (byte_ok),
    .run         (run),
    .floppy      (floppy_transfer),
    .to_ram      (~ram_to_disk),
    .total_bytes (total_bytes),
    .src_valid   (source_valid),
    .src_byte    (source_byte),
    .src_take    (source_take),
    .snk_valid   (sink_valid),
    .snk_byte    (sink_byte),
    .busy        (strobes_busy),
    .done        (total_bytes_done),
    .nHDBCS      (nHDBCS),
    .nHDRE       (nHDRE),
    .nHDWE       (nHDWE),
    .fd_drq      (floppy_drq),
    .nFDRE       (nFDRE),
    .nFDWE       (nFDWE),
    .dd_in       (disk_data_bus),
    .dd_out      (controller_data),
    .dd_drive    (controller_drives_data)
);

/*
    ---- the disk PAL
    The latch on the floppy DRQ (sheet 11) sits in front of the PAL.
*/
reg floppy_drq_latched = 1'b0;

always @(posedge clk) begin
    if (ce) begin
        floppy_drq_latched <= floppy_drq;
    end
end

/*
    The PAL's DMAR/W- input is the direction of the floppy transfer in progress (the command decides, not the drive select bit of the disk control
    register: the kernel runs hard disk commands with that bit clear); for the hard disk it is held high: with the equations as printed the term QD* x
    DMARW would keep TFER asserted in one direction and hide the hard disk strobes.
*/
wire pal_dmarw_input = floppy_transfer ? dma_read : 1'b1;
wire nTFER;

disk_interface disk_pal
(
    .PCK     (clk),
    .CE      (ce),
    .nRD     (1'b1),
    .nWR     (1'b1),
    .nDMARW  (pal_dmarw_input),
    .FDDRQL  (floppy_drq_latched),
    .nHDBCS  (nHDBCS),
    .nFDCS   (1'b1),
    .nHDRE   (nHDRE),
    .nHDWE   (nHDWE),
    .OE      (1'b0),
    .nFDRE   (nFDRE),
    .nFDWE   (nFDWE),
    .nFDTFER (),
    .nTFER   (nTFER),
    .debug   (),
    .nqb     (),
    .nqc     (),
    .nqd     ()
);

/*
    The PAL's TFER* output follows its DMAR/W input while nothing is strobed (TFER = DMARW when the PAL's QD flip-flop is low): a change of the
    direction, e.g. the first hard disk command after a floppy command, is a pulse on TFER* that the data IC would count as a byte (stale data; every
    byte after it shifted by one, found by the random operations of tb/disk_dma/disk_ctrl_tb.sv). On the board the data IC does not see such a
    pulse (DMAEN is set only for a transfer, the direction does not change under it); here the IC only sees TFER* while a controller strobe is on.
*/
wire strobe_active = ~nHDBCS | ~nFDRE | ~nFDWE;
wire nTFER_to_data_ic = nTFER | ~strobe_active;

/* ------------------------------------------------------------------ the DMA data IC and the DMA address IC */
wire nDKRQ, nDKBGA;
wire [21:1] address_counter;

dma_data_ic dma_data_chip
(
    .clk              (clk),
    .ce               (ce),
    .nRST             (~rst),
    .nDCNTCS          (~count_write_pending),
    .cpu_write        (1'b1),
    .cpu_data_in      (count_write_data),
    .cpu_data_out     (count_register),
    .nTFER            (nTFER_to_data_ic),
    .dd_in            (disk_data_bus),
    .dd_out           (chip_data),
    .dd_drive         (chip_drives_data),
    .nDKBG            (~bus_grant),
    .nDKBGA           (nDKBGA),
    .bus_in           (memory_read_data),
    .bus_out          (dma_write_word),
    .bus_drive        (),
    .dma_read         (dma_read),
    .byte_ok          (byte_ok),
    .word_waiting     (word_waiting),
    .nDKRQ            (nDKRQ),
    .underrun_overrun (underrun_overrun)
);

dma_address_ic dma_address_chip
(
    .clk          (clk),
    .ce           (ce),
    .nRST         (~rst),
    .nDADDWR      (~address_write_pending),
    .address_in   (address_write_data),
    .nDKBG        (~bus_grant),
    .nRFBG        (1'b1),
    .nBGC         (1'b1),
    .iorq         (1'b0),
    .nENRAS       (memory_timing_idle),
    .rw_read      (1'b1),
    .nBGACK       (),
    .io_dtack     (),
    .nFWR         (),
    .nDKBGA       (nDKBGA),
    .nDKADDREN    (),
    .address_out  (address_counter),
    .nRFADDREN    (),
    .refresh_row  (),
    .nRFRQ        (),
    .mck1         ()
);

assign bus_request = ~nDKRQ;
assign window_open = ~nDKBGA;
assign dma_address = address_counter;

endmodule
