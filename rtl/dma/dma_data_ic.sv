/*
    Copyright (c) 2026 by Hrvoje Cavrak

    DMA data IC (custom gate array, IC 22H, sheet 10 of the logic board): the disk DMA word buffer, the DMA count register and the bus request.

    There is no schematic or equation list for this chip. The function is taken from the manual's theory text ("Executing Hard Disk DMA Read",
    "Terminal Count", "Under run / Overrun Errors", the count register table) and, for the details the text leaves open, from the same design in the
    Convergent MiniFrame (1984): the first byte of a word is the HIGH byte (even address), in both directions. Parity generation / checking 
    (UPARIN, LPARIN) is not modelled.

    The disk controllers move BYTES (8 bit disk data bus DD7:DD0), the memory moves WORDS, so the chip assembles / splits words:
      disk -> RAM (IDMARW = 0): TFER* pulse 1 = high byte, pulse 2 = low byte; after the second one the chip raises DKRQ* (disk bus request).
                                When the bus is granted (DKBG*) the word goes onto the data bus for the memory write cycle.
      RAM -> disk (IDMARW = 1): on the grant the chip reads a word from the data bus; TFER* pulse 1 sends the high byte, pulse 2 the low byte and
      raises
                                DKRQ* for the next word. DKRQ* is therefore asserted once per two TFER* pulses.
      The byte is taken (or given) at the END of the TFER* pulse: the disk PAL (disk_interface.sv) asserts TFER* for the length of the controller's
      strobe.

    Count register (460000, bits as in the manual's Table 2-20):
      D15 DMAEN   1 = DMA enabled.  Every 0 -> 1 change makes ONE bus request at once (the documented "false transfer"): for a disk -> RAM transfer
                  it writes a garbage word to the address in the counter, for RAM -> disk it reads the first word of the sector (a prefetch). Both
                  cycles count, so the kernel loads the count that includes them.
      D14 IDMARW  0 = DMA write (disk -> RAM), 1 = DMA read (RAM -> disk)
      D13:D0 DC   counter, +1 for every word moved (every bus grant). Terminal count (TC*) is DC = 3FFF: no more bus requests are made.
      On a read of the register D15 is the under/overrun error flag (the controller wanted or delivered a byte before the buffer was ready).
*/
module dma_data_ic
import unixpc_registers_pkg::*;
(
    input         clk,              /* 1PCK+, 10 MHz (in the FPGA the system clock with ce) */
    input         ce,               /* clock enable, one pulse per 1PCK+ period */
    input         nRST,

    /* ---- CPU: count register */
    input         nDCNTCS,          /* count register chip select (low); 460000 */
    input         cpu_write,        /* 1 = the CPU writes (R/W* low), 0 = reads */
    input  [15:0] cpu_data_in,
    output [15:0] cpu_data_out,     /* count register as read back (valid while nDCNTCS is low and reading) */

    /* ---- disk side */
    input         nTFER,            /* transfer request from the disk PAL, low for the length of the controller's byte strobe */
    input  [7:0]  dd_in,            /* disk data bus, from the controller (disk -> RAM) */
    output [7:0]  dd_out,           /* disk data bus, to the controller (RAM -> disk) */
    output        dd_drive,         /* the chip drives DD7:DD0 (RAM -> disk) */

    /* ---- memory side */
    input         nDKBG,            /* disk bus grant: the request is dropped when it comes */
    input         nDKBGA,           /* disk bus grant acknowledge: the window of the memory cycle (from the grant until the memory is done) */
    input  [15:0] bus_in,           /* system data bus (RAM -> disk: the word read from memory) */
    output [15:0] bus_out,          /* system data bus (disk -> RAM: the word to write) */
    output        bus_drive,        /* the chip drives the bus during a disk -> RAM grant */
    output        dma_read,         /* IDMARW as seen by the PAL: 1 = the RAM is being read */

    /* the word buffer is ready for the next byte of the controller (see below); the real chip has no such output, see the note at its assignment */
    output        byte_ok,
    /* RAM -> disk: the word buffer still holds bytes the controller has not taken (the transfer is not over at terminal count) */
    output        word_waiting,
    output        nDKRQ,            /* disk bus request */
    output        underrun_overrun
);

reg        dma_enable   = 1'b0;     /* D15 */
reg        direction    = 1'b0;     /* D14 */
reg [13:0] count        = 14'd0;    /* D13:D0 */
reg        error        = 1'b0;

wire terminal_count = (count == 14'h3fff);

assign dma_read     = direction;

/* the count register as the CPU reads it: bit 15 is the under / overrun error flag instead of DMAEN */
disk_dma_count_reg_t count_register_read;

always @* begin
    count_register_read.DMAEN  = error;
    count_register_read.IDMARW = direction;
    count_register_read.DC     = count;
end

assign cpu_data_out = count_register_read;
assign underrun_overrun = error;
assign word_waiting     = word_ready;

/*
    Back-pressure for a controller that cannot be late. On the real board the disk controllers deliver a byte every 1.6 us and the DMA gets the bus
    within that time, always; a late bus grant would be an under- or overrun error. In the FPGA the CPU core and the Ethernet board can hold the bus
    for longer, and the controller model would then overwrite or repeat words (found by random grant delays in tb/disk_dma/disk_ctrl_tb.sv). So the
    strobe model starts a byte only when the buffer is ready for it:
      second byte of a word        always (the first byte of the word was accepted)
      first byte, disk -> RAM      the previous word has left: no request pending, no memory cycle running
      first byte, RAM -> disk      the word for it has arrived from memory
*/
assign byte_ok = second_byte | (direction ? word_ready : ~(request | ~nDKBGA));

/* ---- TFER pulse end detection */
reg tfer_active_before = 1'b0;
wire tfer_active = ~nTFER;
wire tfer_ended  = tfer_active_before & ~tfer_active;

reg [7:0] dd_sampled = 8'd0;        /* the disk data bus during the strobe: the last value before it ends is the byte */

always @(posedge clk) if (ce) begin
    tfer_active_before <= tfer_active;
    if (tfer_active) dd_sampled <= dd_in;
end

/* ---- word buffer and byte phase */
reg [15:0] word            = 16'd0;
reg        second_byte     = 1'b0;   /* 0: next byte is the high byte */
reg        word_ready      = 1'b0;   /* RAM -> disk: the buffer holds a word the controller has not fully taken */

assign dd_out   = second_byte ? word[7:0] : word[15:8];
assign dd_drive = dma_enable & direction;

/* ---- bus request */
reg request = 1'b0;
assign nDKRQ = ~request;

reg  nDKBGA_before = 1'b1;
wire grant_ended   = nDKBGA & ~nDKBGA_before;           /* the memory cycle is over */

assign bus_drive = ~nDKBGA & ~direction;
assign bus_out   = word;

/* the CPU turns DMA on: one request */
wire enable_write      = ~nDCNTCS & cpu_write;
disk_dma_count_reg_t count_register_written;

assign count_register_written = cpu_data_in;

wire dma_turned_on     = enable_write & ~dma_enable & count_register_written.DMAEN;

always @(posedge clk) if (ce) begin
    nDKBGA_before <= nDKBGA;

    if (!nRST) begin
        dma_enable  <= 1'b0;
        direction   <= 1'b0;
        count       <= 14'd0;
        error       <= 1'b0;
        request     <= 1'b0;
        second_byte <= 1'b0;
        word_ready  <= 1'b0;
        word        <= 16'd0;
    end
    else begin
        /* ---- CPU writes the count register */
        if (enable_write) begin
            dma_enable <= count_register_written.DMAEN;
            direction  <= count_register_written.IDMARW;
            count      <= count_register_written.DC;
            error      <= 1'b0;

            if (!count_register_written.DMAEN)
                request <= 1'b0;

            if (dma_turned_on)
                second_byte <= 1'b0;
        end
        if (dma_turned_on) request <= 1'b1;

        /* ---- a request is dropped when the bus is granted */
        if (!nDKBG) request <= 1'b0;

        /* ---- memory cycle */
        if (!nDKBGA && direction) word <= bus_in;                    /* RAM -> disk: keep the last value seen during the cycle */
        if (grant_ended) begin
            count <= count + 14'd1;
            if (direction) begin
                word_ready  <= 1'b1;
                second_byte <= 1'b0;
            end
        end

        /* ---- bytes from / to the disk controller */
        if (tfer_ended && dma_enable) begin
            if (!direction) begin                                   /* disk -> RAM */
                if (!second_byte) begin
                    word[15:8]  <= dd_sampled;
                    second_byte <= 1'b1;
                end
                else begin
                    word[7:0]   <= dd_sampled;
                    second_byte <= 1'b0;
                    if (request || !nDKBGA) error <= 1'b1;           /* the last word has not left yet: overrun */
                    else if (!terminal_count) request <= 1'b1;
                end
            end
            else begin                                              /* RAM -> disk */
                if (!word_ready) error <= 1'b1;                     /* the controller wants a byte but the word has not arrived: underrun */
                if (!second_byte) begin
                    second_byte <= 1'b1;
                end
                else begin
                    second_byte <= 1'b0;
                    word_ready  <= 1'b0;
                    if (!terminal_count) request <= 1'b1;
                end
            end
        end
    end
end

endmodule
