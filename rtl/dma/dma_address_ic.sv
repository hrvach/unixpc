/*
    Copyright (c) 2026 by Hrvoje Cavrak

    DMA address IC (custom gate array, IC 22E, sheet 9 of the logic board; drawing D-32-00379-00 "DMA ADDRESS I.C." sheets 1 and 2 of 2).

    Written as clocked equations (the way a PAL would be), not as a copy of the gate array netlist: the timing of the board is taken from the manual's
    text, the structure from the drawing.

    What the chip does:
      1. Disk DMA address counter (21 bit word address, three 74LS461 counters on sheet 1).
           - loaded by the CPU with DADDWR* (decoded outside, address 4D00FF / 4D7FFF): the data comes from the ADDRESS bus, not the data bus:
             A14 = 0: counter bits 8:1 <= A8:A1 (low half);  A14 = 1: counter bits 21:9 <= A13:A1 (high half)
           - goes onto the address bus for the whole disk DMA cycle (DKBGA*: from the grant until the memory timing sequence is over, about 500 ns)
             and counts up by one word when the cycle ends.
      2. Bus grant acknowledge BGACK* for every memory cycle:
           - disk DMA cycle: asserted one clock after DKBG*, held 200 ns;   refresh cycle: asserted with RFBG*, 200 ns
           - CPU memory cycle: ENRAS* (memory timing started) makes BGACK* for 200 ns
           - CPU I/O cycle: I/ORQ+ makes BGACK* for 900 ns and I/O DTACK+ after 650 ns (the slow cycle)
      3. Refresh: an 8 bit counter on 1PCK+ requests a refresh (RFRQ*) every 144 clocks (14.4 us) until RFBG* comes; the refresh row counter drives
        A10:A3
         during the refresh grant (RFADDREN*).
      4. 1 MHz clock 1MCK+ (1PCK+ divided by 10) for the keyboard / real time clock.

    Not modelled: the exact flip-flop chain (a chain of 74LS174 on sheet 2 that makes the delays); the delays are counted instead.
*/
module dma_address_ic
(
    input         clk,              /* 1PCK+, 10 MHz (the board's rising edge); in the FPGA the faster system clock with ce */
    input         ce,               /* clock enable: one pulse per 1PCK+ period (1 when clk is the 10 MHz clock itself) */
    input         nRST,             /* VRST */

    /* ---- CPU side: loading the address counter */
    input         nDADDWR,          /* DMA address counter load strobe (low) */
    input  [21:1] address_in,       /* address bus as seen by the chip (the load data) */

    /* ---- bus grant / cycle control */
    input         nDKBG,            /* disk bus grant (from the arbiter PAL) */
    input         nRFBG,            /* refresh bus grant */
    input         nBGC,             /* bus grant common */
    input         iorq,             /* I/ORQ+: CPU I/O cycle */
    input         nENRAS,           /* memory timing sequence started */
    input         rw_read,          /* R/W* of the current cycle: 1 = read, 0 = write (forms FWR* with ENRAS*) */

    output reg    nBGACK,
    output reg    io_dtack,         /* I/O DTACK+ */
    output        nFWR,             /* fast write */
    /* disk bus grant acknowledge: low from the grant until the memory cycle is over (the window of a disk DMA cycle) */
    output        nDKBGA,
    output        nDKADDREN,        /* disk DMA address drivers enabled (the same window) */
    output [21:1] address_out,      /* the counter, valid while nDKADDREN is low */
    output        nRFADDREN,        /* refresh row drivers enabled */
    output [7:0]  refresh_row,      /* A10:A3 during refresh */
    output reg    nRFRQ,            /* refresh request */
    output reg    mck1              /* 1 MHz clock */
);

localparam REFRESH_PERIOD   = 144;   /* clocks between refresh requests (manual: every 144th clock, 14.4 us) */
localparam BGACK_CLOCKS     = 2;     /* 200 ns for memory cycles */
localparam IO_BGACK_CLOCKS  = 9;     /* 900 ns for I/O cycles */
localparam IO_DTACK_CLOCKS  = 6;     /* I/O DTACK+ 650 ns after I/ORQ+ (6.5 clocks, rounded as in the board's delay chain) */

/* ---- disk DMA address counter */
reg [21:1] counter = 21'd0;

wire loading   = ~nDADDWR;
wire load_high = address_in[14];

/* ---- disk DMA cycle window: from the grant until the memory timing sequence (ENRAS* low, then high again) is over */
reg disk_cycle      = 1'b0;
reg enras_was_low   = 1'b0;

always @(posedge clk) if (ce) begin
    if (!nRST) begin
        disk_cycle    <= 1'b0;
        enras_was_low <= 1'b0;
    end
    else begin
        if (!nDKBG && !disk_cycle) begin
            disk_cycle    <= 1'b1;
            enras_was_low <= 1'b0;
        end
        if (disk_cycle && !nENRAS) enras_was_low <= 1'b1;
        if (disk_cycle && enras_was_low && nENRAS) disk_cycle <= 1'b0;
    end
end

reg disk_cycle_before = 1'b0;

always @(posedge clk) if (ce) begin
    disk_cycle_before <= disk_cycle;
    if (!nRST) begin
        counter <= 21'd0;
    end
    else if (loading) begin
        if (load_high) counter[21:9] <= address_in[13:1];
        else           counter[8:1]  <= address_in[8:1];
    end
    else if (disk_cycle_before && !disk_cycle) begin    /* the cycle ended: the word has been moved */
        counter <= counter + 21'd1;
    end
end

assign nDKBGA      = ~disk_cycle;
assign nDKADDREN   = ~disk_cycle;
assign address_out = counter;

/* ---- BGACK*, I/O DTACK+ */
reg [3:0] bgack_left = 4'd0;
reg [3:0] dtack_left = 4'd0;

wire refresh_cycle = ~nBGC & ~nRFBG;

always @(posedge clk) if (ce) begin
    io_dtack <= (dtack_left == 4'd1);
    if (!nRST) begin
        nBGACK     <= 1'b1;
        bgack_left <= 4'd0;
        dtack_left <= 4'd0;
    end
    else if (bgack_left == 4'd0 && !nBGACK) begin       /* the hold time is over */
        nBGACK <= 1'b1;
    end
    else if (iorq && nBGACK) begin
        nBGACK     <= 1'b0;
        bgack_left <= IO_BGACK_CLOCKS - 1;
        dtack_left <= IO_DTACK_CLOCKS;
    end
    /* ENRAS* only for CPU cycles: a DMA cycle has been acknowledged by its grant already */
    else if ((!nDKBG || (!nENRAS && nBGC) || refresh_cycle) && nBGACK) begin
        nBGACK     <= 1'b0;
        bgack_left <= BGACK_CLOCKS - 1;
    end
    else begin
        if (bgack_left != 4'd0) bgack_left <= bgack_left - 4'd1;
        if (dtack_left != 4'd0) dtack_left <= dtack_left - 4'd1;
    end
end

assign nFWR = ~(~nENRAS & ~rw_read);

/* ---- refresh request and row counter */
reg [7:0] refresh_count = 8'd0;
reg [7:0] row = 8'd0;
reg nRFBG_before = 1'b1;

always @(posedge clk) if (ce) begin
    nRFBG_before <= nRFBG;
    if (!nRST) begin
        refresh_count <= 8'd0;
        nRFRQ         <= 1'b1;
        row           <= 8'd0;
    end
    else begin
        if (refresh_count == REFRESH_PERIOD - 1) begin
            refresh_count <= 8'd0;
            nRFRQ         <= 1'b0;
        end
        else begin
            refresh_count <= refresh_count + 8'd1;
        end
        if (!nRFBG) nRFRQ <= 1'b1;                      /* the grant presets the request flip-flop */
        if (nRFBG && !nRFBG_before) row <= row + 8'd1;  /* next row after every refresh cycle */
    end
end

assign nRFADDREN   = nRFBG;
assign refresh_row = row;

/* ---- 1 MHz clock */
reg [3:0] mck_count = 4'd0;
always @(posedge clk) if (ce) begin
    if (!nRST) begin
        mck_count <= 4'd0;
        mck1      <= 1'b0;
    end
    else if (mck_count == 4'd4) begin
        mck_count <= 4'd0;
        mck1      <= ~mck1;
    end
    else begin
        mck_count <= mck_count + 4'd1;
    end
end

endmodule
