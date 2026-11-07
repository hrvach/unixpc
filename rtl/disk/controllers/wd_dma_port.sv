/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The DMA side of the two disk controllers, seen from the disk bus interface: the strobes of the WD1010 hard disk controller (HDBCS*, HDRE*, HDWE*)
    and of the WD2797 floppy controller (DRQ, RE*, WE*) while a sector is moved, byte by byte, and the disk data bus DD7:DD0. FPGA model of the chips'
    pins; what happens behind the pins (commands, registers, the media) is in wd1010_controller.sv and wd2797_controller.sv.

    A transfer moves one byte per byte time (the data rate of the drive) and runs until `total_bytes` have gone (0 = until stopped).
      disk -> RAM (to_ram = 1): the next byte is taken from src_byte (src_valid says it is there); the controller puts it on DD and strobes it into
      the bus
                                interface: hard disk HDBCS* + HDWE* low, floppy: DRQ, then the disk PAL answers with RE* and the byte is read out.
      RAM -> disk (to_ram = 0): the controller asks for a byte (hard disk HDBCS* + HDRE* low, floppy DRQ then WE*) and latches the byte from DD at the
      end of the strobe;
                                it is passed on in snk_byte (snk_valid pulses).
    Timing from the data sheets: floppy RE* / WE* are at least 200 ns wide (the PAL makes 4 clocks of 100 ns), DRQ is reset 100 .. 200 ns after RE* /
    WE* falls (here one 1PCK+ period), read data is held for one period after RE*. The hard disk strobe is 4 periods (400 ns, the MiniFrame manual:
    "roughly 400 ns").
*/
module wd_dma_port
#(
    parameter HD_BYTE_PERIODS = 16,     /* 1PCK+ periods per byte: 1.6 us, the data rate of the hard disk */
    parameter FD_BYTE_PERIODS = 320     /* 32 us per byte: the double density floppy */
)
(
    input         clk,
    input         ce,                   /* one pulse per 1PCK+ period (10 MHz) */
    input         rst,

    /*
        RAM -> disk: the previous word is not stored on the media yet (the image store takes longer than the controller's word period): wait, the next
        byte would overwrite it
    */
    input         media_wait,
    /* the DMA chip can take / give a byte now (a late bus grant holds the controller back instead of losing a word) */
    input         byte_ok,
    input         run,                  /* a transfer is in progress */
    input         floppy,               /* 1: WD2797, 0: WD1010 */
    input         to_ram,               /* 1: disk -> RAM */
    input  [15:0] total_bytes,          /* bytes to move, 0 = no limit */

    input         src_valid,            /* disk -> RAM: the next byte of the media */
    input  [7:0]  src_byte,
    output        src_take,             /* pulse: the byte has been taken */
    output reg    snk_valid,            /* RAM -> disk: a byte has been written to the media (pulse) */
    output reg [7:0] snk_byte,
    output        busy,                 /* a byte strobe is in progress (the byte of a RAM -> disk strobe is delivered one period after it ends) */
    output reg    done,                 /* total_bytes moved (stays until run drops) */

    /* ---- pins towards the disk PAL / data IC */
    output reg    nHDBCS,
    output reg    nHDRE,
    output reg    nHDWE,
    output reg    fd_drq,
    input         nFDRE,
    input         nFDWE,
    input  [7:0]  dd_in,                /* disk data bus as driven by the bus interface (RAM -> disk) */
    output [7:0]  dd_out,               /* disk data bus as driven by the controller (disk -> RAM) */
    output        dd_drive
);

localparam HD_STROBE_PERIODS = 4;

typedef enum logic [2:0] {
    IDLE      = 3'd0,
    GAP       = 3'd1,
    HD_STROBE = 3'd2,
    FD_DRQ    = 3'd3,
    FD_STROBE = 3'd4,
    FINISH    = 3'd5
} dma_port_state_t;
reg [7:0]  dd_in_held = 8'd0;            /* the byte written to the floppy is the value on DD just before WE* ended */
dma_port_state_t state = IDLE;
reg [15:0] periods_left = 16'd0;
reg [15:0] moved = 16'd0;
reg [7:0]  data_register = 8'd0;

/*
    disk -> RAM: the controller keeps the byte on DD until it fetches the next one. (The real chips drive DD only during their strobe and for a short
    hold time; the data IC takes the byte at the end of TFER*, which comes one period after the strobe has ended, so this is the safe superset.)
*/
assign dd_out   = data_register;
assign busy     = (state == HD_STROBE) || (state == FD_DRQ) || (state == FD_STROBE) || snk_valid;
assign dd_drive = run & to_ram;

wire [15:0] byte_periods = floppy ? FD_BYTE_PERIODS : HD_BYTE_PERIODS;
wire        limit_reached = (total_bytes != 16'd0) && (moved == total_bytes);

reg take_pulse = 1'b0;
assign src_take = take_pulse;

always @(posedge clk) begin
    snk_valid  <= 1'b0;
    take_pulse <= 1'b0;
    if (rst || !run) begin
        state     <= IDLE;
        nHDBCS    <= 1'b1;
        nHDRE     <= 1'b1;
        nHDWE     <= 1'b1;
        fd_drq    <= 1'b0;
        moved     <= 16'd0;
        done      <= 1'b0;
    end
    else if (ce) begin
        if (state == FD_STROBE && !nFDWE) dd_in_held <= dd_in;
        case (state)
            IDLE: begin
                periods_left <= byte_periods;
                state        <= GAP;
            end

            /* the gap between two bytes (the drive turns at its own speed) */
            GAP: begin
                if (limit_reached) begin
                    done  <= 1'b1;
                    state <= FINISH;
                end
                else if (periods_left > 16'd1) begin
                    periods_left <= periods_left - 16'd1;
                end
                else if (to_ram && !src_valid) begin
                    /* the media has no byte yet: wait (an overrun for the real controller; here the source is always ahead) */
                end
                else if (!byte_ok || media_wait) begin
                    /*
                        the DMA chip is not ready for this byte (its word is still on the way, or the bus grant is late): wait; the real controller
                        would report an under- or overrun, a late grant cannot happen on the board but can with the FPGA's CPU core
                    */
                end
                else begin
                    if (to_ram) begin
                        data_register <= src_byte;
                        take_pulse    <= 1'b1;
                    end
                    periods_left <= HD_STROBE_PERIODS;
                    if (floppy) begin
                        fd_drq <= 1'b1;
                        state  <= FD_DRQ;
                    end
                    else begin
                        nHDBCS <= 1'b0;

                        if (to_ram)
                            nHDWE <= 1'b0;
                        else
                            nHDRE <= 1'b0;

                        state  <= HD_STROBE;
                    end
                end
            end

            /* hard disk: the strobe lasts HD_STROBE_PERIODS, the byte is latched at its end */
            HD_STROBE: begin
                if (periods_left > 16'd1) periods_left <= periods_left - 16'd1;
                else begin
                    if (!to_ram) begin
                        snk_byte  <= dd_in;
                        snk_valid <= 1'b1;
                    end
                    nHDBCS <= 1'b1;
                    nHDRE  <= 1'b1;
                    nHDWE  <= 1'b1;
                    moved  <= moved + 16'd1;
                    periods_left <= byte_periods;
                    state  <= GAP;
                end
            end

            /* floppy: DRQ is up; the disk PAL answers with RE* (disk -> RAM) or WE* (RAM -> disk) */
            FD_DRQ: begin
                if ((to_ram && !nFDRE) || (!to_ram && !nFDWE)) begin
                    fd_drq <= 1'b0;                              /* DRQ is reset one period after the strobe starts */
                    state  <= FD_STROBE;
                end
            end

            FD_STROBE: begin
                if ((to_ram && nFDRE) || (!to_ram && nFDWE)) begin     /* the strobe has ended */
                    if (!to_ram) begin
                        snk_byte  <= dd_in_held;
                        snk_valid <= 1'b1;
                    end
                    moved        <= moved + 16'd1;
                    periods_left <= byte_periods;
                    state        <= GAP;
                end
            end

            FINISH: ;                                            /* done is up until run drops */

            default: state <= IDLE;
        endcase
    end
end


endmodule
