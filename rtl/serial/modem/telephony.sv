/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Telephony / modem support registers of the logic board: everything around the 7201 that is not the 7201.

      address         dir  what
      450000          R    telephone status      D0 OFF HOOK*  D1 L1 RING*  D2 L2 RING*  D3 MSG WAIT*   (* = active low)
      490000-497000   W    line control latches: A14:A12 choose one, D14 is the value (handset relay, line select, hook relays, hold, A-lead)
      4B0000-4B03FF   W    baud rate latch 4L: N = A8:A1; the modem clock TMOUT = 1.2288 MHz / (4 N)
      4B0400, 4B0800  W    dialer load (838A): A11:A10 = 01 lower byte, 10 upper byte (the data is on address bits A8:A1); the upper byte starts the
                           16 bit stream. (The kernel's writedialer() writes 4B0400 + (low << 1), then 4B0800 + (high << 1).)
      E6xxxx          R/W  882A modem chip (modem_882a.sv); E68000 / E69000 W transceiver control and E6A000 R transceiver status are part of it

    The analog side is not modelled. The harness drives ring_l1 / ring_l2 (an incoming call: 20 Hz ring bursts on the status bits) and reads the line
    state (hook relays, dial words) to decide about the carrier.
*/
module telephony
import unixpc_registers_pkg::*;
#(
    parameter MESSAGE_LAMP_BITS = 23        /* the message waiting lamp input toggles every 2^23 clocks (a test uses a smaller number) */
)
(
    input         clk,
    input         rst,

    /* ---- CPU bus */
    input  [23:1] a,            /* CPU address bits 23:1 */
    input         wr_go,
    input         rd_go,
    input  [15:0] wdata,

    /* ---- the telephone line (harness) */
    input         ring_l1,      /* incoming call on line 1 / 2 (held by the harness for the duration of a ring burst) */
    input         ring_l2,
    input         ext_offhook,  /* another phone on the line is off hook */
    input         msg_wait,
    input         carrier,      /* the far end modem is connected: data carrier on the line */

    /* ---- read data */
    output [15:0] status,       /* 450000 */
    output [15:0] e6_rdata,     /* E6xxxx reads */

    /* ---- line control latches (490000 ...) */
    output        hdset_relay,  /* 1 = handset on line */
    output        line_sel2,    /* 0 = line 2 */
    output        hook1,        /* 1 = hook relay closed (line seized, off hook) */
    output        hook2,
    output        l1_hold,
    output        l2_hold,
    output        l1_alead,
    output        l2_alead,

    /* ---- baud latch and dialer */
    output [7:0]  baud_n,
    output        dial_tog,     /* toggles when a 16 bit dial word has been sent */
    output [15:0] dial_word,

    /* ---- 882A registers */
    output [15:0] mreg0,
    output [15:0] mreg1,
    output [15:0] mreg8,
    output [15:0] mreg3,
    output [15:0] mreg4,
    output [15:0] mreg5,
    output [15:0] mreg6,
    output        modem_loop    /* local analog loopback / self test active (data from the modem transmitter returns to its receiver) */
);

wire [23:0] byte_address = {a, 1'b0};
wire [7:0]  block        = byte_address[23:16];     /* which 64 kbyte block of the I/O space */
wire [1:0]  sub_block    = byte_address[11:10];     /* 1 kbyte pieces inside the 4B and 49 blocks */

localparam BLOCK_LINE_CONTROL = 8'h49;
localparam BLOCK_DIALER_BAUD  = 8'h4B;
localparam BLOCK_MODEM_882A   = 8'hE6;

localparam SUB_BAUD_LATCH     = 2'b00;
localparam SUB_DIALER_LOW     = 2'b01;
localparam SUB_DIALER_HIGH    = 2'b10;

/*
    ---- telephone status (450000) The 90 VAC ring burst goes through a bridge rectifier and an opto isolator, so the status bit stays low for the
    whole burst (the kernel samples it every 1/2 s and wants >= 1 s of continuous ring before it answers a data call). The message waiting lamp input
    blinks.
*/
reg                          message_lamp = 1'b1;
reg [MESSAGE_LAMP_BITS-1:0]  lamp_divider = 0;

always @(posedge clk) begin
    lamp_divider <= lamp_divider + 1'b1;
    if (lamp_divider == 0) message_lamp <= ~message_lamp;
end

telephone_status_reg_t status_fields;       /* layout: unixpc_registers_pkg.sv */

always @* begin
    status_fields.unused_15_4 = 12'hfff;
    status_fields.MSG_WAIT    = msg_wait ? message_lamp : 1'b1;
    status_fields.L2_RING     = ~ring_l2;
    status_fields.L1_RING     = ~ring_l1;
    status_fields.OFF_HOOK    = ~ext_offhook;
end

assign status = status_fields;

/* ---- line control latches: eight latches, one per address A14:A12, the value comes on data bit 14 */
localparam LATCHES_RESET = 8'hce;           /* relays released: the active low signals are high */

reg [7:0] latch = LATCHES_RESET;

always @(posedge clk) begin
    if (rst) latch <= LATCHES_RESET;
    else if (wr_go && block == BLOCK_LINE_CONTROL && sub_block == 2'b00) latch[byte_address[14:12]] <= wdata[14];
end

assign hdset_relay = latch[0];
assign line_sel2   = latch[1];
assign hook1       = ~latch[2];             /* HOOK RELAY1*: 0 = relay on */
assign hook2       = ~latch[3];
assign l1_hold     = latch[4];
assign l2_hold     = latch[5];
assign l1_alead    = ~latch[6];             /* A-LEAD1* */
assign l2_alead    = ~latch[7];

/* ---- dialer (838A): the lower byte is kept, the upper byte completes the word and starts the 16 bit stream */
reg [7:0]  dialer_low_byte = 8'h00;
reg [15:0] dialer_word     = 16'h0000;
reg        dialer_toggle   = 1'b0;          /* flips with every word, so the harness sees a new word even if it equals the last one */

always @(posedge clk) begin
    if (wr_go && block == BLOCK_DIALER_BAUD) begin
        if (sub_block == SUB_DIALER_LOW) dialer_low_byte <= byte_address[8:1];
        if (sub_block == SUB_DIALER_HIGH) begin
            dialer_word   <= {byte_address[8:1], dialer_low_byte};
            dialer_toggle <= ~dialer_toggle;
        end
    end
end

assign dial_tog  = dialer_toggle;
assign dial_word = dialer_word;

/* ---- baud rate latch 4L */
reg [7:0] baud_latch = 8'd2;

always @(posedge clk) begin
    if (wr_go && block == BLOCK_DIALER_BAUD && sub_block == SUB_BAUD_LATCH) baud_latch <= byte_address[8:1];
end

assign baud_n = baud_latch;

/* ---- 882A modem chip */
modem_882a modem
(
    .clk             (clk),
    .rst             (rst),
    .write           (wr_go && block == BLOCK_MODEM_882A),
    .register_select (byte_address[15:12]),
    .wdata           (wdata),
    .carrier         (carrier),
    .w0              (mreg0),
    .w1              (mreg1),
    .w3              (mreg3),
    .w4              (mreg4),
    .w5              (mreg5),
    .w6              (mreg6),
    .w8              (mreg8),
    .w9              (),
    .read_data       (e6_rdata),
    .loopback        (modem_loop)
);

endmodule
