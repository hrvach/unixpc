/*
    Copyright (c) 2026 by Hrvoje Cavrak

    uPD7201 (Intel 8274 / Z80-SIO compatible) dual serial controller, asynchronous mode.

      channel A = RS-232 port, channel B = modem (see rtl/serial/modem/telephony.sv)

    Character level model with real timing. The FPGA does not make the individual bits of the line: a transmitted character occupies the transmitter
    for the time it would need on the line and then appears on tx_data_*; a received character is handed over by the host side and arrives in the
    receive FIFO one character time later. The bit time is the serial clock period (clocks_per_serial_clock_*, system clocks per RxC/TxC cycle) times
    the clock mode (x1/x16/x32/x64).

    Structure:
      upd7201_channel (x2)     write registers, read registers and interrupt sources of one channel
        upd7201_transmitter    holding register + shifter
        upd7201_receiver       shifter + overrun flag
        upd7201_rx_fifo        three characters
      this module              the two channels, WR2 (interrupt vector) and RR2, the interrupt priority, the CPU read latch

    Interrupts: receive (character available / special condition = overrun), transmit buffer empty, external/status (DCD, CTS). The 3B1 autovectors
    the interrupt, but RR2 and "status affects vector" are modelled anyway.
*/
module upd7201
(
    input         clk,
    input         rst,

    /* ---- CPU bus */
    input  [1:0]  sel,                  /* address bits [2:1]: 0 A data, 1 B data, 2 A control, 3 B control */
    input         cs,
    input         rd_go,                /* one-shot at read */
    input         wr_go,                /* one-shot at write */
    input  [7:0]  wdata,
    output [7:0]  rdata,
    output        irq,

    /* ---- modem lines (1 = asserted) */
    input         loopback,             /* test plug: each transmitter feeds its own receiver */
    input         loopback_b,           /* modem local analog loopback (882A): channel B transmitter feeds its receiver */
    input         dcd_a,
    input         cts_a,
    input         dcd_b,
    input         cts_b,
    input         ri_a,                 /* ring indicator of channel A (the 3B1 wires it to the SYNDET input: RR0 bit 4) */
    input  [15:0] clk_per_a,            /* system clocks per serial clock cycle */
    input  [15:0] clk_per_b,
    output [1:0]  dtr,                  /* WR5 bit 7 (bit 0 = A, bit 1 = B) */
    output [1:0]  rts,                  /* WR5 bit 1 (the 3B1 diagnostic writes 0x82 for DTR + RTS, as for the Intel 8274 / Z80 SIO) */
    output [1:0]  brk,                  /* WR5 bit 4 (send break) */

    /* ---- host side of the serial lines */
    output [1:0]  tx_valid,             /* one-clk pulse: a character left the transmitter (bit 0 = A, 1 = B) */
    output [7:0]  tx_data_a,
    output [7:0]  tx_data_b,
    input  [1:0]  rx_valid,             /* host offers a character */
    input  [7:0]  rx_data_a,
    input  [7:0]  rx_data_b,
    output [1:0]  rx_ready,             /* receiver can take a character now */

    output [63:0] dbg                   /* debug view (overlay): last RR0 read per channel, WR5, WR3, WR4 of channel A and transmit flags */
);

wire address_is_b = sel[0];
wire control   = sel[1];

/*
    ---- WR2: interrupt vector, written through either channel's pointer, read back in RR2 of channel B WR2 of channel A: bit 2 chooses the priority
    between the channels, bits 4:3 the vector mode. WR2 of channel B: the vector itself.
*/
reg [7:0] wr2_channel_a = 8'h00;
reg [7:0] wr2_vector    = 8'h00;
wire      wr2_load_a;
wire      wr2_load_b;

always @(posedge clk) begin
    if (rst) begin
        wr2_channel_a <= 8'h00;
        wr2_vector    <= 8'h00;
    end
    else begin
        if (wr2_load_a) wr2_channel_a <= wdata;
        if (wr2_load_b) wr2_vector    <= wdata;
    end
end

wire both_receivers_first = wr2_channel_a[2];
wire [1:0] vector_mode          = wr2_channel_a[4:3];

/* ---- the two channels */
wire [7:0] rdata_a;
wire [7:0] rdata_b;
wire [7:0] rr2_value;

wire rx_int_a, tx_int_a, ext_int_a, overrun_a;
wire rx_int_b, tx_int_b, ext_int_b, overrun_b;
wire status_affects_vector_b;

wire [7:0] wr3_a, wr4_a, wr5_a, last_rr0_a;
wire [7:0] wr3_b, wr4_b, wr5_b, last_rr0_b;
wire       tx_holding_full_a, tx_shifting_a, tx_enabled_a, rx_fifo_not_empty_a;
wire       tx_holding_full_b, tx_shifting_b, tx_enabled_b, rx_fifo_not_empty_b;

upd7201_channel channel_a
(
    .clk                     (clk),
    .rst                     (rst),

    .selected                (cs & ~address_is_b),
    .control                 (control),
    .rd_go                   (rd_go),
    .wr_go                   (wr_go),
    .wdata                   (wdata),
    .rr2_value               (rr2_value),
    .rr0_interrupt_bit       (irq),
    .rdata                   (rdata_a),
    .wr2_load                (wr2_load_a),

    .dcd                     (dcd_a),
    .cts                     (cts_a),
    .ring_indicator          (ri_a),
    .loopback                (loopback),
    .clocks_per_serial_clock (clk_per_a),

    .tx_valid                (tx_valid[0]),
    .tx_data                 (tx_data_a),
    .rx_valid                (rx_valid[0]),
    .rx_data                 (rx_data_a),
    .rx_ready                (rx_ready[0]),

    .dtr                     (dtr[0]),
    .rts                     (rts[0]),
    .send_break              (brk[0]),

    .rx_interrupt            (rx_int_a),
    .tx_interrupt            (tx_int_a),
    .ext_interrupt           (ext_int_a),
    .rx_overrun              (overrun_a),
    .status_affects_vector   (),

    .wr3_value               (wr3_a),
    .wr4_value               (wr4_a),
    .wr5_value               (wr5_a),
    .last_rr0_read           (last_rr0_a),
    .tx_holding_full         (tx_holding_full_a),
    .tx_shifting             (tx_shifting_a),
    .tx_enabled              (tx_enabled_a),
    .rx_fifo_not_empty       (rx_fifo_not_empty_a)
);

upd7201_channel channel_b
(
    .clk                     (clk),
    .rst                     (rst),

    .selected                (cs & address_is_b),
    .control                 (control),
    .rd_go                   (rd_go),
    .wr_go                   (wr_go),
    .wdata                   (wdata),
    .rr2_value               (rr2_value),
    .rr0_interrupt_bit       (1'b0),
    .rdata                   (rdata_b),
    .wr2_load                (wr2_load_b),

    .dcd                     (dcd_b),
    .cts                     (cts_b),
    .ring_indicator          (1'b0),
    .loopback                (loopback | loopback_b),
    .clocks_per_serial_clock (clk_per_b),

    .tx_valid                (tx_valid[1]),
    .tx_data                 (tx_data_b),
    .rx_valid                (rx_valid[1]),
    .rx_data                 (rx_data_b),
    .rx_ready                (rx_ready[1]),

    .dtr                     (dtr[1]),
    .rts                     (rts[1]),
    .send_break              (brk[1]),

    .rx_interrupt            (rx_int_b),
    .tx_interrupt            (tx_int_b),
    .ext_interrupt           (ext_int_b),
    .rx_overrun              (overrun_b),
    .status_affects_vector   (status_affects_vector_b),

    .wr3_value               (wr3_b),
    .wr4_value               (wr4_b),
    .wr5_value               (wr5_b),
    .last_rr0_read           (last_rr0_b),
    .tx_holding_full         (tx_holding_full_b),
    .tx_shifting             (tx_shifting_b),
    .tx_enabled              (tx_enabled_b),
    .rx_fifo_not_empty       (rx_fifo_not_empty_b)
);

/* ---- interrupt request to the CPU */
assign irq = rx_int_a | rx_int_b | tx_int_a | tx_int_b | ext_int_a | ext_int_b;

/*
    ---- RR2 of channel B: the vector, with the code of the highest priority pending source in its low bits ("status affects vector", WR1 bit 2 of
    channel B) Codes (V2 V1 V0): channel B uses 0xx, channel A 1xx; within a channel 00 transmit, 01 external/status, 10 receive, 11 special receive
    condition.
*/
localparam VEC_TX_B       = 3'b000;
localparam VEC_EXT_B      = 3'b001;
localparam VEC_RX_B       = 3'b010;
localparam VEC_SPECIAL_B  = 3'b011;     /* special receive condition (overrun); also what is returned when nothing is pending */
localparam VEC_TX_A       = 3'b100;
localparam VEC_EXT_A      = 3'b101;
localparam VEC_RX_A       = 3'b110;
localparam VEC_SPECIAL_A  = 3'b111;

wire [2:0] rx_code_a = overrun_a ? VEC_SPECIAL_A : VEC_RX_A;
wire [2:0] rx_code_b = overrun_b ? VEC_SPECIAL_B : VEC_RX_B;

reg [2:0] pending_source_code;
always @* begin
    pending_source_code = VEC_SPECIAL_B;
    if (both_receivers_first) begin
        /* priority: receive A, receive B, transmit A, transmit B, external A, external B */
        if      (rx_int_a)  pending_source_code = rx_code_a;
        else if (rx_int_b)  pending_source_code = rx_code_b;
        else if (tx_int_a)  pending_source_code = VEC_TX_A;
        else if (tx_int_b)  pending_source_code = VEC_TX_B;
        else if (ext_int_a) pending_source_code = VEC_EXT_A;
        else if (ext_int_b) pending_source_code = VEC_EXT_B;
    end
    else begin
        /* priority: receive A, transmit A, receive B, transmit B, external A, external B */
        if      (rx_int_a)  pending_source_code = rx_code_a;
        else if (tx_int_a)  pending_source_code = VEC_TX_A;
        else if (rx_int_b)  pending_source_code = rx_code_b;
        else if (tx_int_b)  pending_source_code = VEC_TX_B;
        else if (ext_int_a) pending_source_code = VEC_EXT_A;
        else if (ext_int_b) pending_source_code = VEC_EXT_B;
    end
end

/*
    Where the code lands in the vector: in 8086/88 mode (WR2A bits 4:3 = 10, what the 3B1 kernel programs) it replaces V2..V0; the 8085 modes move it
    up (an approximation of the 8274 data sheet).
*/
localparam VECTOR_MODE_8085_LOW = 2'b00;
localparam VECTOR_MODE_8086   = 2'b10;

reg [7:0] vector_with_code;
always @* begin
    case (vector_mode)
        VECTOR_MODE_8086:   vector_with_code = {wr2_vector[7:3], pending_source_code};
        VECTOR_MODE_8085_LOW: vector_with_code = {wr2_vector[7:5], pending_source_code, wr2_vector[1:0]};
        default:            vector_with_code = {wr2_vector[7:6], pending_source_code, wr2_vector[2:0]};
    endcase
end

assign rr2_value = status_affects_vector_b ? vector_with_code : wr2_vector;

/*
    ---- CPU read data The read strobe pops the FIFO or resets the register pointer while the CPU is still waiting for the data: hold the value that
    was visible when the strobe came until the bus cycle ends.
*/
wire [7:0] rdata_now = address_is_b ? rdata_b : rdata_a;

reg [7:0] read_latch;
reg       read_latch_valid = 1'b0;
always @(posedge clk) begin
    if (cs & rd_go) begin
        read_latch       <= rdata_now;
        read_latch_valid <= 1'b1;
    end
    else if (!cs) begin
        read_latch_valid <= 1'b0;
    end
end

assign rdata = read_latch_valid ? read_latch : rdata_now;

/*
    ---- debug view for the on-screen overlay: bytes from the top: transmit/receive flags of B and A, WR3 B, WR4 A, WR3 A, WR5 B, WR5 A, last RR0 read
    of B and A
*/
assign dbg = {
    tx_holding_full_b, tx_shifting_b, tx_enabled_b, rx_fifo_not_empty_b,
    tx_holding_full_a, tx_shifting_a, tx_enabled_a, rx_fifo_not_empty_a,
    wr3_b,
    wr4_a,
    wr3_a,
    wr5_b,
    wr5_a,
    last_rr0_b,
    last_rr0_a
};

endmodule
