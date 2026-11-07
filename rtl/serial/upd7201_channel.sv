/*
    Copyright (c) 2026 by Hrvoje Cavrak

    One channel (A or B) of the uPD7201 / Intel 8274 dual serial controller: write registers WR1 / WR3 / WR4 / WR5, read registers RR0 / RR1 / RR8,
    the transmitter, the receiver with its FIFO, and the three interrupt sources of the channel.

    How the CPU talks to the channel (same on both channels, the chip's address bit A1 chooses data or control, A2 the channel):
      - data register:    write = a character to send, read = oldest received character
      - control register: a write to WR0 chooses a register for the NEXT access (pointer) and may give a command; the access after that goes to the
                          chosen register, then the pointer is 0 again. A read of the control register returns RR0, or the register the pointer chose.

    WR0 is laid out like the Z80 SIO: bits D2..D0 pointer, bits D5..D3 command. (Some descriptions of this chip have these two fields swapped.)
*/
module upd7201_channel
(
    input         clk,
    input         rst,

    /* ---- CPU side (the parent has already decoded chip select and channel) */
    input         selected,             /* this channel is addressed */
    input         control,              /* 0: data register, 1: control register */
    input         rd_go,                /* one clock pulse when the CPU reads */
    input         wr_go,                /* one clock pulse when the CPU writes */
    input  [7:0]  wdata,
    input  [7:0]  rr2_value,            /* RR2 (interrupt vector) as built by the parent, the registers of both channels are involved */
    input         rr0_interrupt_bit,    /* RR0 bit 1 "interrupt pending": the chip reports it in channel A only, channel B reads 0 */
    output [7:0]  rdata,                /* what a read gets now (the parent latches it for the length of the bus cycle) */
    output        wr2_load,             /* the CPU writes WR2 (interrupt vector, kept by the parent) */

    /* ---- modem lines (1 = asserted) */
    input         dcd,                  /* data carrier detect */
    input         cts,                  /* clear to send */
    input         ring_indicator,       /* the 3B1 wires it to the SYNDET input of channel A: RR0 bit 4. Channel B has no such input. */
    input         loopback,             /* test plug / modem loopback: the transmitter feeds the receiver of this channel */
    input  [15:0] clocks_per_serial_clock, /* system clocks per RxC / TxC cycle */

    /* ---- the line, one character at a time */
    output        tx_valid,
    output [7:0]  tx_data,
    input         rx_valid,
    input  [7:0]  rx_data,
    output        rx_ready,

    /* ---- outputs of the WR5 register */
    output        dtr,                  /* WR5 bit 7 */
    output        rts,                  /* WR5 bit 1 */
    output        send_break,           /* WR5 bit 4 */

    /* ---- interrupt sources and what the parent needs to prioritise them */
    output        rx_interrupt,
    output        tx_interrupt,
    output        ext_interrupt,
    output        rx_overrun,
    output        status_affects_vector, /* WR1 bit 2 (the parent uses channel B's) */

    /* ---- debug view */
    output [7:0]  wr3_value,
    output [7:0]  wr4_value,
    output [7:0]  wr5_value,
    output [7:0]  last_rr0_read,
    output        tx_holding_full,
    output        tx_shifting,
    output        tx_enabled,
    output        rx_fifo_not_empty
);

/* ---- WR0 commands (bits D5..D3) */
localparam CMD_RESET_EXT_STATUS_INT = 3'b010;   /* forget the pending external / status change */
localparam CMD_CHANNEL_RESET        = 3'b011;
localparam CMD_ENABLE_RX_INT_NEXT   = 3'b100;   /* "enable interrupt on next received character" */
localparam CMD_RESET_TX_INT_PENDING = 3'b101;
localparam CMD_ERROR_RESET          = 3'b110;   /* clears the overrun flag */

/* ---- WR1 interrupt enables / modes */
localparam RX_INT_DISABLED   = 2'b00;           /* WR1 bits 4:3 */
localparam RX_INT_FIRST_CHAR = 2'b01;           /* only the first character after the enable command, then only errors */
/* any other value: every character */

/* ---- character length codes, same coding in WR3 (receiver) and WR5 (transmitter) */
localparam LENGTH_5_BITS = 2'b00;
localparam LENGTH_7_BITS = 2'b01;
localparam LENGTH_6_BITS = 2'b10;

/* ---- registers written by the CPU */
reg [2:0] pointer = 3'd0;               /* register the next control access goes to */
reg [7:0] wr1_int_modes;                /* bit 0 external/status int enable, bit 1 Tx int enable, bits 4:3 Rx int mode, bit 2 status affects vector */
reg [7:0] wr3_rx_params;                /* bits 7:6 receive character length, bit 5 auto enables, bit 0 receiver enable */
reg [7:0] wr4_clock_mode;               /* bits 7:6 clock mode x1/x16/x32/x64, bits 3:2 stop bits, bit 0 parity enable */
reg [7:0] wr5_tx_params;                /* bit 7 DTR, bits 6:5 transmit character length, bit 4 send break, bit 3 transmitter enable, bit 1 RTS */

wire auto_enables = wr3_rx_params[5];   /* the receiver listens only while DCD is asserted, the transmitter sends only while CTS is asserted */
wire [1:0] clock_mode = wr4_clock_mode[7:6];
wire [1:0] stop_bits  = wr4_clock_mode[3:2];
wire       parity     = wr4_clock_mode[0];

assign wr3_value  = wr3_rx_params;
assign wr4_value  = wr4_clock_mode;
assign wr5_value  = wr5_tx_params;
assign dtr        = wr5_tx_params[7];
assign rts        = wr5_tx_params[1];
assign send_break = wr5_tx_params[4];
assign status_affects_vector = wr1_int_modes[2];

/* ---- decoding of the CPU access */
wire data_read        = selected & rd_go & ~control;
wire control_read     = selected & rd_go &  control;
wire data_write       = selected & wr_go & ~control;
wire pointer_write    = selected & wr_go &  control & (pointer == 3'd0);      /* WR0 */
wire register_write   = selected & wr_go &  control & (pointer != 3'd0);      /* WR1 .. WR7 */

wire [2:0] command = wdata[5:3];
wire cmd_reset_ext_status = pointer_write & (command == CMD_RESET_EXT_STATUS_INT);
wire cmd_channel_reset    = pointer_write & (command == CMD_CHANNEL_RESET);
wire cmd_enable_rx_int    = pointer_write & (command == CMD_ENABLE_RX_INT_NEXT);
wire cmd_reset_tx_int     = pointer_write & (command == CMD_RESET_TX_INT_PENDING);
wire cmd_error_reset      = pointer_write & (command == CMD_ERROR_RESET);

assign wr2_load = register_write & (pointer == 3'd2);

/*
    ---- character timing A character takes (start bit + data bits + parity bit + stop bits) serial clock cycles times the clock mode (the line clock
    is divided by 1 / 16 / 32 / 64 to get the bit time), and each serial clock cycle is clocks_per_serial_clock system clocks. The stop bits are
    counted in half bits so that 1.5 comes out exact.
*/
function [3:0] data_bits(input [1:0] length_code);
    case (length_code)
        LENGTH_5_BITS: data_bits = 4'd5;
        LENGTH_7_BITS: data_bits = 4'd7;
        LENGTH_6_BITS: data_bits = 4'd6;
        default:       data_bits = 4'd8;
    endcase
endfunction

function [31:0] stop_half_bits(input [1:0] stop_code);
    case (stop_code)
        2'b10:   stop_half_bits = 32'd3;                    /* 1.5 stop bits */
        2'b11:   stop_half_bits = 32'd4;                    /* 2 stop bits */
        default: stop_half_bits = 32'd2;                    /* 1 stop bit (00 is the synchronous mode, not modelled) */
    endcase
endfunction

function [31:0] clock_mode_divider(input [1:0] mode);
    case (mode)
        2'b00:   clock_mode_divider = 32'd1;
        2'b01:   clock_mode_divider = 32'd16;
        2'b10:   clock_mode_divider = 32'd32;
        default: clock_mode_divider = 32'd64;
    endcase
endfunction

function [31:0] system_clocks_per_character(input [1:0] length_code);
    reg [31:0] bits_without_stop;
    reg [31:0] half_bits;
    begin
        bits_without_stop = 32'd1 + data_bits(length_code) + (parity ? 32'd1 : 32'd0);      /* start bit + data + parity */
        half_bits         = bits_without_stop * 32'd2 + stop_half_bits(stop_bits);
        system_clocks_per_character = (half_bits * clock_mode_divider(clock_mode) * {16'd0, clocks_per_serial_clock}) >> 1;
    end
endfunction

wire [31:0] tx_clocks_per_character = system_clocks_per_character(wr5_tx_params[6:5]);
wire [31:0] rx_clocks_per_character = system_clocks_per_character(wr3_rx_params[7:6]);

/* ---- enables */
wire rx_enabled_now = wr3_rx_params[0] & (~auto_enables | dcd);
assign tx_enabled   = wr5_tx_params[3] & (~auto_enables | cts);

/* ---- receive FIFO, receiver, transmitter */
wire       fifo_full;
wire [1:0] fifo_count;
wire [7:0] fifo_head;
wire       rx_idle, rx_character_done;
wire [7:0] rx_character;
wire       tx_finishing;
wire [7:0] tx_finishing_data;
wire       tx_underrun;
wire       tx_interrupt_pending;

/*
    The looped-back transmitter character enters the FIFO at the end of its time on the line (if the receiver is on and there is room); a received
    character that completes in the same clock wins.
*/
wire loop_push = loopback & tx_finishing & wr3_rx_params[0] & ~fifo_full;
wire rx_push   = rx_character_done & ~fifo_full;

upd7201_rx_fifo rx_fifo
(
    .clk       (clk),
    .rst       (rst),
    .clear     (cmd_channel_reset),
    .push      (loop_push | rx_push),
    .push_data (rx_push ? rx_character : tx_finishing_data),
    .pop       (data_read),
    .count     (fifo_count),
    .not_empty (rx_fifo_not_empty),
    .full      (fifo_full),
    .head      (fifo_head)
);

/* Characters from the host are taken only while the receiver is on, idle and has room, and never while the plug loops the transmitter back. */
assign rx_ready = rx_enabled_now & rx_idle & ~fifo_full & ~loopback;

upd7201_receiver receiver
(
    .clk                  (clk),
    .rst                  (rst),
    .channel_reset        (cmd_channel_reset),
    .clear_overrun        (cmd_error_reset),
    .data_bits_code       (wr3_rx_params[7:6]),
    .clocks_per_character (rx_clocks_per_character),
    .accept               (rx_valid & rx_ready),
    .host_data            (rx_data),
    .fifo_full            (fifo_full),
    .idle                 (rx_idle),
    .character_done       (rx_character_done),
    .character            (rx_character),
    .overrun              (rx_overrun)
);

/* WR1 bit 1 switched on while the transmit buffer is empty: the interrupt is pending at once. */
wire tx_int_enable_rises = register_write & (pointer == 3'd1) & wdata[1] & ~wr1_int_modes[1] & ~tx_holding_full;

upd7201_transmitter transmitter
(
    .clk                  (clk),
    .rst                  (rst),
    .channel_reset        (cmd_channel_reset),
    .enable               (tx_enabled),
    .clocks_per_character (tx_clocks_per_character),
    .write_character      (data_write),
    .write_data           (wdata),
    .clear_interrupt      (cmd_reset_tx_int),
    .raise_interrupt      (tx_int_enable_rises),
    .interrupt_pending    (tx_interrupt_pending),
    .holding_full         (tx_holding_full),
    .shifting             (tx_shifting),
    .underrun             (tx_underrun),
    .finishing            (tx_finishing),
    .finishing_data       (tx_finishing_data),
    .sent                 (tx_valid),
    .sent_data            (tx_data)
);

/* ---- external / status interrupt: any change of DCD, CTS (and the ring indicator on channel A) sets it */
reg dcd_before = 1'b0;
reg cts_before = 1'b0;
reg ring_before = 1'b0;
reg ext_status_changed = 1'b0;

always @(posedge clk) begin
    if (rst) begin
        dcd_before         <= 1'b0;
        cts_before         <= 1'b0;
        ring_before        <= 1'b0;
        ext_status_changed <= 1'b0;
    end
    else begin
        if (dcd != dcd_before || cts != cts_before || ring_indicator != ring_before) begin
            ext_status_changed <= 1'b1;
            dcd_before         <= dcd;
            cts_before         <= cts;
            ring_before        <= ring_indicator;
        end
        if (cmd_reset_ext_status || cmd_channel_reset) ext_status_changed <= 1'b0;
    end
end

/*
    ---- interrupt requests of the channel "interrupt on first character" is armed by the WR0 command or by a read of the data register and stays
    armed (the chip's exact re-arming is not modelled).
*/
reg rx_first_char_armed = 1'b0;
always @(posedge clk) begin
    if (rst)                                       rx_first_char_armed <= 1'b0;
    else if (data_read || cmd_enable_rx_int)       rx_first_char_armed <= 1'b1;
end

reg rx_interrupt_request;
always @* begin
    case (wr1_int_modes[4:3])
        RX_INT_DISABLED:   rx_interrupt_request = 1'b0;
        RX_INT_FIRST_CHAR: rx_interrupt_request = (rx_fifo_not_empty & rx_first_char_armed) | rx_overrun;
        default:           rx_interrupt_request = rx_fifo_not_empty | rx_overrun;      /* every character, and the overrun error */
    endcase
end
assign rx_interrupt  = rx_interrupt_request;
assign tx_interrupt  = tx_interrupt_pending & wr1_int_modes[1];
assign ext_interrupt = ext_status_changed   & wr1_int_modes[0];

/*
    ---- read registers
    RR0: transmit/receive status and modem lines.  RR1: error flags, as of the Intel 8274 the 3B1 really has (its diagnostic reads D6..D4 as framing /
    overrun / parity and waits for D0 = all sent).
*/
localparam RESIDUE_CODE_ASYNC = 3'b011;     /* RR1 D3..D1 in asynchronous mode */

wire [7:0] rr0 = {
    1'b0,                                   /* D7 break/abort (not modelled) */
    tx_underrun,                            /* D6 transmit underrun / end of message */
    cts,                                    /* D5 */
    ring_indicator,                         /* D4 SYNDET input: on the 3B1 the ring indicator (channel A) */
    dcd,                                    /* D3 */
    ~tx_holding_full,                       /* D2 transmit buffer empty */
    rr0_interrupt_bit,                      /* D1 interrupt pending */
    rx_fifo_not_empty                       /* D0 receive character available */
};

wire [7:0] rr1 = {
    1'b0,                                   /* D7 end of frame */
    1'b0,                                   /* D6 CRC / framing error (not modelled) */
    rx_overrun,                             /* D5 */
    1'b0,                                   /* D4 parity error (not modelled) */
    RESIDUE_CODE_ASYNC,                     /* D3..D1 */
    ~tx_shifting & ~tx_holding_full         /* D0 all characters sent */
};

reg [7:0] register_value;
always @* begin
    case (pointer)
        3'd0:    register_value = rr0;
        3'd1:    register_value = rr1;
        3'd2:    register_value = rr2_value;
        default: register_value = 8'h00;
    endcase
end

assign rdata = control ? register_value : fifo_head;

reg [7:0] last_rr0 = 8'd0;
always @(posedge clk) if (control_read && pointer == 3'd0) last_rr0 <= rr0;
assign last_rr0_read = last_rr0;

/* ---- write registers and the pointer */
always @(posedge clk) begin
    if (rst) begin
        pointer       <= 3'd0;
        wr1_int_modes <= 8'h00;
        wr3_rx_params <= 8'h00;
        wr4_clock_mode<= 8'h04;             /* x1 clock, 1 stop bit, no parity */
        wr5_tx_params <= 8'h00;
    end
    else begin
        if (control_read) pointer <= 3'd0;

        if (pointer_write) pointer <= wdata[2:0];

        if (register_write) begin
            case (pointer)
                3'd1:    wr1_int_modes  <= wdata;
                3'd3:    wr3_rx_params  <= wdata;
                3'd4:    wr4_clock_mode <= wdata;
                3'd5:    wr5_tx_params  <= wdata;
                default: ;                  /* WR2 is kept by the parent, WR6 / WR7 are synchronous mode only */
            endcase
            pointer <= 3'd0;
        end

        if (cmd_channel_reset) begin
            wr1_int_modes <= 8'h00;
            wr3_rx_params[0] <= 1'b0;       /* receiver off */
            wr5_tx_params[3] <= 1'b0;       /* transmitter off */
            wr5_tx_params[7] <= 1'b0;       /* DTR off */
            wr5_tx_params[1] <= 1'b0;       /* RTS off */
        end
    end
end

endmodule
