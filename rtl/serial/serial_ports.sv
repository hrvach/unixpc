/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Where the two serial channels of the board go. Channel A is the RS-232 port: it goes to the MiSTer's UART, to the user port, or to both (OSD
    "Serial port"). Channel B is the built-in modem: in the UART mode "Modem" of the MiSTer menu the modem bridge (modem_bridge.sv) turns the guest's
    dialling into a call through MidiLink on the same UART. The call state is passed on for the call sounds.
*/
module serial_ports
(
    input         clk_sys,
    input         reset,
    input  [7:0]  uart_mode,  /* MiSTer menu: UART mode (1 = Modem) */
    input  [31:0] uart_speed,
    input         osd_serial_to_hps_uart,
    input         osd_serial_to_user_port,
    input  [1:0]  osd_modem_flow_control,
    input  [1:0]  ser_tx_valid,  /* from the serial chip: channel A, channel B send a byte */
    input  [7:0]  ser_tx_a,
    input  [7:0]  ser_tx_b,
    input  [1:0]  ser_rx_ready,  /* ... can take a byte */
    input  [1:0]  ser_dtr,
    input         telephone_hook_closed,  /* the telephone registers of the board */
    input         dialer_toggle,
    input  [15:0] dialer_word,
    input  [15:0] modem_chip_w0,
    input  [15:0] modem_chip_w1,
    input  [15:0] modem_chip_w8,
    output        channel_a_rx_valid,  /* to the serial chip */
    output [7:0]  channel_a_rx_data,
    output        modem_channel_b_rx_valid,
    output [7:0]  modem_channel_b_rx_data,
    output        modem_mode,  /* the HPS UART serves the modem, not channel A */
    output [2:0]  modem_call_state,  /* for the call sounds */
    output        modem_digits_dialled,
    output        modem_carrier,
    input         UART_RXD,
    output        UART_TXD,
    output        UART_RTS,
    output        UART_DTR,
    input  [6:0]  USER_IN,
    output [6:0]  USER_OUT
);

/* the modem bridge (rtl/serial/modem/modem_bridge.sv) */
wire        modem_receive_ready;
wire        modem_uart_tx_valid;
wire [7:0]  modem_uart_tx_data;
wire        modem_uart_rx_ready;

/*
    the HPS UART serves serial channel A or, in the UART mode "Modem" of the MiSTer menu (MidiLink's Hayes modem with TCP), the built-in modem on
    channel B
*/
assign modem_mode = (uart_mode == 8'd1);

/* the line speed the guest selected in the modem chip: 300 baud, 1200 (W1 bit 4), 2400 (W1 bit 4 and W8 bit 3) */
reg [1:0] modem_line_speed;

always @* begin
    if (!modem_chip_w1[4])
        modem_line_speed = 2'd0;
    else if (modem_chip_w8[3])
        modem_line_speed = 2'd2;
    else
        modem_line_speed = 2'd1;
end

wire        hps_uart_rx_valid;
wire [7:0]  hps_uart_rx_data;
wire        user_port_rx_valid;
wire [7:0]  user_port_rx_data;
wire        hps_txd;
wire        user_txd;

/*
    Where serial channel A goes (OSD "Serial port"): the MiSTer's UART (HPS, /dev/ttyS1), the user port of the I/O board (a 3.3 V TX / RX pair for an
    external RS-232 level shifter), or both. The built-in modem (channel B) always uses the HPS UART in the modem mode of the MiSTer menu, whatever is
    chosen here; the user port carries channel A only.
      TX  the characters of channel A go to the bridge of every selected port.
      RX  a character from either selected port is delivered to channel A (if both receive one at the same time, one is lost).
    Pins: USER_IO[1] = TX, USER_IO[0] = RX, as in the VT52 core. The user pins are open drain (a 1 floats), the level shifter board pulls TX up.
*/
/* one divider for both UARTs: they always run at the speed the OSD sets */
wire [24:0] uart_bit_clocks;

uart_baud_divider #(.CLK_HZ(20000000)) uart_baud_divider
(
    .clk        (clk_sys),
    .rst        (reset),
    .speed      (uart_speed),
    .bit_clocks (uart_bit_clocks)
);

uart_bridge hps_uart
(
    .clk        (clk_sys),
    .rst        (reset),
    .bit_clocks (uart_bit_clocks),
    .tx_valid (modem_mode ? modem_uart_tx_valid : (ser_tx_valid[0] & osd_serial_to_hps_uart)),
    .tx_data  (modem_mode ? modem_uart_tx_data : ser_tx_a),
    .rx_valid (hps_uart_rx_valid),
    .rx_data  (hps_uart_rx_data),
    .rx_ready (modem_mode ? modem_uart_rx_ready : ser_rx_ready[0]),
    .txd      (hps_txd),
    .rxd      (UART_RXD)
);

assign UART_TXD = hps_txd;

uart_bridge user_port_uart
(
    .clk        (clk_sys),
    .rst        (reset),
    .bit_clocks (uart_bit_clocks),
    .tx_valid (ser_tx_valid[0] & osd_serial_to_user_port),
    .tx_data  (ser_tx_a),
    .rx_valid (user_port_rx_valid),
    .rx_data  (user_port_rx_data),
    .rx_ready (ser_rx_ready[0]),
    .txd      (user_txd),
    .rxd      (USER_IN[0])
);


assign USER_OUT = {5'b11111, osd_serial_to_user_port ? user_txd : 1'b1, 1'b1};

wire       hps_rx_for_channel_a  = hps_uart_rx_valid & ~modem_mode & osd_serial_to_hps_uart;
wire       user_rx_for_channel_a = user_port_rx_valid & osd_serial_to_user_port;
assign channel_a_rx_valid = hps_rx_for_channel_a | user_rx_for_channel_a;
assign channel_a_rx_data = user_rx_for_channel_a ? user_port_rx_data : hps_uart_rx_data;

modem_bridge #(.CLK_HZ(20000000)) modem
(
    .clk        (clk_sys),
    .rst        (reset),
    .en         (modem_mode),
    .hook1      (telephone_hook_closed),
    .dial_tog   (dialer_toggle),
    .dial_word  (dialer_word),
    .mreg0      (modem_chip_w0),
    .b_tx_valid (ser_tx_valid[1]),
    .b_tx_data  (ser_tx_b),
    .b_rx_ready (ser_rx_ready[1]),
    .spd        (modem_line_speed),
    .u_tx_valid (modem_uart_tx_valid),
    .u_tx_data  (modem_uart_tx_data),
    .u_rx_valid (hps_uart_rx_valid),
    .u_rx_data  (hps_uart_rx_data),
    .u_rx_ready (modem_uart_rx_ready),
    .b_rx_valid (modem_channel_b_rx_valid),
    .b_rx_data  (modem_channel_b_rx_data),
    .carrier    (modem_carrier),
    .rx_ready_o (modem_receive_ready),
    .state      (modem_call_state),
    .digits     (modem_digits_dialled),
    .dbg        ()
);


/*
    Flow control towards MidiLink from the receive buffer of the modem bridge, in the modem mode. The OSD "Modem flow control": On = the buffer
    decides, Off = always ready (RTS 0), Opposite = ready is 1. Outside the modem mode UART_RTS is always "ready" (0): the HPS side of the UART
    then has its CTS asserted all the time. The login of the MiSTer on /dev/ttyS1 (UART mode Console) runs with hardware flow control and would
    stay silent while the guest does not raise RTS of channel A (cu does not), so the guest's RTS is not passed on.
*/
reg modem_rts;

always @* begin
    case (osd_modem_flow_control)
        2'd1:    modem_rts = 1'b0;
        2'd2:    modem_rts = modem_receive_ready;
        default: modem_rts = ~modem_receive_ready;
    endcase
end

assign UART_RTS = modem_mode ? modem_rts : 1'b0;
assign UART_DTR = ser_dtr[0];

endmodule
