/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Bit-level 8N1 UART between the character-level 7201 channel A model (ser_tx_* / ser_rx_*) and the HPS UART pins of the MiSTer framework (FPGA
    only: there is no such chip on the 3B1, the real board has an RS-232 driver here).

    The baud rate arrives as `bit_clocks`, the number of system clocks per bit, from the shared uart_baud_divider (which turns the OSD speed of hps_io
    into it; the core has one divider for all its UARTs).

        bit_clocks  -->  uart_transmitter  (16 character FIFO, frame shifter)  --> txd
                         uart_receiver     (synchroniser, bit sampler, one character buffer)  <-- rxd
*/
module uart_bridge
#(
    parameter BIT_CLOCKS_BITS = 25
)
(
    input         clk,
    input         rst,

    input  [BIT_CLOCKS_BITS-1:0] bit_clocks,    /* system clocks per bit, from uart_baud_divider; 0 = idle line */

    /* from / to the 7201 model */
    input         tx_valid,
    input  [7:0]  tx_data,
    output        rx_valid,
    output [7:0]  rx_data,
    input         rx_ready,

    /* pins (core side naming of emu_ports.vh) */
    output        txd,
    input         rxd
);

uart_transmitter #(.BIT_CLOCKS_BITS(BIT_CLOCKS_BITS)) transmitter
(
    .clk             (clk),
    .rst             (rst),
    .bit_clocks      (bit_clocks),
    .character_valid (tx_valid),
    .character       (tx_data),
    .txd             (txd)
);

uart_receiver #(.BIT_CLOCKS_BITS(BIT_CLOCKS_BITS)) receiver
(
    .clk             (clk),
    .rst             (rst),
    .bit_clocks      (bit_clocks),
    .character_valid (rx_valid),
    .character       (rx_data),
    .character_ready (rx_ready),
    .rxd             (rxd)
);

endmodule
