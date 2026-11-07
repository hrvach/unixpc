/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The main RAM of the board: the SDRAM on the add-on board. The board works on its own 20 MHz clock and asks for one access at a time with a toggle
    handshake (rtl/board/ram_port.sv); sdram_bridge.sv carries that over to the 100 MHz clock of the SDRAM controller (sdram.sv).
*/
module main_ram
(
    input         clk_sys,  /* 20 MHz: the board */
    input         clk_sd,  /* 100 MHz: the SDRAM */
    input         reset,
    input         sdram_request_toggle,  /* the toggle handshake of the board (rtl/board/ram_port.sv) */
    input         sdram_write,
    input  [21:1] sdram_word_address,
    input  [1:0]  sdram_byte_enables,
    input  [15:0] sdram_write_data,
    input         sdram_parity_flags_in_use,
    input         sdram_store_bad_parity,
    output [15:0] sdram_read_data,
    output [1:0]  sdram_bad_parity_flags,
    output        sdram_acknowledge_toggle,
    output        sdram_fast_ok,
    inout  [15:0] SDRAM_DQ,  /* the SDRAM chip on the add-on board */
    output [12:0] SDRAM_A,
    output        SDRAM_DQML,
    output        SDRAM_DQMH,
    output [1:0]  SDRAM_BA,
    output        SDRAM_nCS,
    output        SDRAM_nWE,
    output        SDRAM_nRAS,
    output        SDRAM_nCAS,
    output        SDRAM_CKE,
    output        SDRAM_CLK
);

/* ---- main RAM in the SDRAM */
wire        sdram_init;
wire        sdram_read_pulse;
wire        sdram_write_pulse;
wire        sdram_ready;
wire        sdram_idle_ok;
wire [24:0] sdram_controller_address;
wire  [1:0] sdram_controller_byte_mask;
wire [15:0] sdram_controller_write_data;
wire [15:0] sdram_controller_read_data;

sdram_bridge sdram_clock_bridge
(
    .clk                 (clk_sys),
    .clk_sd              (clk_sd),
    .rst                 (reset),
    .request_toggle      (sdram_request_toggle),
    .write               (sdram_write),
    .word_address        (sdram_word_address),
    .byte_enables        (sdram_byte_enables),
    .write_data          (sdram_write_data),
    .parity_flags_in_use (sdram_parity_flags_in_use),
    .store_bad_parity    (sdram_store_bad_parity),
    .read_data           (sdram_read_data),
    .bad_parity_flags    (sdram_bad_parity_flags),
    .acknowledge_toggle  (sdram_acknowledge_toggle),
    .sd_init             (sdram_init),
    .sd_rd               (sdram_read_pulse),
    .sd_we               (sdram_write_pulse),
    .sd_addr             (sdram_controller_address),
    .sd_wtbt             (sdram_controller_byte_mask),
    .sd_din              (sdram_controller_write_data),
    .sd_dout             (sdram_controller_read_data),
    .sd_ready            (sdram_ready),
    .sd_idle_ok          (sdram_idle_ok),
    .fast_ok             (sdram_fast_ok)
);

sdram16 sdram
(
    .init       (sdram_init),
    .clk        (clk_sd),
    .SDRAM_DQ   (SDRAM_DQ),
    .SDRAM_A    (SDRAM_A),
    .SDRAM_DQML (SDRAM_DQML),
    .SDRAM_DQMH (SDRAM_DQMH),
    .SDRAM_BA   (SDRAM_BA),
    .SDRAM_nCS  (SDRAM_nCS),
    .SDRAM_nWE  (SDRAM_nWE),
    .SDRAM_nRAS (SDRAM_nRAS),
    .SDRAM_nCAS (SDRAM_nCAS),
    .SDRAM_CKE  (SDRAM_CKE),
    .SDRAM_CLK  (SDRAM_CLK),
    .wtbt       (sdram_controller_byte_mask),
    .addr       (sdram_controller_address),
    .dout       (sdram_controller_read_data),
    .din        (sdram_controller_write_data),
    .we         (sdram_write_pulse),
    .rd         (sdram_read_pulse),
    .ready      (sdram_ready),
    .idle_ok    (sdram_idle_ok)
);

endmodule
