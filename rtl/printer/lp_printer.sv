/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Virtual Centronics line printer for the 3B1's parallel port, with the printout shown on screen as a sheet of fan-fold paper (OSD "printer view").

    Three parts:
      lp_port.sv        the hardware side: data register, strobe, busy, acknowledge interrupt, status bits (what the logic board really has)
      lp_paper_text.sv  the virtual printer (FPGA only): characters -> text on the paper, a ring of 32 lines of 80 columns
      lp_paper_view.sv  the paper picture (FPGA only): the top level shows it instead of the machine's screen

    Bus side: the CPU writes the byte to the data register (4F0000), then toggles MCR bit 13
    (4A0000) from 1 to 0 (LPSTROBE). The printer is busy for a short while, then acknowledges with LPINT (level 2 interrupt, shared with the disk),
    which the next data write or strobe clears. Status bits (470000): 7 busy, 6 select (printer on line), 5 no paper, 4 error*.
*/
module lp_printer
(
    input             clk,
    input             rst,

    input             online,        /* printer on line (OSD) */
    input             tear,          /* clear the paper (OSD trigger, level) */
    input             wr_data,       /* CPU write to the data register */
    input      [7:0]  data,
    input             wr_mcr,        /* CPU write to the MCR */
    input             mcr13,         /* MCR bit 13 as written (LPSTROBE) */

    output            busy,
    output            select,
    output            nopaper,
    output            lpint,

    /* picture timing of the output (1 = visible pixel / sync pulse) */
    input             de,
    input             vs,
    output     [23:0] rgb,

    /* the font ROM (lp_font.sv), outside */
    output     [9:0]  font_address,
    input      [7:0]  font_row
);

wire [7:0]  printed_byte;
wire        byte_ready;
wire        byte_taken;
wire        paper_clearing;

wire [4:0]  head_row;
wire [2:0]  line_feed_phase;
wire [11:0] text_read_address;
wire [6:0]  text_read_data;

lp_port port
(
    .clk            (clk),
    .rst            (rst),
    .online         (online),
    .wr_data        (wr_data),
    .data           (data),
    .wr_mcr         (wr_mcr),
    .mcr13          (mcr13),
    .busy           (busy),
    .select         (select),
    .nopaper        (nopaper),
    .lpint          (lpint),
    .printed_byte   (printed_byte),
    .byte_ready     (byte_ready),
    .byte_taken     (byte_taken),
    .paper_clearing (paper_clearing)
);

lp_paper_text paper_text
(
    .clk             (clk),
    .rst             (rst),
    .tear            (tear),
    .online          (online),
    .printed_byte    (printed_byte),
    .byte_ready      (byte_ready),
    .byte_taken      (byte_taken),
    .clearing        (paper_clearing),
    .head_row        (head_row),
    .line_feed_phase (line_feed_phase),
    .read_address    (text_read_address),
    .read_data       (text_read_data)
);

lp_paper_view paper_view
(
    .clk               (clk),
    .de                (de),
    .vs                (vs),
    .head_row          (head_row),
    .line_feed_phase   (line_feed_phase),
    .text_read_address (text_read_address),
    .text_read_data    (text_read_data),
    .font_address      (font_address),
    .font_row          (font_row),
    .rgb               (rgb)
);

endmodule
