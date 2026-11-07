/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The virtual printer behind the parallel port (FPGA only): turns the bytes the CPU prints into text on a sheet of paper.

    The paper is a ring of 32 text lines of 80 columns, kept in a block RAM of 4096 seven bit characters (address = {row[4:0], column[6:0]}). The line
    being typed is the bottom line of the picture; a line feed moves the whole text up by moving the ring's head, nothing is copied.

    Characters (plain ASCII):
      20..7E   printable: stored at the cursor, the cursor moves right; at column 80 the line wraps (as after a line feed)
      0D CR    cursor to the left margin
      0A LF    new line at the left margin (acts as CR LF)
      0C FF    new page: the paper is cleared
      08 BS    cursor one column left (not past the margin)
      09 TAB   cursor to the next multiple of 8 (to the last column from column 72 on)
      others   ignored
    "Tear off paper" (OSD) clears the paper like FF.

    A new line is cleared before it is typed on (the old text of the line that scrolls out is still in the ring), a new page clears the whole ring.
    While clearing, no byte is taken, and the port waits with its acknowledge (paper_clearing).
*/
module lp_paper_text
(
    input             clk,
    input             rst,

    input             tear,                     /* clear the paper (OSD trigger, level; the rising edge counts) */
    input             online,                   /* an off-line printer takes the byte from the port but prints nothing */

    /* from the port */
    input      [7:0]  printed_byte,
    input             byte_ready,
    output            byte_taken,               /* this clock the byte is taken (and printed if on line) */
    output            clearing,                 /* a line or the whole paper is being cleared */

    /* for the paper view */
    output reg [4:0]  head_row = 5'd0,          /* ring row of the line being typed (the bottom line) */
    output reg [2:0]  line_feed_phase = 3'd0,   /* number of line feeds modulo 6: the green bars move up with the text */
    input      [11:0] read_address,             /* text read port for the view: {row, column} */
    output reg [6:0]  read_data = 7'd0          /* the character, one clock later */
);

localparam COLUMNS          = 80;
localparam LAST_COLUMN      = COLUMNS - 1;
localparam LINES_PER_BAR_CYCLE = 6;             /* three green lines, three white lines: the bar pattern repeats every 6 lines */

localparam [7:0] CHAR_BACKSPACE = 8'h08, CHAR_TAB = 8'h09, CHAR_LINE_FEED = 8'h0a, CHAR_FORM_FEED = 8'h0c, CHAR_RETURN = 8'h0d;

/* -------------------------------------------------------------------------------------------------------------------------------- text RAM */
reg [6:0]  text_ram [0:4095];

reg        write_enable  = 1'b0;                /* registered write port: the RAM is written one clock after the decision */
reg [11:0] write_address = 12'd0;
reg [6:0]  write_data    = 7'd0;

always @(posedge clk) begin
    if (write_enable)
        text_ram[write_address] <= write_data;
    read_data <= text_ram[read_address];
end

/*
    ---- what the byte is
*/
reg        tear_was_high = 1'b0;
wire       tear_rising = tear & ~tear_was_high;                         /* rising edge of the OSD "tear off" */
wire       new_sheet_requested = tear_rising & ~clearing;               /* not while the paper is being cleared: that request is lost */

/* A byte is taken when the paper is not being cleared and no new sheet starts in this clock. */
assign byte_taken = byte_ready & ~clearing & ~new_sheet_requested;
wire   print_now  = byte_taken & online;

wire is_printable = ~printed_byte[7] & (printed_byte[6:0] >= 7'h20) & (printed_byte[6:0] <= 7'h7e);
wire is_return    = print_now & ~is_printable & (printed_byte == CHAR_RETURN);
wire is_line_feed = print_now & ~is_printable & (printed_byte == CHAR_LINE_FEED);
wire is_form_feed = print_now & ~is_printable & (printed_byte == CHAR_FORM_FEED);
wire is_backspace = print_now & ~is_printable & (printed_byte == CHAR_BACKSPACE);
wire is_tab       = print_now & ~is_printable & (printed_byte == CHAR_TAB);
wire print_char   = print_now & is_printable;

reg [6:0] column = 7'd0;                        /* cursor column 0..79 */
wire      at_last_column = (column == LAST_COLUMN);

wire start_new_line = (print_char & at_last_column) | is_line_feed;    /* the line is full, or LF: go to the left margin of the next line */
wire start_new_page = new_sheet_requested | is_form_feed;

/*
    ---- cursor and ring head
*/
wire [6:0] next_tab_stop = (column >= 7'd72) ? 7'd79 : {column[6:3] + 4'd1, 3'd0};    /* the next multiple of 8 */

always @(posedge clk) begin
    tear_was_high <= tear;

    if (print_char)
        column <= at_last_column ? 7'd0 : column + 7'd1;
    if (is_return)
        column <= 7'd0;
    if (is_line_feed)
        column <= 7'd0;
    if (is_backspace && column != 7'd0)
        column <= column - 7'd1;
    if (is_tab)
        column <= next_tab_stop;
    if (start_new_page)
        column <= 7'd0;

    if (start_new_line)
        head_row <= head_row + 5'd1;
    if (start_new_page)
        head_row <= 5'd0;

    if (start_new_line)
        line_feed_phase <= (line_feed_phase == LINES_PER_BAR_CYCLE - 1) ? 3'd0 : line_feed_phase + 3'd1;
    if (start_new_page)
        line_feed_phase <= 3'd0;

    if (rst) begin
        head_row <= 5'd0;
        column   <= 7'd0;
    end
end

/*
    ---- writing characters and clearing
*/
reg        clearing_active      = 1'b0;
reg        clearing_whole_paper = 1'b0;       /* 1: all 4096 cells, 0: one line of 128 cells */
reg [11:0] clear_address        = 12'd0;

assign clearing = clearing_active;

wire clear_is_last_cell = clearing_whole_paper ? (clear_address == 12'd4095) : (clear_address[6:0] == 7'd127);

always @(posedge clk) begin
    write_enable <= 1'b0;

    if (clearing_active) begin                                  /* write blanks over a line or over the whole ring */
        write_enable  <= 1'b1;
        write_address <= clear_address;
        write_data    <= 7'd0;
        clear_address <= clear_address + 12'd1;
        if (clear_is_last_cell)
            clearing_active <= 1'b0;
    end
    else begin
        if (print_char) begin
            write_enable  <= 1'b1;
            write_address <= {head_row, column};
            write_data    <= printed_byte[6:0];
        end

        if (start_new_page) begin
            clearing_active      <= 1'b1;
            clearing_whole_paper <= 1'b1;
            clear_address        <= 12'd0;
        end
        else if (start_new_line) begin
            clearing_active      <= 1'b1;
            clearing_whole_paper <= 1'b0;
            clear_address        <= {head_row + 5'd1, 7'd0};    /* the line that is about to be typed on */
        end
    end

    if (rst)
        clearing_active <= 1'b0;
end

endmodule
