/*
    Copyright (c) 2026 by Hrvoje Cavrak

    What the Hayes modem (MidiLink) says on the UART while a call is being set up (FPGA only), as seen by modem_bridge.sv: the two answers it waits
    for, "CONNECT" and "NO CARRIER", and the one data byte that MidiLink sends before its CONNECT line.

      modem_text_matcher         finds a fixed text in the stream of bytes (instantiated once for each answer)
      modem_stray_byte_catcher   catches the data byte before CONNECT, see its own header
      modem_reply_parser         the three of them together: one instance in the bridge
*/
module modem_reply_parser
(
    input        clk,
    input        clear_matchers,    /* the bridge is off or in reset: forget the partial matches */
    input        clear_stray,       /* not dialling (on hook, off hook, reset): forget the stray byte candidate */
    input        line_active,       /* the bridge waits for the answer to the dial command: the stray byte catcher looks at the bytes */
    input        byte_valid,        /* a byte arrives from the UART in this clock */
    input  [7:0] byte_in,

    output       got_connect,       /* "CONNECT" has just arrived (one clock) */
    output       got_no_carrier,    /* "NO CARRIER" has just arrived (one clock) */
    output       stray_byte_valid,  /* a candidate for the data byte before CONNECT exists */
    output [7:0] stray_byte
);

modem_text_matcher #(.TEXT("CONNECT"), .LENGTH(7)) connect_matcher
(
    .clk        (clk),
    .clear      (clear_matchers),
    .byte_valid (byte_valid),
    .byte_in    (byte_in),
    .found      (got_connect)
);

modem_text_matcher #(.TEXT("NO CARRIER"), .LENGTH(10)) no_carrier_matcher
(
    .clk        (clk),
    .clear      (clear_matchers),
    .byte_valid (byte_valid),
    .byte_in    (byte_in),
    .found      (got_no_carrier)
);

modem_stray_byte_catcher stray_catcher
(
    .clk             (clk),
    .clear           (clear_stray),
    .line_active     (line_active),
    .byte_valid      (byte_valid),
    .byte_in         (byte_in),
    .candidate_valid (stray_byte_valid),
    .candidate       (stray_byte)
);

endmodule


/*
    Finds a fixed text ("CONNECT", "NO CARRIER") in a stream of bytes, one byte at a time.

    `matched` counts how many characters of the text have arrived in a row. The next byte either extends the match, or, if it does not fit, starts
    over: with 1 if the byte is the first character of the text, with 0 otherwise. (This simple restart is enough for these two texts; it is not a
    general string search.) When the last character arrives, `found` is high for one clock.
*/
module modem_text_matcher
#(
    parameter [8*16-1:0] TEXT   = "CONNECT",    /* the characters, right aligned in the parameter */
    parameter            LENGTH = 7
)
(
    input        clk,
    input        clear,         /* forget the partial match */
    input        byte_valid,    /* a byte of the stream arrives in this clock */
    input  [7:0] byte_in,
    output reg   found = 1'b0   /* the whole text has just arrived */
);

/* character number i (0 = first) of the text */
function [7:0] text_character(input integer i);
    text_character = TEXT[8*(LENGTH-1-i) +: 8];
endfunction

reg [4:0] matched = 5'd0;

always @(posedge clk) begin
    found <= 1'b0;
    if (clear) begin
        matched <= 5'd0;
    end
    else if (byte_valid) begin
        if (byte_in == text_character(matched)) begin
            if (matched == LENGTH - 1) begin
                matched <= 5'd0;
                found   <= 1'b1;
            end
            else begin
                matched <= matched + 5'd1;
            end
        end
        else begin
            matched <= (byte_in == text_character(0)) ? 5'd1 : 5'd0;
        end
    end
end

endmodule


/*
    Catches the one data byte that MidiLink sends BEFORE its "CONNECT" line.

    MidiLink writes the first byte of the TCP stream to the UART before its "\r\nCONNECT 1200\r\n" line (seen with gdb on its write() calls: "DIALING
    ...", then one byte, then CONNECT, then the rest of the data). A response line that has exactly one character and ends with CR, directly before
    the CONNECT line, is therefore data and has to be handed to the guest first. The candidate is dropped again when the next line has a second
    character and does not start with "C": then it was not the line just before CONNECT.

    While the bridge waits for the answer to the dial command, the module follows the lines: it counts the characters of the current line (CR and LF
    not counted, saturating at 2), remembers the first one, and when a CR ends a line of exactly one character that character becomes the candidate.
*/
module modem_stray_byte_catcher
(
    input        clk,
    input        clear,             /* not dialling (on hook, off hook, reset): forget everything */
    input        line_active,       /* the bridge waits for the answer to the dial command: look at the received bytes */
    input        byte_valid,        /* a byte arrives from the UART in this clock */
    input  [7:0] byte_in,

    output reg       candidate_valid = 1'b0,
    output reg [7:0] candidate       = 8'd0
);

localparam LF = 8'h0a;
localparam CR = 8'h0d;

reg [1:0] characters_in_line = 2'd0;    /* 0, 1, or 2 meaning "two or more" */
reg [7:0] first_character    = 8'd0;

always @(posedge clk) begin
    if (clear) begin
        characters_in_line <= 2'd0;
        candidate_valid    <= 1'b0;
    end
    else if (byte_valid && line_active) begin
        if (byte_in == LF) begin
            characters_in_line <= 2'd0;
        end
        else if (byte_in == CR) begin
            if (characters_in_line == 2'd1) begin           /* a line of exactly one character just ended */
                candidate_valid <= 1'b1;
                candidate       <= first_character;
            end
        end
        else begin
            if (characters_in_line == 2'd0) first_character <= byte_in;
            if (characters_in_line == 2'd1 && first_character != "C") candidate_valid <= 1'b0;
            if (characters_in_line != 2'd2) characters_in_line <= characters_in_line + 2'd1;
        end
    end
end

endmodule
