/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Collects the digits the guest dials (FPGA side of the 838A dialer).

    The kernel dials with DTMF: for every digit it writes a 16 bit word to the dialer, once with the tone on and once with the tone off (see
    telephony.sv: every word toggles dial_tog). The tone-on words carry the kernel's digit code in the low nibble. This module turns the codes into
    characters and keeps up to 20 of them, first digit first. When the number is complete it also tells whether it is an IP address (see
    is_ip_number).

    Pulse dialing is not decoded (the kernel uses DTMF by default).
*/
module modem_digit_collector
(
    input         clk,

    input         clear,            /* a new call starts: forget the digits */
    input         collect,          /* the line is off hook and the number is being dialled */
    input         dial_tog,
    input  [15:0] dial_word,

    output reg [159:0] digits = 160'd0,     /* 20 characters, the first digit in the low byte */
    output reg [4:0]   count  = 5'd0,
    output             is_ip_number         /* aaabbbcccddd or aaabbbcccddd*port: see the command sender */
);

localparam MAX_DIGITS = 20;

/* ---- the kernel's DTMF nibble codes (dialer.h) to characters */
function [7:0] digit_character(input [3:0] code);
    case (code)
        4'h0:    digit_character = "1";
        4'h8:    digit_character = "2";
        4'h4:    digit_character = "3";
        4'h1:    digit_character = "4";
        4'h9:    digit_character = "5";
        4'h5:    digit_character = "6";
        4'h2:    digit_character = "7";
        4'hA:    digit_character = "8";
        4'h6:    digit_character = "9";
        4'hB:    digit_character = "0";
        4'h3:    digit_character = "*";
        4'h7:    digit_character = "#";
        default: digit_character = "?";
    endcase
endfunction

/* A dial word with the tone on is 0x0930 | code (+0x0A00); tone off is 0x1000 / 0x1A00. */
wire tone_on = (dial_word[7:4] == 4'h3) && (dial_word[11] | dial_word[8]);

reg dial_tog_before = 1'b0;
wire new_word = (dial_tog != dial_tog_before);

always @(posedge clk) begin
    dial_tog_before <= dial_tog;
    if (clear) begin
        count <= 5'd0;
    end
    else if (collect && new_word && tone_on && count < MAX_DIGITS) begin
        digits[{count, 3'd0} +: 8] <= digit_character(dial_word[3:0]);
        count <= count + 5'd1;
    end
end

/* ---- IP number: four 3-digit octets, optionally followed by '*' and a port */
function is_decimal_digit(input [7:0] c);
    is_decimal_digit = (c >= "0") && (c <= "9");
endfunction

function first_twelve_are_digits(input [159:0] text);
    integer i;
    begin
        first_twelve_are_digits = 1'b1;
        for (i = 0; i < 12; i = i + 1)
            if (!is_decimal_digit(text[8*i +: 8])) first_twelve_are_digits = 1'b0;
    end
endfunction

wire [7:0] thirteenth_character = digits[103:96];

assign is_ip_number = (count >= 5'd12) && first_twelve_are_digits(digits) && (count == 5'd12 || thirteenth_character == "*");

endmodule
