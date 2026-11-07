/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Sends the Hayes commands for the dialled number to MidiLink's modem on the UART: "ATDT<number>\r" to dial, "+++ATH\r" to hang up.

    One byte goes out every 10 ms (the UART FIFO would hold more, the pacing keeps the modem's input buffer happy).

    Dialing: a number of the form aaabbbcccddd[*port] (four 3-digit octets, optional '*' and port) is an IPv4 address and is sent as ATDT
    a.b.c.d[:port] with the leading zeros removed (a leading 0 would read as octal). Any other number is sent as dialled (a dialing directory entry of
    the Hayes modem).

    Hanging up: the "+++" escape needs a guard time of silence before and after it, so the sender waits 1.1 s, sends "+++", waits 1.1 s, sends
    "ATH\r". (start_hangup preloads the first guard time, the second one is loaded after the third '+'.)
*/
module modem_command_sender
#(
    parameter CLK_HZ = 20000000
)
(
    input         clk,
    input         rst,

    input         start_dial,       /* begin "ATDT..." with the digits below */
    input         start_hangup,     /* begin "+++ATH" (with the guard time first) */
    input         running,          /* the bridge is in the state "sending the command": the sender may advance and send */

    input  [159:0] digits,          /* from modem_digit_collector */
    input  [4:0]   digit_count,
    input          is_ip_number,

    output         send,            /* a byte goes out in this clock ... */
    output reg [7:0] send_data,     /* ... this one (combinational, the bridge registers it) */
    output         done,            /* the whole command is out (in this clock); the bridge moves on */
    output reg     hangup = 1'b0    /* the command being sent is the hang up command */
);

localparam BYTE_PACE   = CLK_HZ / 100;              /* 10 ms between bytes */
localparam GUARD_TIME  = CLK_HZ + CLK_HZ / 10;      /* 1.1 s before and after "+++" */
localparam HANGUP_LENGTH = 5'd7;                    /* "+++ATH\r" */
localparam DIAL_PREFIX   = 5'd4;                    /* "ATDT" */

localparam CR = 8'h0d;

reg [4:0]  index = 5'd0;                            /* next byte of the command (for an IP number: of the "ATDT" prefix only) */
reg [17:0] pace_left  = 18'd0;                      /* clocks until the next byte may go out */
reg [25:0] guard_left = 26'd0;                      /* clocks of guard time left */

wire [4:0] command_length = hangup ? HANGUP_LENGTH : (5'd5 + digit_count);     /* "ATDT" + digits + CR */

/* ---- the plain command: prefix, digits as dialled, CR; or the hang up command */
reg [7:0] plain_byte;
always @* begin
    if (hangup) begin
        case (index[2:0])
            3'd0, 3'd1, 3'd2: plain_byte = "+";
            3'd3:             plain_byte = "A";
            3'd4:             plain_byte = "T";
            3'd5:             plain_byte = "H";
            default:          plain_byte = CR;
        endcase
    end
    else if (index < DIAL_PREFIX) begin
        case (index[1:0])
            2'd0:    plain_byte = "A";
            2'd1:    plain_byte = "T";
            2'd2:    plain_byte = "D";
            default: plain_byte = "T";
        endcase
    end
    else if (index < DIAL_PREFIX + digit_count) begin
        plain_byte = digits[{index - DIAL_PREFIX, 3'd0} +: 8];
    end
    else begin
        plain_byte = CR;
    end
end

/* ---- IP number translation: "aaabbbcccddd*port" becomes "a.b.c.d:port" with the zeros in front of an octet removed */
localparam PHASE_OCTET_DIGITS = 2'd0;
localparam PHASE_SEPARATOR    = 2'd1;               /* '.' between octets; after the last octet ':' + port, or the end of the line */
localparam PHASE_PORT_DIGITS  = 2'd2;
localparam PHASE_END_OF_LINE  = 2'd3;
localparam FIRST_PORT_DIGIT   = 5'd13;              /* 12 digits and the '*' */

reg        translating = 1'b0;                      /* this call dials an IP number */
reg [1:0]  phase       = PHASE_OCTET_DIGITS;
reg [1:0]  octet       = 2'd0;                      /* 0..3 */
reg [1:0]  digit_in_octet = 2'd0;                   /* 0..2 */
reg        octet_started = 1'b0;                    /* a digit other than a leading zero of this octet has been sent */
reg [4:0]  port_index  = 5'd0;
reg        line_ended  = 1'b0;

wire [7:0] octet_digit = digits[{(5'd3 * {3'd0, octet}) + {3'd0, digit_in_octet}, 3'd0} +: 8];
wire       skip_leading_zero = !octet_started && digit_in_octet != 2'd2 && octet_digit == "0"; /* the last digit of an octet is always sent */

wire translating_now = translating && !hangup && index >= DIAL_PREFIX;
wire waiting         = (guard_left != 26'd0) || (pace_left != 18'd0);

/* something goes out (translation) / (plain command) */
reg        translated_send;
reg [7:0]  translated_data;
always @* begin
    translated_send = 1'b0;
    translated_data = 8'h00;
    if (translating_now && !line_ended) begin
        case (phase)
            PHASE_OCTET_DIGITS: begin
                translated_send = !skip_leading_zero;
                translated_data = octet_digit;
            end
            PHASE_SEPARATOR: begin
                translated_send = 1'b1;
                if (octet != 2'd3)                    translated_data = ".";
                else if (digit_count > 5'd13)         translated_data = ":";
                else                                  translated_data = CR;
            end
            PHASE_PORT_DIGITS: begin
                translated_send = 1'b1;
                translated_data = digits[{port_index, 3'd0} +: 8];
            end
            default: begin
                translated_send = 1'b1;
                translated_data = CR;
            end
        endcase
    end
end

wire plain_done = !translating_now && (index == command_length);
wire plain_send = !translating_now && (index != command_length);

assign send = running && !waiting && (translating_now ? translated_send : plain_send);

always @* send_data = translating_now ? translated_data : plain_byte;

assign done = running && !waiting && (translating_now ? line_ended : plain_done);

always @(posedge clk) begin
    if (rst) begin
        index <= 5'd0;
    end
    else if (start_dial) begin
        index          <= 5'd0;
        pace_left      <= 18'd0;
        hangup         <= 1'b0;
        translating    <= is_ip_number;
        phase          <= PHASE_OCTET_DIGITS;
        octet          <= 2'd0;
        digit_in_octet <= 2'd0;
        octet_started  <= 1'b0;
        line_ended     <= 1'b0;
    end
    else if (start_hangup) begin
        index      <= 5'd0;
        pace_left  <= 18'd0;
        hangup     <= 1'b1;
        guard_left <= GUARD_TIME;
    end
    else if (running) begin
        if (guard_left != 26'd0) begin
            guard_left <= guard_left - 26'd1;
        end
        else if (pace_left != 18'd0) begin
            pace_left <= pace_left - 18'd1;
        end
        else if (translating_now) begin
            if (!line_ended) begin
                case (phase)
                    PHASE_OCTET_DIGITS: begin
                        if (skip_leading_zero) begin
                            digit_in_octet <= digit_in_octet + 2'd1;
                        end
                        else begin
                            pace_left     <= BYTE_PACE;
                            octet_started <= 1'b1;
                            if (digit_in_octet == 2'd2) begin
                                digit_in_octet <= 2'd0;
                                octet_started  <= 1'b0;
                                phase          <= PHASE_SEPARATOR;
                            end
                            else begin
                                digit_in_octet <= digit_in_octet + 2'd1;
                            end
                        end
                    end
                    PHASE_SEPARATOR: begin
                        pace_left <= BYTE_PACE;
                        if (octet != 2'd3) begin
                            octet <= octet + 2'd1;
                            phase <= PHASE_OCTET_DIGITS;
                        end
                        else if (digit_count > 5'd13) begin
                            port_index <= FIRST_PORT_DIGIT;
                            phase      <= PHASE_PORT_DIGITS;
                        end
                        else begin
                            line_ended <= 1'b1;
                        end
                    end
                    PHASE_PORT_DIGITS: begin
                        pace_left  <= BYTE_PACE;
                        port_index <= port_index + 5'd1;
                        if (port_index + 5'd1 == digit_count) phase <= PHASE_END_OF_LINE;
                    end
                    default: begin
                        pace_left  <= BYTE_PACE;
                        line_ended <= 1'b1;
                    end
                endcase
            end
        end
        else if (index != command_length) begin
            index     <= index + 5'd1;
            pace_left <= BYTE_PACE;
            if (hangup && index == 5'd2) guard_left <= GUARD_TIME;      /* the guard time after the "+++" */
        end
    end
end

endmodule
