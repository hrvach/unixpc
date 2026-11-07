/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Modem bridge: the 3B1's built-in modem (882A modem chip + 838A dialer, no AT commands) in front of a Hayes modem on the HPS UART, i.e. MiSTer's
    own "Modem" UART mode (MidiLink: ATDT host[:port], +++ATH, TCP behind it). FPGA only: on the real board the modem is connected to the telephone
    line.

    The guest dials like it would on a phone line: it closes the hook relay, sends DTMF words to the dialer, and sets W0 = 0x04 (end of number,
    connect).

      hook relay closed            -> off hook, the digit buffer is cleared                                          (OFF_HOOK)
      dial words with tone on      -> digits collected (DTMF code table of the kernel: dialer.h)                       (modem_digit_collector)
      W0 becomes 0x04 with digits  -> "ATDT<digits>\r" to the UART; the digits are the dialing directory key of MidiLink (MidiLink.DIR) or host:port
                                                                                                                        (SENDING_COMMAND,
                                                                                                                        modem_command_sender)
      "CONNECT" from the UART      -> carrier = 1 (the kernel sees answer tone, handshake, data mode), channel B data goes both ways
                                                                                                                        (WAITING_FOR_CONNECT ->
                                                                                                                        DATA_MODE)
      "NO CARRIER" in data mode    -> carrier = 0 (the kernel sees the loss twice and hangs up the line)
      hook relay opens             -> carrier = 0; after a guard time "+++ATH\r" is sent so the TCP connection is closed
                                      (SENDING_COMMAND, then ON_HOOK)

                       +----------------------+   digits    +--------------------+
      dial words ----> | modem_digit_collector| ----------> | modem_command_sender | ---> UART (Hayes commands)
                       +----------------------+             +--------------------+
      UART bytes ----> modem_text_matcher x2 (CONNECT, NO CARRIER), modem_stray_byte_catcher ---> state machine below UART bytes ----> modem_rx_buffer
      (8 KB, pacing, hold back) ---> channel B of the 7201 channel B bytes ---------------------------------------------> UART (data mode)
*/
module modem_bridge
import modem_pkg::*;
#(
    parameter CLK_HZ = 20000000
)
(
    input         clk,
    input         rst,
    input         en,             /* modem mode: the UART belongs to the modem, not to serial channel A */

    /* ---- from the board (telephony and channel B) */
    input         hook1,          /* hook relay closed: the line is seized */
    input         dial_tog,       /* toggles with every dial word */
    input  [15:0] dial_word,
    input  [15:0] mreg0,          /* W0 of the modem chip */
    input         b_tx_valid,     /* channel B sends a byte (data mode) */
    input  [7:0]  b_tx_data,
    input         b_rx_ready,     /* channel B can take a byte */
    input  [1:0]  spd,            /* line speed the guest selected (0 = 300, 1 = 1200, 2 = 2400 baud): received bytes are delivered at that pace */

    /* ---- UART bridge (byte level) */
    output reg    u_tx_valid,
    output reg [7:0] u_tx_data,
    input         u_rx_valid,
    input  [7:0]  u_rx_data,
    output        u_rx_ready,

    /* ---- to the board */
    output        b_rx_valid,
    output [7:0]  b_rx_data,
    output reg    carrier,
    output        rx_ready_o,     /* flow control: 1 = MidiLink may send */
    output [2:0]  state,          /* for the call sounds (line_sound.sv uses the numbers) */
    output        digits,         /* at least one digit has been dialled */
    output [3:0]  dbg             /* sticky progress of the last call: carrier, command sent, digits seen, hook seen */
);

/* ---- call states */
/* 3'd5 was a "hang up" state that is not used any more; a stray value goes back to CALL_ON_HOOK */

modem_call_state_t call_state = CALL_ON_HOOK;
reg       no_carrier_pending = 1'b0;      /* "NO CARRIER" arrived; carrier drops when the guest has read everything before it */

assign state = call_state;

wire running_normally = en && !rst;       /* reset and "modem mode off" put everything back to the start */

/* ---- "end of number": the guest writes W0 = 0x04 */
reg  end_of_number_before = 1'b0;
wire end_of_number = (mreg0[7:0] == 8'h04);
always @(posedge clk) end_of_number_before <= end_of_number;
wire end_of_number_now = end_of_number && !end_of_number_before;

/* ---- the digits */
wire [159:0] dialled_digits;
wire [4:0]   digit_count;
wire         is_ip_number;

modem_digit_collector digit_collector
(
    .clk          (clk),
    .clear        (rst || !en || (call_state == CALL_ON_HOOK && hook1)),
    .collect      (call_state == CALL_OFF_HOOK),
    .dial_tog     (dial_tog),
    .dial_word    (dial_word),
    .digits       (dialled_digits),
    .count        (digit_count),
    .is_ip_number (is_ip_number)
);

assign digits = (digit_count != 5'd0);

/* ---- what the UART says */
wire uart_byte_taken = u_rx_valid && u_rx_ready;
wire got_connect;
wire got_no_carrier;
wire stray_byte_valid;
wire [7:0] stray_byte;

modem_reply_parser reply_parser
(
    .clk              (clk),
    .clear_matchers   (rst || !en),
    .clear_stray      (rst || !en || call_state == CALL_ON_HOOK || call_state == CALL_OFF_HOOK),
    .line_active      (call_state == CALL_WAITING_FOR_CONNECT),
    .byte_valid       (uart_byte_taken),
    .byte_in          (u_rx_data),
    .got_connect      (got_connect),
    .got_no_carrier   (got_no_carrier),
    .stray_byte_valid (stray_byte_valid),
    .stray_byte       (stray_byte)
);

/* ---- the commands to the Hayes modem */
wire       command_send;
wire [7:0] command_data;
wire       command_done;
wire       command_is_hangup;

wire start_dial   = running_normally && call_state == CALL_OFF_HOOK && hook1 && end_of_number_now && digit_count != 5'd0;
wire start_hangup = running_normally && call_state == CALL_DATA_MODE && !hook1;

modem_command_sender #(.CLK_HZ(CLK_HZ)) command_sender
(
    .clk          (clk),
    .rst          (rst || !en),
    .start_dial   (start_dial),
    .start_hangup (start_hangup),
    .running      (running_normally && call_state == CALL_SENDING_COMMAND),
    .digits       (dialled_digits),
    .digit_count  (digit_count),
    .is_ip_number (is_ip_number),
    .send         (command_send),
    .send_data    (command_data),
    .done         (command_done),
    .hangup       (command_is_hangup)
);

/* ---- receive path in data mode */
wire guest_drained;

modem_rx_buffer #(.CLK_HZ(CLK_HZ)) rx_buffer
(
    .clk              (clk),
    .rst              (rst),
    .en               (en),
    .data_mode        (call_state == CALL_DATA_MODE),
    .byte_valid       (u_rx_valid),
    .byte_in          (u_rx_data),
    .byte_ready       (u_rx_ready),
    .rx_ready_out     (rx_ready_o),
    .no_carrier_seen  (got_no_carrier),
    .stray_byte_valid (stray_byte_valid),
    .stray_byte       (stray_byte),
    .carrier          (carrier),
    .mreg0            (mreg0),
    .speed_code       (spd),
    .guest_valid      (b_rx_valid),
    .guest_data       (b_rx_data),
    .guest_ready      (b_rx_ready),
    .drained          (guest_drained)
);

/* ---- progress flags of the current call (bring-up aid: shown on the LED strip of the lean dev build) */
reg seen_hook = 1'b0;
reg seen_digit = 1'b0;
reg seen_command = 1'b0;

always @(posedge clk) begin
    if (rst || !en) begin
        seen_hook    <= 1'b0;
        seen_digit   <= 1'b0;
        seen_command <= 1'b0;
    end
    else begin
        if (hook1 && call_state == CALL_ON_HOOK) begin
            seen_hook    <= 1'b1;
            seen_digit   <= 1'b0;
            seen_command <= 1'b0;
        end
        if (digit_count != 5'd0) seen_digit <= 1'b1;
        if (call_state == CALL_SENDING_COMMAND) seen_command <= 1'b1;
    end
end

assign dbg = {carrier, seen_command, seen_digit, seen_hook};

/* ---- the call state machine */
always @(posedge clk) begin
    u_tx_valid <= 1'b0;

    if (rst || !en) begin
        call_state         <= CALL_ON_HOOK;
        carrier            <= 1'b0;
        no_carrier_pending <= 1'b0;
    end
    else begin
        case (call_state)
            CALL_ON_HOOK: begin
                if (hook1) call_state <= CALL_OFF_HOOK;
            end

            CALL_OFF_HOOK: begin
                if (!hook1)                call_state <= CALL_ON_HOOK;
                else if (start_dial)       call_state <= CALL_SENDING_COMMAND;
            end

            CALL_SENDING_COMMAND: begin
                if (command_done) call_state <= command_is_hangup ? CALL_ON_HOOK : CALL_WAITING_FOR_CONNECT;
            end

            CALL_WAITING_FOR_CONNECT: begin
                if (got_connect) begin
                    carrier    <= 1'b1;
                    call_state <= CALL_DATA_MODE;
                end
                else if (!hook1) call_state <= CALL_ON_HOOK;
            end

            CALL_DATA_MODE: begin
                if (got_no_carrier) no_carrier_pending <= 1'b1;
                if (no_carrier_pending && guest_drained) begin      /* let the guest read what arrived before the line dropped */
                    carrier            <= 1'b0;
                    no_carrier_pending <= 1'b0;
                end
                if (start_hangup) begin                             /* "+++" guard, "+++", guard, ATH */
                    carrier    <= 1'b0;
                    call_state <= CALL_SENDING_COMMAND;
                end
            end

            default: call_state <= CALL_ON_HOOK;
        endcase

        /* the commands go out on the UART, and in data mode what channel B sends */
        if (call_state == CALL_SENDING_COMMAND && command_send) begin
            u_tx_valid <= 1'b1;
            u_tx_data  <= command_data;
        end
        if (call_state == CALL_DATA_MODE && b_tx_valid) begin
            u_tx_valid <= 1'b1;
            u_tx_data  <= b_tx_data;
        end
    end
end

endmodule
