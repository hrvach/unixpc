/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Receive path of the modem bridge in data mode: bytes from MidiLink's UART wait in an 8 KB buffer, and are handed to serial channel B at the pace
    the guest has selected, but only once the guest is ready for them.

    Why a buffer: a BBS screenful arrives at once, the guest takes it at its own speed (300 / 1200 / 2400 baud).

    What this module does not pass on:
      - the rest of the "CONNECT 1200" line that follows the CONNECT (dropped up to its LF)
      - the text "NO CARRIER": MidiLink sends it as plain bytes in the data stream when the far end hangs up. The last 10 received bytes are therefore
        held back until the line has been quiet for 15 ms, and when the text is recognised (no_carrier_seen) it is taken back out of the buffer again,
        so the guest never sees it, but still receives everything that came before it.

    When the guest is ready: a real far end sends nothing before the handshake is done, and the guest's receiver is not enabled earlier. The bridge
    therefore waits until W0 of the modem chip went 0x24 -> 0x04 after the carrier came (the kernel finished the handshake), plus 4 s (the first byte
    would be eaten while cu still reconfigures the line).

    Flow control towards MidiLink (the Linux side of ttyS1 runs with crtscts): rx_ready_out is 1 until the buffer is 5/8 full, and 1 again below 1/4.
*/
module modem_rx_buffer
#(
    parameter CLK_HZ = 20000000
)
(
    input         clk,
    input         rst,
    input         en,                       /* modem mode (the pacing timer runs whenever the modem owns the UART) */
    input         data_mode,                /* the call is in data mode (otherwise the buffer is empty and the UART bytes are consumed elsewhere) */

    /* ---- from the UART */
    input         byte_valid,
    input  [7:0]  byte_in,
    output        byte_ready,               /* the UART may deliver the next byte (always, except when the buffer is full) */
    output        rx_ready_out,             /* flow control: 1 = MidiLink may send */

    /* ---- events from the rest of the bridge */
    input         no_carrier_seen,          /* the text "NO CARRIER" has just completed in the stream */
    input         stray_byte_valid,         /* the one data byte that came before CONNECT (modem_stray_byte_catcher) */
    input  [7:0]  stray_byte,
    input         carrier,
    input  [15:0] mreg0,                    /* W0 of the modem chip: 0x24 = handshake running (bit 5), 0x04 = data mode */
    input  [1:0]  speed_code,               /* 0 = 300, 1 = 1200, 2 = 2400 baud: bytes are delivered at that pace (10 bits per byte) */

    /* ---- to serial channel B */
    output        guest_valid,
    output [7:0]  guest_data,
    input         guest_ready,

    output        drained                   /* nothing waiting and no byte in the pacing gap: the guest has read everything */
);

localparam BUFFER_BYTES = 8192;
localparam POINTER_BITS = 14;                       /* one more than the index so that full and empty differ */

localparam [POINTER_BITS-1:0] STOP_ASKING_AT   = 14'd5120;      /* 5/8 full */
localparam [POINTER_BITS-1:0] ASK_AGAIN_BELOW  = 14'd2048;      /* 1/4 full */
localparam [POINTER_BITS-1:0] HELD_BACK_BYTES  = 14'd10;        /* the length of "NO CARRIER" */

localparam HANDSHAKE_WAIT = CLK_HZ * 4;
localparam QUIET_TIME     = CLK_HZ / 1000 * 15;                 /* 15 ms without a byte */

wire reading_state = en && data_mode && !rst;       /* otherwise everything restarts */

/* ---- the rest of the CONNECT line is dropped up to the line feed */
reg dropping_connect_line = 1'b1;

always @(posedge clk) begin
    if (!reading_state) dropping_connect_line <= 1'b1;
    else if (dropping_connect_line && byte_valid && byte_ready && byte_in == 8'h0a) dropping_connect_line <= 1'b0;
end

/* ---- is the guest ready? W0 has to show the handshake state (bit 5) and then leave it; 4 s after that */
reg        saw_handshake_state = 1'b0;
reg        guest_in_data_mode  = 1'b0;
reg [26:0] handshake_timer     = 27'd0;

always @(posedge clk) begin
    if (!reading_state) begin
        saw_handshake_state <= 1'b0;
        guest_in_data_mode  <= 1'b0;
        handshake_timer     <= 27'd0;
    end
    else begin
        if (mreg0[5]) saw_handshake_state <= 1'b1;
        if (saw_handshake_state && !mreg0[5] && !guest_in_data_mode) begin
            handshake_timer <= handshake_timer + 27'd1;
            if (handshake_timer == HANDSHAKE_WAIT) guest_in_data_mode <= 1'b1;
        end
    end
end

/* ---- the buffer: block RAM with a registered read; write pointer, read pointer */
reg [7:0] memory [0:BUFFER_BYTES-1];
reg [POINTER_BITS-1:0] write_pointer = 0;
reg [POINTER_BITS-1:0] read_pointer  = 0;

wire [POINTER_BITS-1:0] bytes_held = write_pointer - read_pointer;
wire buffer_empty = (write_pointer == read_pointer);
wire buffer_full  = (bytes_held == BUFFER_BYTES);

assign byte_ready = data_mode ? !buffer_full : 1'b1;       /* outside data mode the responses are consumed by the bridge */

wire take_byte  = byte_valid && byte_ready;
wire store_byte = take_byte && !dropping_connect_line && !buffer_full;

reg stray_byte_done = 1'b0;                  /* the stray byte is put into the buffer in the first clock of the data mode */

always @(posedge clk) begin
    if (!reading_state) begin
        write_pointer   <= 0;
        read_pointer    <= 0;
        stray_byte_done <= 1'b0;
    end
    else if (!stray_byte_done) begin
        stray_byte_done <= 1'b1;
        if (stray_byte_valid) begin
            memory[0]     <= stray_byte;
            write_pointer <= 1;
        end
    end
    else begin
        if (store_byte) begin
            memory[write_pointer[12:0]] <= byte_in;
            write_pointer               <= write_pointer + 1'b1;
        end
        if (no_carrier_seen) write_pointer <= write_pointer - HELD_BACK_BYTES + (store_byte ? 1 : 0);      /* take the matched text back out */
        if (guest_valid && guest_ready) read_pointer <= read_pointer + 1'b1;
    end
end

reg [7:0] read_data = 8'd0;
/* registered read (block RAM); the read pointer moves only every few thousand clocks */
always @(posedge clk) read_data <= memory[read_pointer[12:0]];

assign guest_data = read_data;

/* ---- flow control */
reg asking = 1'b1;
always @(posedge clk) begin
    if (!reading_state)                        asking <= 1'b1;
    else if (bytes_held >= STOP_ASKING_AT)     asking <= 1'b0;
    else if (bytes_held <= ASK_AGAIN_BELOW)    asking <= 1'b1;
end

assign rx_ready_out = asking;

/* ---- quiet line: the last bytes are held back until nothing has arrived for 15 ms */
reg [19:0] quiet_left = 20'd0;
always @(posedge clk) begin
    if (take_byte && !dropping_connect_line) quiet_left <= QUIET_TIME;
    else if (quiet_left != 20'd0)            quiet_left <= quiet_left - 20'd1;
end

wire line_is_quiet = (quiet_left == 20'd0);
wire not_held_back = (bytes_held > HELD_BACK_BYTES) || line_is_quiet;

/* ---- delivery pace: one byte takes 10 bit times */
reg [19:0] pace_left = 20'd0;
reg  [19:0] byte_time;

always @* begin
    case (speed_code)
        2'd0:    byte_time = CLK_HZ / 30;           /* 300 baud */
        2'd1:    byte_time = CLK_HZ / 120;          /* 1200 baud */
        default: byte_time = CLK_HZ / 240;          /* 2400 baud */
    endcase
end

always @(posedge clk) begin
    if (rst || !en)                       pace_left <= 20'd0;
    else if (guest_valid && guest_ready)  pace_left <= byte_time;
    else if (pace_left != 20'd0)          pace_left <= pace_left - 20'd1;
end

/* not empty for two clocks: the read data has caught up with a fresh write */
reg [1:0] not_empty_history = 2'd0;
always @(posedge clk) not_empty_history <= {not_empty_history[0], !buffer_empty};

assign guest_valid = data_mode && carrier && guest_in_data_mode && !buffer_empty && not_empty_history[1] && not_held_back && (pace_left == 20'd0);

assign drained = buffer_empty && (pace_left == 20'd0);

endmodule
