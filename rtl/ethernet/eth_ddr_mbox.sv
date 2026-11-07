/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Ethernet frame mailbox between the LANCE (eth_board) and Main_MiSTer (FPGA only), in the DDR3 window that the Minimig A2065 and the Mac Ethernet
    code use (ARM physical 0x1FF00000 = Avalon word address 0x03FE0000). The FPGA moves whole frames; the HPS side (support/3b1/3b1_eth.cpp in
    Main_MiSTer) puts them on the network with the same raw socket helper as the other cores. Frames carry no FCS (the LANCE model appends and checks
    it itself).

    64 bit word index inside the window (byte offset = index * 8):
      0      magic "B31ETH01"                                              FPGA -> ARM, written at reset
      1      TX descriptor   [31:0] sequence, [47:32] length               FPGA -> ARM: a frame from the 3B1 for the network
      2      TX acknowledge  [31:0] sequence                               ARM -> FPGA: the frame with this sequence was taken
      3      RX descriptor   [31:0] sequence, [47:32] length               ARM -> FPGA: a frame from the network for the 3B1
      4      RX acknowledge  [31:0] sequence                               FPGA -> ARM: the frame with this sequence was taken
      5      station address [47:0] (first byte on the wire in bits 47:40) FPGA -> ARM, written at reset
      6      frame counters  [15:0] frames sent, [31:16] frames received, [47:32] sent frames dropped (previous one not yet taken)
      0x200  TX frame bytes (up to 1536, byte 0 in bits 7:0 of the first word)
      0x400  RX frame bytes
    One side writes the data first and the descriptor after it; the other side acknowledges by copying the sequence. Every sequence word has a single
    writer.

    Structure: a capture block collects the LANCE's transmit frame (one byte per valid strobe) into the transmit buffer; a DDR master
    (eth_ddr_word_master) does one 64 bit access at a time; a sequencer runs the protocol above: after reset it writes the identity words, then it
    publishes a captured frame, hands a fetched frame to the LANCE, or polls the RX descriptor.
*/
module eth_ddr_mbox
#(
    parameter [28:0] BASE = 29'h03FE0000,
    parameter [47:0] MAC  = 48'h023b10000001,
    parameter [19:0] POLL = 20'd4000                        /* clocks between looks at the RX descriptor (about 0.2 ms at 20 MHz) */
)
(
    input             clk,
    input             rst,

    /* LANCE frame streams (eth_board / lance.sv) */
    input             tx_vld,
    input      [7:0]  tx_dat,
    input             tx_end,
    output reg        rx_vld = 1'b0,
    output reg [7:0]  rx_dat = 8'd0,
    output reg        rx_end = 1'b0,
    input             rx_rdy,

    /* DDRAM (Avalon, 64 bit words), clocked with clk */
    input             ddr_busy,
    output [7:0]      ddr_burstcnt,
    output reg [28:0] ddr_addr = 29'd0,
    input      [63:0] ddr_dout,
    input             ddr_dready,
    output            ddr_rd,
    output reg [63:0] ddr_din = 64'd0,
    output [7:0]      ddr_be,
    output            ddr_we,

    output            activity
);

localparam [28:0] WORD_MAGIC            = 29'd0;
localparam [28:0] WORD_TX_DESCRIPTOR    = 29'd1;
localparam [28:0] WORD_TX_ACKNOWLEDGE   = 29'd2;
localparam [28:0] WORD_RX_DESCRIPTOR    = 29'd3;
localparam [28:0] WORD_RX_ACKNOWLEDGE   = 29'd4;
localparam [28:0] WORD_STATION_ADDRESS  = 29'd5;
localparam [28:0] WORD_COUNTERS         = 29'd6;
localparam [28:0] WORD_TX_FRAME         = 29'h200;
localparam [28:0] WORD_RX_FRAME         = 29'h400;
localparam [63:0] MAGIC                 = 64'h4231_3145_5448_3031;      /* "B31ETH01" */
localparam [11:0] MAXIMUM_FRAME_BYTES   = 12'd1536;

/* ---------------------------------------------------------------- frame buffers (block RAM, byte wide) */
reg        transmit_write = 1'b0;
reg [10:0] transmit_write_address = 11'd0;
reg [7:0]  transmit_write_data = 8'd0;
reg [10:0] transmit_read_address = 11'd0;
wire [7:0] transmit_read_data;
reg        receive_write = 1'b0;
reg [10:0] receive_write_address = 11'd0;
reg [7:0]  receive_write_data = 8'd0;
reg [10:0] receive_read_address = 11'd0;
wire [7:0] receive_read_data;

eth_frame_buffers frame_buffers
(
    .clk                    (clk),
    .transmit_write         (transmit_write),
    .transmit_write_address (transmit_write_address),
    .transmit_write_data    (transmit_write_data),
    .transmit_read_address  (transmit_read_address),
    .transmit_read_data     (transmit_read_data),
    .receive_write          (receive_write),
    .receive_write_address  (receive_write_address),
    .receive_write_data     (receive_write_data),
    .receive_read_address   (receive_read_address),
    .receive_read_data      (receive_read_data)
);

/* ---------------------------------------------------------------- frame capture from the LANCE */
reg [11:0] capture_count = 12'd0;                       /* bytes of the frame being captured */
reg [11:0] captured_length = 12'd0;                     /* length of the frame that waits to be published */
reg        capture_pending = 1'b0;                      /* a captured frame waits to be published */
/* the frame in progress is lost (the previous one is not published yet, or this one is too long) */
reg        capture_lost = 1'b0;
reg        capture_published = 1'b0;                    /* set by the sequencer: the pending frame was published */

reg [15:0] frames_sent = 16'd0;
reg [15:0] frames_received = 16'd0;
reg [15:0] frames_dropped = 16'd0;

wire capture_byte_fits = !capture_pending && !capture_lost && capture_count < MAXIMUM_FRAME_BYTES;

always @(posedge clk) begin
    transmit_write <= 1'b0;

    if (rst) begin
        capture_count   <= 12'd0;
        capture_pending <= 1'b0;
        capture_lost    <= 1'b0;
    end else begin
        if (capture_published)
            capture_pending <= 1'b0;

        if (tx_vld) begin
            if (capture_byte_fits) begin
                transmit_write         <= 1'b1;
                transmit_write_address <= capture_count[10:0];
                transmit_write_data    <= tx_dat;
            end else begin
                capture_lost <= 1'b1;
            end

            capture_count <= capture_count + 12'd1;

            if (tx_end) begin
                capture_count <= 12'd0;
                capture_lost  <= 1'b0;

                if (capture_byte_fits) begin
                    capture_pending <= 1'b1;
                    captured_length <= capture_count + 12'd1;
                end
            end
        end
    end
end

/* ---------------------------------------------------------------- DDR master: one 64 bit access at a time */
reg        write_request = 1'b0;
reg        read_request = 1'b0;
wire       write_starts;
wire       read_starts;
wire       access_done;
wire [63:0] read_data;
wire       shift_read_data;

eth_ddr_word_master ddr_master
(
    .clk             (clk),
    .rst             (rst),
    .write_request   (write_request),
    .read_request    (read_request),
    .shift_read_data (shift_read_data),
    .write_starts    (write_starts),
    .read_starts     (read_starts),
    .access_done     (access_done),
    .read_data       (read_data),
    .ddr_busy        (ddr_busy),
    .ddr_burstcnt    (ddr_burstcnt),
    .ddr_dout        (ddr_dout),
    .ddr_dready      (ddr_dready),
    .ddr_rd          (ddr_rd),
    .ddr_be          (ddr_be),
    .ddr_we          (ddr_we)
);

/* ---------------------------------------------------------------- sequencer */
typedef enum logic [4:0] {
    RESET_STATE                = 5'd0,
    /* the identity words: zero the words that belong to either side (the HPS starts its sequence numbers from what it reads) */
    INIT_CLEAR_TX_DESCRIPTOR   = 5'd1,
    INIT_CLEAR_TX_ACKNOWLEDGE  = 5'd2,
    INIT_CLEAR_RX_DESCRIPTOR   = 5'd3,
    INIT_CLEAR_RX_ACKNOWLEDGE  = 5'd4,
    INIT_WRITE_STATION_ADDRESS = 5'd5,
    INIT_CLEAR_COUNTERS        = 5'd6,
    INIT_WRITE_MAGIC           = 5'd7,   /* last, so the HPS sees the magic only when the rest is in place */
    IDLE                       = 5'd8,
    TX_READ_ACKNOWLEDGE        = 5'd9,   /* wait for the HPS to have taken the previous frame */
    TX_WRITE_COUNTERS          = 5'd10,   /* the counters, for the HPS side's log */
    TX_GATHER_WORD             = 5'd11,   /* gather eight bytes from the transmit buffer */
    TX_WRITE_WORD              = 5'd12,
    TX_WRITE_DESCRIPTOR        = 5'd13,   /* the descriptor goes last: the HPS looks at it first */
    RX_READ_DESCRIPTOR         = 5'd14,   /* a new sequence number means a frame is waiting */
    RX_READ_WORD               = 5'd15,   /* one data word */
    RX_UNPACK_WORD             = 5'd17,   /* write its eight bytes into the receive buffer */
    RX_WRITE_ACKNOWLEDGE       = 5'd18,   /* the frame is in our RAM: give the slot back */
    RX_STREAM_PRIME_0          = 5'd19,   /* stream the frame to the LANCE: the RAM output is one clock behind the address */
    RX_STREAM_PRIME_1          = 5'd20,
    RX_STREAM_BYTES            = 5'd21   /* one byte per clock */
} mailbox_state_t;

mailbox_state_t state = RESET_STATE;
reg        waiting_for_access = 1'b0;                   /* 0: issue the DDR access, 1: wait until it is done */
reg [31:0] transmit_sequence = 32'd0;
reg [31:0] receive_sequence_taken = 32'd0;
reg [31:0] receive_sequence_new = 32'd0;
reg [11:0] receive_length = 12'd0;
reg [11:0] receive_stream_position = 12'd0;
reg [11:0] publish_length = 12'd0;
reg        receive_frame_loaded = 1'b0;                 /* a frame sits in the receive buffer, waiting for the LANCE to be ready */
reg [8:0]  word_count = 9'd0;
reg [3:0]  byte_count = 4'd0;
reg [19:0] poll_clocks_left = 20'd0;

assign activity = (state != IDLE) | capture_pending | receive_frame_loaded;

/* while a received word is unpacked, the master's data register shifts down one byte per clock */
assign shift_read_data = (state == RX_UNPACK_WORD);

/* The address and the write data go straight to the DDR output registers (they are not used before the master asserts WE / RD). */
task issue_write(input [28:0] word_address, input [63:0] data);
    begin
        ddr_addr           <= BASE + word_address;
        ddr_din            <= data;
        write_request      <= 1'b1;
        waiting_for_access <= 1'b1;
    end
endtask

/* the same, with the data already assembled in ddr_din */
task issue_write_assembled(input [28:0] word_address);
    begin
        ddr_addr           <= BASE + word_address;
        write_request      <= 1'b1;
        waiting_for_access <= 1'b1;
    end
endtask

task issue_read(input [28:0] word_address);
    begin
        ddr_addr           <= BASE + word_address;
        read_request       <= 1'b1;
        waiting_for_access <= 1'b1;
    end
endtask

/* a step of the identity sequence: write one word, then go on to the next state */
task init_step(input [28:0] word_address, input [63:0] data, input mailbox_state_t next_state);
    begin
        if (!waiting_for_access) begin
            issue_write(word_address, data);
        end else if (access_done) begin
            waiting_for_access <= 1'b0;
            state              <= next_state;
        end
    end
endtask

always @(posedge clk) begin
    rx_vld            <= 1'b0;
    rx_end            <= 1'b0;
    receive_write     <= 1'b0;
    capture_published <= 1'b0;

    if (poll_clocks_left != 20'd0)
        poll_clocks_left <= poll_clocks_left - 20'd1;

    /* the DDR master took the request */
    if (write_starts)
        write_request <= 1'b0;

    if (read_starts)
        read_request <= 1'b0;

    case (state)
        /* ---- reset: tell the HPS who we are */
        RESET_STATE: begin
            transmit_sequence      <= 32'd0;
            receive_sequence_taken <= 32'd0;
            receive_frame_loaded   <= 1'b0;
            frames_sent            <= 16'd0;
            frames_received        <= 16'd0;
            frames_dropped         <= 16'd0;
            waiting_for_access     <= 1'b0;
            state                  <= INIT_CLEAR_TX_DESCRIPTOR;
        end

        INIT_CLEAR_TX_DESCRIPTOR:   init_step(WORD_TX_DESCRIPTOR,   64'd0,         INIT_CLEAR_TX_ACKNOWLEDGE);
        INIT_CLEAR_TX_ACKNOWLEDGE:  init_step(WORD_TX_ACKNOWLEDGE,  64'd0,         INIT_CLEAR_RX_DESCRIPTOR);
        INIT_CLEAR_RX_DESCRIPTOR:   init_step(WORD_RX_DESCRIPTOR,   64'd0,         INIT_CLEAR_RX_ACKNOWLEDGE);
        INIT_CLEAR_RX_ACKNOWLEDGE:  init_step(WORD_RX_ACKNOWLEDGE,  64'd0,         INIT_WRITE_STATION_ADDRESS);
        INIT_WRITE_STATION_ADDRESS: init_step(WORD_STATION_ADDRESS, {16'd0, MAC},  INIT_CLEAR_COUNTERS);
        INIT_CLEAR_COUNTERS:        init_step(WORD_COUNTERS,        64'd0,         INIT_WRITE_MAGIC);

        INIT_WRITE_MAGIC: begin
            if (!waiting_for_access) begin
                issue_write(WORD_MAGIC, MAGIC);
            end else if (access_done) begin
                waiting_for_access <= 1'b0;
                poll_clocks_left   <= POLL;
                state              <= IDLE;
            end
        end

        IDLE: begin
            if (capture_pending) begin
                /* publish the captured frame */
                waiting_for_access <= 1'b0;
                state              <= TX_READ_ACKNOWLEDGE;
            end else if (receive_frame_loaded && rx_rdy) begin
                /* hand the loaded frame to the LANCE */
                receive_stream_position <= 12'd0;
                receive_read_address    <= 11'd0;
                byte_count              <= 4'd0;
                state                   <= RX_STREAM_PRIME_0;
            end else if (poll_clocks_left == 20'd0 && !receive_frame_loaded) begin
                /* look at the RX descriptor */
                waiting_for_access <= 1'b0;
                state              <= RX_READ_DESCRIPTOR;
            end
        end

        /* ---- TX: wait for the HPS to have taken the previous frame, then write the data words, then the descriptor */
        TX_READ_ACKNOWLEDGE: begin
            if (!waiting_for_access) begin
                issue_read(WORD_TX_ACKNOWLEDGE);
            end else if (access_done) begin
                waiting_for_access <= 1'b0;

                if (read_data[31:0] == transmit_sequence) begin
                    word_count            <= 9'd0;
                    byte_count            <= 4'd0;
                    transmit_read_address <= 11'd0;
                    publish_length        <= captured_length;
                    state                 <= TX_GATHER_WORD;
                end else begin
                    /* the previous frame is still there: drop this one (and publish the counters) */
                    frames_dropped    <= frames_dropped + 16'd1;
                    capture_published <= 1'b1;
                    state             <= TX_WRITE_COUNTERS;
                    poll_clocks_left  <= POLL;
                end
            end
        end

        TX_GATHER_WORD: begin
            /* the RAM output is two clocks behind the address that is set here */
            if (byte_count <= 4'd7)
                transmit_read_address <= {word_count[7:0], 3'd0} + {7'd0, byte_count};

            if (byte_count >= 4'd2)
                ddr_din <= {transmit_read_data, ddr_din[63:8]};

            if (byte_count == 4'd9) begin
                byte_count         <= 4'd0;
                waiting_for_access <= 1'b0;
                state              <= TX_WRITE_WORD;
            end else begin
                byte_count <= byte_count + 4'd1;
            end
        end

        TX_WRITE_WORD: begin
            if (!waiting_for_access) begin
                issue_write_assembled(WORD_TX_FRAME + {20'd0, word_count});
            end else if (access_done) begin
                waiting_for_access <= 1'b0;
                word_count         <= word_count + 9'd1;

                if ({word_count, 3'd0} + 12'd8 >= publish_length) begin
                    capture_published <= 1'b1;                  /* all bytes are out of the RAM: it can take the next frame */
                    state             <= TX_WRITE_DESCRIPTOR;
                end else begin
                    byte_count <= 4'd0;
                    state      <= TX_GATHER_WORD;
                end
            end
        end

        TX_WRITE_DESCRIPTOR: begin
            if (!waiting_for_access) begin
                transmit_sequence <= transmit_sequence + 32'd1;
                issue_write(WORD_TX_DESCRIPTOR, {16'd0, 4'd0, publish_length, transmit_sequence + 32'd1});
            end else if (access_done) begin
                waiting_for_access <= 1'b0;
                frames_sent        <= frames_sent + 16'd1;
                poll_clocks_left   <= POLL;
                state              <= TX_WRITE_COUNTERS;
            end
        end

        TX_WRITE_COUNTERS: begin
            if (!waiting_for_access) begin
                issue_write(WORD_COUNTERS, {16'd0, frames_dropped, frames_received, frames_sent});
            end else if (access_done) begin
                waiting_for_access <= 1'b0;
                state              <= IDLE;
            end
        end

        /* ---- RX: read the descriptor; a new sequence number means a frame is waiting */
        RX_READ_DESCRIPTOR: begin
            if (!waiting_for_access) begin
                issue_read(WORD_RX_DESCRIPTOR);
            end else if (access_done) begin
                waiting_for_access <= 1'b0;
                poll_clocks_left   <= POLL;

                if (read_data[31:0] != receive_sequence_taken && read_data[47:32] != 16'd0 && read_data[47:32] <= {4'd0, MAXIMUM_FRAME_BYTES}) begin
                    receive_sequence_new <= read_data[31:0];
                    receive_length       <= read_data[43:32];
                    word_count           <= 9'd0;
                    state                <= RX_READ_WORD;
                end else begin
                    state <= IDLE;
                end
            end
        end

        RX_READ_WORD: begin
            if (!waiting_for_access) begin
                issue_read(WORD_RX_FRAME + {20'd0, word_count});
            end else if (access_done) begin
                waiting_for_access <= 1'b0;
                byte_count         <= 4'd0;
                state              <= RX_UNPACK_WORD;
            end
        end

        RX_UNPACK_WORD: begin
            /* one byte per clock into the receive buffer; the word is shifted out of the master's data register */
            receive_write         <= 1'b1;
            receive_write_address <= {word_count[7:0], byte_count[2:0]};
            receive_write_data    <= read_data[7:0];
            byte_count            <= byte_count + 4'd1;

            if (byte_count == 4'd7) begin
                word_count <= word_count + 9'd1;

                if ({word_count, 3'd0} + 12'd8 >= receive_length) begin
                    waiting_for_access <= 1'b0;
                    state              <= RX_WRITE_ACKNOWLEDGE;
                end else begin
                    waiting_for_access <= 1'b0;
                    state              <= RX_READ_WORD;
                end
            end
        end

        RX_WRITE_ACKNOWLEDGE: begin
            if (!waiting_for_access) begin
                issue_write(WORD_RX_ACKNOWLEDGE, {32'd0, receive_sequence_new});
            end else if (access_done) begin
                waiting_for_access     <= 1'b0;
                receive_sequence_taken <= receive_sequence_new;
                receive_frame_loaded   <= 1'b1;
                frames_received        <= frames_received + 16'd1;
                state                  <= IDLE;
            end
        end

        /* ---- RX: stream the frame to the LANCE (one byte per clock; the RAM output is one clock behind the address) */
        RX_STREAM_PRIME_0: begin
            receive_read_address <= 11'd0;
            state                <= RX_STREAM_PRIME_1;
        end

        RX_STREAM_PRIME_1: begin
            receive_read_address <= 11'd1;
            state                <= RX_STREAM_BYTES;
        end

        RX_STREAM_BYTES: begin
            rx_vld                  <= 1'b1;
            rx_dat                  <= receive_read_data;
            rx_end                  <= (receive_stream_position + 12'd1 == receive_length);
            receive_stream_position <= receive_stream_position + 12'd1;
            receive_read_address    <= receive_read_address + 11'd1;

            if (receive_stream_position + 12'd1 == receive_length) begin
                receive_frame_loaded <= 1'b0;
                state                <= IDLE;
            end
        end

        default: state <= RESET_STATE;
    endcase

    if (rst) begin
        state              <= RESET_STATE;
        waiting_for_access <= 1'b0;
        write_request      <= 1'b0;
        read_request       <= 1'b0;
    end
end

endmodule
