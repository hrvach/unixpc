/*
    Copyright (c) 2026 by Hrvoje Cavrak

    AMD Am7990 LANCE (Local Area Network Controller for Ethernet), behavioural model. Ethernet expansion board, see eth_board.sv.

    What is modelled: the register interface (CSR0..CSR3 through RAP / RDP), the initialisation block, the transmit and receive descriptor rings, DMA
    through a word wide master port (lance_bus_master.sv), the CRC-32, receive address filtering (physical address, broadcast, logical address hash,
    promiscuous mode) and loopback.

    Frames travel as byte streams, one byte per 16 clocks (10 Mbit/s), to and from a "PHY" (the simulation harness, or a MAC in a real build). The
    streams carry NO FCS: on receive the LANCE appends the computed FCS, so that the descriptor's message count and the buffer contents look like the
    real thing; on transmit the FCS is only generated for loopback.

    Not modelled: collisions and retries, babble, SQE test, late collision, FIFO overflow, deferred transmission, the timing details of CSR0 TDMD.
    Transmit descriptors are polled every 1.6 ms (and on a transmit demand), as on the real part.

    Byte order: descriptor and initialisation block words are natural 16 bit big endian values in memory, whatever BSWP says (the WIN/3B driver's
    enet.h defines the RMD1 / TMD1 fields as plain ushort masks, OWN = 0x8000, and sets BSWP = 1). BSWP only decides the byte order of the packet
    data: with BSWP set the byte at the even address is the first byte of the frame (network order). "LANCE word" means the word as the LANCE sees it:
    data words are swapped, descriptor words are not.

    Descriptors (4 words of 16 bits, 8 bytes in the ring):
      word 0  buffer address bits 15..0
      word 1  flags (D15 OWN: the LANCE owns it, D9 STP: start of packet, D8 ENP: end of packet, receive: D14 ERR, D12 CRC)
              and buffer address bits 23..16 in the low byte
      word 2  buffer byte count as a negative 12 bit number (the upper four bits read 1)
      word 3  receive: message count of the whole frame; transmit: status (error bits)
*/
module lance
import ethernet_pkg::*;
(
    input         clk,
    input         rst,                  /* RESET pin: acts like STOP */

    /* slave side (the board decodes RDP / RAP and does the two step read) */
    input         rap_we,               /* write RAP */
    input         rdp_we,               /* write the CSR selected by RAP */
    input  [15:0] wdata,
    output [15:0] rap_q,
    output [15:0] csr_q,                /* the CSR selected by RAP */
    output        intr,                 /* INTR & INEA */
    input  [6:0]  csr0_clear,           /* the board's interrupt acknowledge (same as writing ones to IDON..BABL) */
    output [15:0] csr0_q,

    /* DMA master (word port; the board arbitrates the bus) */
    output        br,                   /* bus request, held while the LANCE has work for the memory */
    input         bg,                   /* bus granted */
    output        m_req,
    output        m_we,
    output [23:1] m_addr,
    output [15:0] m_wdata,
    output [1:0]  m_be,                 /* bus byte enables: bit 1 = D15..D8 */
    input         m_done,
    input         m_fault,
    input  [15:0] m_rdata,

    /* frame stream to the wire */
    output reg    tx_vld = 1'b0,
    output reg [7:0] tx_dat = 8'd0,
    output reg    tx_end = 1'b0,
    /* frame stream from the wire */
    input         rx_vld,
    input  [7:0]  rx_dat,
    input         rx_end,
    output        rx_rdy                /* idle: a frame may start */
);

/* ---------------------------------------------------------------- CSR0..CSR3 */
reg [1:0]  rap = 2'd0;                                  /* register address pointer */
reg        csr0_stop = 1'b1;
reg        csr0_strt = 1'b0;
reg        csr0_init = 1'b0;
reg        csr0_inea = 1'b0;                            /* interrupt enable */
reg        csr0_txon = 1'b0;
reg        csr0_rxon = 1'b0;
reg        start_after_init = 1'b0;                     /* INIT and STRT were written together: start once the initialisation is done */
reg        csr0_idon = 1'b0;                            /* initialisation done */
reg        csr0_tint = 1'b0;                            /* transmitter interrupt */
reg        csr0_rint = 1'b0;                            /* receiver interrupt */
reg        csr0_merr = 1'b0;                            /* memory error */
reg        csr0_miss = 1'b0;                            /* missed packet */
reg        csr0_cerr = 1'b0;                            /* collision error (never set) */
reg        csr0_babl = 1'b0;                            /* babble (never set) */
reg [15:0] csr1 = 16'd0;                                /* initialisation block address, bits 15..1 */
reg [7:0]  csr2 = 8'd0;                                 /* initialisation block address, bits 23..16 */
reg [2:0]  csr3 = 3'd0;                                 /* BSWP, ACON, BCON */
wire       byte_swap = csr3[2];

wire csr0_err  = csr0_babl | csr0_cerr | csr0_miss | csr0_merr;
wire csr0_intr = csr0_babl | csr0_miss | csr0_merr | csr0_rint | csr0_tint | csr0_idon;

assign csr0_q = {csr0_err, csr0_babl, csr0_cerr, csr0_miss, csr0_merr, csr0_rint, csr0_tint, csr0_idon,
                 csr0_intr, csr0_inea, csr0_rxon, csr0_txon, 1'b0, csr0_stop, csr0_strt, csr0_init};
assign intr   = csr0_intr & csr0_inea;
assign rap_q  = {14'd0, rap};

reg [15:0] selected_csr;

always @* begin
    case (rap)
        2'd0:    selected_csr = csr0_q;
        2'd1:    selected_csr = {csr1[15:1], 1'b0};
        2'd2:    selected_csr = {8'd0, csr2};
        default: selected_csr = {13'd0, csr3};
    endcase
end

assign csr_q = selected_csr;

/* ---------------------------------------------------------------- initialisation block, as read by INIT */
reg [15:0] mode_register       = 16'd0;
reg [47:0] physical_address    = 48'd0;
reg [63:0] logical_address_filter = 64'd0;
reg [23:0] receive_ring_base   = 24'd0;
reg [23:0] transmit_ring_base  = 24'd0;
reg [2:0]  receive_ring_length_code  = 3'd0;            /* the ring has 2 ** code descriptors */
reg [2:0]  transmit_ring_length_code = 3'd0;

wire mode_disable_receiver        = mode_register[0];   /* DRX */
wire mode_disable_transmitter     = mode_register[1];   /* DTX */
wire mode_loopback                = mode_register[2];   /* LOOP */
wire mode_disable_transmit_crc    = mode_register[3];   /* DTCR */
wire mode_promiscuous             = mode_register[15];  /* PROM */

/* ---------------------------------------------------------------- frame buffers */
reg [10:0] transmit_write_address = 11'd0;
reg [10:0] receive_write_address  = 11'd0;
reg [10:0] receive_read_address   = 11'd0;
reg [7:0]  transmit_write_data    = 8'd0;
reg [7:0]  receive_write_data     = 8'd0;
reg        transmit_write         = 1'b0;
reg        receive_write          = 1'b0;
wire [7:0] transmit_read_data;
wire [7:0] receive_read_data;

reg [7:0]  destination_byte [0:5];      /* the first six bytes of the received frame (destination address), captured as they are written */
reg [11:0] transmit_frame_length;       /* bytes in the transmit buffer (padded) */
reg [11:0] receive_frame_length;        /* bytes in the receive buffer (with the FCS once appended) */

/* the transmit buffer is read at transmit_byte_position (the byte of the address of one clock ago) and the receive buffer at receive_read_address */
reg [11:0] transmit_byte_position = 12'd0;

eth_frame_buffers frame_buffers
(
    .clk                    (clk),
    .transmit_write         (transmit_write),
    .transmit_write_address (transmit_write_address),
    .transmit_write_data    (transmit_write_data),
    .transmit_read_address  (transmit_byte_position[10:0]),
    .transmit_read_data     (transmit_read_data),
    .receive_write          (receive_write),
    .receive_write_address  (receive_write_address),
    .receive_write_data     (receive_write_data),
    .receive_read_address   (receive_read_address),
    .receive_read_data      (receive_read_data)
);

/* ---------------------------------------------------------------- receive address filter */
wire [47:0] destination_address = {destination_byte[5], destination_byte[4], destination_byte[3],
                                    destination_byte[2], destination_byte[1], destination_byte[0]};
wire        frame_accepted;

lance_address_filter address_filter
(
    .clk                    (clk),
    .destination_address    (destination_address),
    .physical_address       (physical_address),
    .logical_address_filter (logical_address_filter),
    .mode_promiscuous       (mode_promiscuous),
    .mode_loopback          (mode_loopback),
    .frame_accepted         (frame_accepted)
);

/* ---------------------------------------------------------------- sequencer */
typedef enum logic [4:0] {
    IDLE                      = 5'd0,
    READ_INITIALISATION_BLOCK = 5'd1,
    TX_READ_FLAGS             = 5'd2,   /* TMD1: owner, start / end of packet, buffer address high */
    TX_READ_ADDRESS_LOW       = 5'd3,   /* TMD0 */
    TX_READ_COUNT             = 5'd4,   /* TMD2 */
    TX_GATHER_DATA            = 5'd5,   /* copy the buffer into the transmit buffer */
    TX_PAD                    = 5'd6,   /* pad the frame to the minimum size */
    TX_SEND                   = 5'd7,   /* stream it to the wire (or through the CRC generator into the receiver) */
    TX_WRITE_BACK_FLAGS       = 5'd8,   /* give the descriptors back: OWN = 0, STP on the first, ENP on the last */
    TX_WRITE_BACK_STATUS      = 5'd9,   /* TMD3: no errors */
    TX_NEXT_DESCRIPTOR        = 5'd10,
    RX_PREPARE                = 5'd11,   /* pad, FCS, address filter */
    RX_READ_FLAGS             = 5'd12,   /* RMD1 */
    RX_READ_ADDRESS_LOW       = 5'd13,   /* RMD0 */
    RX_READ_COUNT             = 5'd14,   /* RMD2 */
    RX_STORE_DATA             = 5'd15,   /* copy the frame into the buffer */
    RX_WRITE_BACK_FLAGS       = 5'd16,   /* RMD1: OWN = 0, ERR / CRC, STP on the first buffer, ENP on the last */
    RX_WRITE_BACK_STATUS      = 5'd17   /* RMD3: message count */
} lance_state_t;

lance_state_t state = IDLE;
reg        waiting_for_access = 1'b0;                   /* 0: issue the memory access, 1: wait until it is finished */

/* the memory access asked for from lance_bus_master */
reg        access_pending = 1'b0;
reg        access_write = 1'b0;
reg [23:1] access_address = 23'd0;
reg [15:0] access_write_data = 16'd0;
reg        access_is_packet_data = 1'b0;                /* packet data: the byte order follows BSWP */
reg [1:0]  access_byte_enables = 2'b11;                 /* bit 0 = low byte of the LANCE word */
wire       access_starts;
wire       access_finished;
wire [15:0] access_read_data;
wire       access_error;

lance_bus_master bus_master
(
    .clk                   (clk),
    .rst                   (rst),
    .byte_swap             (byte_swap),
    .access_pending        (access_pending),
    .access_write          (access_write),
    .access_address        (access_address),
    .access_write_data     (access_write_data),
    .access_is_packet_data (access_is_packet_data),
    .access_byte_enables   (access_byte_enables),
    .access_starts         (access_starts),
    .bus_request           (br),
    .access_finished       (access_finished),
    .access_read_data      (access_read_data),
    .access_error          (access_error),
    .bus_granted           (bg),
    .m_req                 (m_req),
    .m_we                  (m_we),
    .m_addr                (m_addr),
    .m_wdata               (m_wdata),
    .m_be                  (m_be),
    .m_done                (m_done),
    .m_fault               (m_fault),
    .m_rdata               (m_rdata)
);

reg [4:0]  init_word_index = 5'd0;                      /* which word of the initialisation block is read */
reg [7:0]  transmit_ring_index = 8'd0;
reg [7:0]  receive_ring_index = 8'd0;
reg [23:1] transmit_descriptor = 23'd0;                 /* word address of the current descriptor */
reg [23:1] receive_descriptor = 23'd0;
reg [15:0] descriptor_word0 = 16'd0;
reg [15:0] descriptor_word1 = 16'd0;
reg [23:0] buffer_address = 24'd0;                      /* byte address of the current buffer */
reg [11:0] buffer_byte_count = 12'd0;
reg [11:0] buffer_byte_index = 12'd0;                   /* byte index within the buffer */
reg [3:0]  transmit_buffer_count = 4'd0;                /* buffers of this transmit frame */
reg [3:0]  write_back_index = 4'd0;                     /* descriptors handed back so far */
/* the upper address byte of each buffer of the frame (it is the low byte of TMD1, written back as it was) */
reg [7:0]  transmit_buffer_address_high [0:7];
reg [7:0]  transmit_ring_slot [0:7];                    /* the ring index of each buffer of the frame */
reg        transmit_first_buffer = 1'b1;
reg [31:0] poll_counter = 32'd0;
reg        transmit_demand = 1'b0;
reg [4:0]  transmit_byte_clock = 5'd0;                  /* 16 clocks per byte on the wire */
reg [31:0] transmit_crc = 32'd0;

reg        receiving_from_wire = 1'b0;                  /* a frame is arriving from the wire */
reg        frame_ready = 1'b0;                          /* a whole frame sits in the receive buffer */
reg        frame_has_fcs = 1'b0;                        /* it came from the transmitter (loopback) and already has its FCS */
reg [11:0] wire_write_position = 12'd0;
reg [11:0] receive_position = 12'd0;                    /* bytes of the frame stored into the ring so far */
reg [31:0] receive_crc = 32'd0;
reg [2:0]  prepare_phase = 3'd0;
reg        crc_error = 1'b0;
reg        receive_start_of_packet = 1'b1;

assign rx_rdy = ~receiving_from_wire & ~frame_ready & csr0_rxon & ~mode_disable_receiver & (state == IDLE) & ~csr0_stop;

wire [23:0] init_block_address = {csr2, csr1[15:1], 1'b0};
wire [7:0]  transmit_ring_mask = (8'd1 << transmit_ring_length_code) - 8'd1;
wire [7:0]  receive_ring_mask  = (8'd1 << receive_ring_length_code)  - 8'd1;
wire [23:0] next_byte_address  = buffer_address + {12'd0, buffer_byte_index};
wire        last_buffer        = descriptor_word1[8];                   /* ENP */
wire        frame_stored       = (receive_position >= receive_frame_length);

/* The descriptors are 8 bytes, so the ring index counts in units of four words. */
function [23:1] ring_address(input [23:0] base, input [7:0] index);
    ring_address = base[23:1] + {15'd0, index, 2'b00};
endfunction

/* A word of the frame is 16 bits; the access tasks start one memory access and then wait for it (waiting_for_access). */
task read_word(input [23:1] word_address);
    begin
        access_address        <= word_address;
        access_write          <= 1'b0;
        access_is_packet_data <= 1'b0;
        access_pending        <= 1'b1;
        waiting_for_access    <= 1'b1;
    end
endtask

task write_word(input [23:1] word_address, input [15:0] data, input [1:0] byte_enables);
    begin
        access_address        <= word_address;
        access_write          <= 1'b1;
        access_write_data     <= data;
        access_byte_enables   <= byte_enables;
        access_is_packet_data <= 1'b0;
        access_pending        <= 1'b1;
        waiting_for_access    <= 1'b1;
    end
endtask

task read_data_word(input [23:1] word_address);
    begin
        access_address        <= word_address;
        access_write          <= 1'b0;
        access_is_packet_data <= 1'b1;
        access_pending        <= 1'b1;
        waiting_for_access    <= 1'b1;
    end
endtask

task write_data_word(input [23:1] word_address, input [15:0] data, input [1:0] byte_enables);
    begin
        access_address        <= word_address;
        access_write          <= 1'b1;
        access_write_data     <= data;
        access_byte_enables   <= byte_enables;
        access_is_packet_data <= 1'b1;
        access_pending        <= 1'b1;
        waiting_for_access    <= 1'b1;
    end
endtask

/*
    Frame buffer writes: the byte lands in the RAM one clock after the request. The first six bytes of a received frame are also kept for the address
    filter.
*/
task write_transmit_byte(input [10:0] address, input [7:0] data);
    begin
        transmit_write         <= 1'b1;
        transmit_write_address <= address;
        transmit_write_data    <= data;
    end
endtask

task write_receive_byte(input [10:0] address, input [7:0] data);
    begin
        receive_write         <= 1'b1;
        receive_write_address <= address;
        receive_write_data    <= data;

        if (address < 11'd6)
            destination_byte[address[2:0]] <= data;
    end
endtask

reg        transmit_second_byte_pending = 1'b0;         /* second byte of a word copy, written one clock after the first */
reg [10:0] transmit_second_byte_address = 11'd0;
reg [7:0]  transmit_second_byte_data = 8'd0;
reg [2:0]  fcs_bytes_left = 3'd0;                       /* FCS bytes still to be written into the receive buffer, one per clock */
reg [10:0] fcs_write_address = 11'd0;
reg [31:0] fcs_shifter = 32'd0;
reg [1:0]  crc_read_phase = 2'd0;                       /* read phase of the receive CRC loop */
reg [2:0]  store_read_phase = 3'd0;                     /* read phase of the receive word builder */
reg [7:0]  store_low_byte = 8'd0;
reg [11:0] write_position;                              /* where the byte from the wire goes (blocking temporary) */

/* the minimum frame without FCS is 60 bytes; frames above 2040 bytes are cut (the buffer holds 2 KB with room for the FCS) */
localparam [11:0] MINIMUM_FRAME_BYTES    = 12'd60;
localparam [11:0] MAXIMUM_STORED_BYTES   = 12'd2040;
localparam [31:0] POLL_INTERVAL_CLOCKS   = 32'd32000;   /* 1.6 ms at 20 MHz */
localparam [31:0] CRC_GOOD_FRAME_RESIDUE = 32'hdebb20e3;

always @(posedge clk) begin
    transmit_write <= 1'b0;
    receive_write  <= 1'b0;

    if (transmit_second_byte_pending) begin
        transmit_write         <= 1'b1;
        transmit_write_address <= transmit_second_byte_address;
        transmit_write_data    <= transmit_second_byte_data;
        transmit_second_byte_pending <= 1'b0;
    end

    if (fcs_bytes_left != 3'd0) begin
        write_receive_byte(fcs_write_address, fcs_shifter[7:0]);
        fcs_write_address <= fcs_write_address + 11'd1;
        fcs_shifter       <= {8'd0, fcs_shifter[31:8]};
        fcs_bytes_left    <= fcs_bytes_left - 3'd1;
    end

    tx_vld <= 1'b0;
    tx_end <= 1'b0;
    poll_counter <= poll_counter + 32'd1;

    /* ---- the memory access asked for has got the bus */
    if (access_starts)
        access_pending <= 1'b0;

    /* ---- frame from the wire into the receive buffer */
    if (rx_vld && (receiving_from_wire || rx_rdy)) begin
        write_position = receiving_from_wire ? wire_write_position : 12'd0;

        if (write_position < MAXIMUM_STORED_BYTES)
            write_receive_byte(write_position[10:0], rx_dat);

        if (rx_end) begin
            receiving_from_wire  <= 1'b0;
            frame_ready          <= 1'b1;
            frame_has_fcs        <= 1'b0;
            receive_frame_length <= (write_position < MAXIMUM_STORED_BYTES) ? write_position + 12'd1 : write_position;
        end else begin
            receiving_from_wire  <= 1'b1;
            wire_write_position  <= (write_position < MAXIMUM_STORED_BYTES) ? write_position + 12'd1 : write_position;
        end
    end

    /* ---- CSR writes from the CPU. The board's interrupt acknowledge clears the same status bits as writing ones to them. */
    if (rap_we)
        rap <= wdata[1:0];

    if (csr0_clear[0]) csr0_idon <= 1'b0;
    if (csr0_clear[1]) csr0_tint <= 1'b0;
    if (csr0_clear[2]) csr0_rint <= 1'b0;
    if (csr0_clear[3]) csr0_merr <= 1'b0;
    if (csr0_clear[4]) csr0_miss <= 1'b0;
    if (csr0_clear[5]) csr0_cerr <= 1'b0;
    if (csr0_clear[6]) csr0_babl <= 1'b0;

    if (rdp_we) begin
        case (rap)
            2'd0: begin
                if (wdata[14]) csr0_babl <= 1'b0;
                if (wdata[13]) csr0_cerr <= 1'b0;
                if (wdata[12]) csr0_miss <= 1'b0;
                if (wdata[11]) csr0_merr <= 1'b0;
                if (wdata[10]) csr0_rint <= 1'b0;
                if (wdata[9])  csr0_tint <= 1'b0;
                if (wdata[8])  csr0_idon <= 1'b0;

                csr0_inea <= wdata[6];

                if (wdata[2]) begin
                    /* STOP: everything stops, all status clears */
                    csr0_stop          <= 1'b1;
                    start_after_init   <= 1'b0;
                    csr0_strt          <= 1'b0;
                    csr0_init          <= 1'b0;
                    csr0_txon          <= 1'b0;
                    csr0_rxon          <= 1'b0;
                    csr0_inea          <= 1'b0;
                    csr0_idon          <= 1'b0;
                    csr0_tint          <= 1'b0;
                    csr0_rint          <= 1'b0;
                    csr0_merr          <= 1'b0;
                    csr0_miss          <= 1'b0;
                    csr0_cerr          <= 1'b0;
                    csr0_babl          <= 1'b0;
                    state              <= IDLE;
                    waiting_for_access <= 1'b0;
                    frame_ready        <= 1'b0;
                    receiving_from_wire <= 1'b0;
                    transmit_demand    <= 1'b0;
                end else if (wdata[0] && csr0_stop) begin
                    /* INIT (needs STOP): read the initialisation block */
                    csr0_stop          <= 1'b0;
                    csr0_init          <= 1'b1;
                    init_word_index    <= 5'd0;
                    waiting_for_access <= 1'b0;
                    state              <= READ_INITIALISATION_BLOCK;
                    start_after_init   <= wdata[1];
                end else if (wdata[1] && !csr0_stop) begin
                    /* STRT: start the receiver and transmitter as the mode register allows */
                    csr0_strt            <= 1'b1;
                    csr0_rxon            <= ~mode_disable_receiver;
                    csr0_txon            <= ~mode_disable_transmitter;
                    transmit_ring_index  <= 8'd0;
                    receive_ring_index   <= 8'd0;
                    poll_counter         <= 32'd0;
                end

                if (wdata[3] && csr0_txon)
                    transmit_demand <= 1'b1;
            end

            2'd1: csr1 <= wdata;
            2'd2: csr2 <= wdata[7:0];
            2'd3: csr3 <= wdata[2:0];
        endcase
    end

    /* ---- the sequencer */
    case (state)
        IDLE: begin
            if (!csr0_stop) begin
                if (frame_ready && csr0_rxon) begin
                    state         <= RX_PREPARE;
                    prepare_phase <= 3'd0;
                end else if (csr0_txon && (transmit_demand || poll_counter >= POLL_INTERVAL_CLOCKS)) begin
                    transmit_demand       <= 1'b0;
                    poll_counter          <= 32'd0;
                    transmit_buffer_count <= 4'd0;
                    transmit_byte_position <= 12'd0;
                    transmit_first_buffer <= 1'b1;
                    transmit_descriptor   <= ring_address(transmit_ring_base, transmit_ring_index);
                    waiting_for_access    <= 1'b0;
                    state                 <= TX_READ_FLAGS;
                end
            end
        end

        /* ---------------- initialisation block: 12 single word reads */
        READ_INITIALISATION_BLOCK: begin
            if (!waiting_for_access) begin
                read_word(init_block_address[23:1] + {18'd0, init_word_index});
            end else if (access_finished) begin
                waiting_for_access <= 1'b0;

                if (access_error) begin
                    csr0_merr <= 1'b1;
                    csr0_stop <= 1'b1;
                    csr0_init <= 1'b0;
                    state     <= IDLE;
                end else begin
                    case (init_word_index)
                        5'd0:  mode_register <= access_read_data;
                        5'd1:  physical_address[15:0] <= access_read_data;
                        5'd2:  physical_address[31:16] <= access_read_data;
                        5'd3:  physical_address[47:32] <= access_read_data;
                        5'd4:  logical_address_filter[15:0] <= access_read_data;
                        5'd5:  logical_address_filter[31:16] <= access_read_data;
                        5'd6:  logical_address_filter[47:32] <= access_read_data;
                        5'd7:  logical_address_filter[63:48] <= access_read_data;
                        5'd8:  receive_ring_base[15:0] <= access_read_data;
                        5'd9: begin
                            receive_ring_base[23:16]  <= access_read_data[7:0];
                            receive_ring_length_code  <= access_read_data[15:13];
                        end
                        5'd10: transmit_ring_base[15:0] <= access_read_data;
                        5'd11: begin
                            transmit_ring_base[23:16] <= access_read_data[7:0];
                            transmit_ring_length_code <= access_read_data[15:13];
                        end
                        default: ;
                    endcase

                    if (init_word_index == 5'd11) begin
                        csr0_idon <= 1'b1;                      /* INIT stays set until STOP */
                        state     <= IDLE;

                        if (start_after_init) begin             /* STRT was written with INIT: start now */
                            start_after_init     <= 1'b0;
                            csr0_strt            <= 1'b1;
                            csr0_rxon            <= ~mode_disable_receiver;
                            csr0_txon            <= ~mode_disable_transmitter;
                            transmit_ring_index  <= 8'd0;
                            receive_ring_index   <= 8'd0;
                            poll_counter         <= 32'd0;
                        end
                    end

                    init_word_index <= init_word_index + 5'd1;
                end
            end
        end

        /* ---------------- transmit: read the descriptors, gather the buffers into the transmit buffer */
        TX_READ_FLAGS: begin
            if (!waiting_for_access) begin
                read_word(transmit_descriptor + 23'd1);
            end else if (access_finished) begin
                waiting_for_access <= 1'b0;
                descriptor_word1   <= access_read_data;

                if (access_error) begin
                    csr0_merr <= 1'b1;
                    state     <= IDLE;
                end else if (!access_read_data[15]) begin
                    state <= IDLE;                              /* not owned by the LANCE */
                end else if (transmit_first_buffer && !access_read_data[9]) begin
                    state <= TX_NEXT_DESCRIPTOR;                /* not a start of packet: skip */
                end else begin
                    state <= TX_READ_ADDRESS_LOW;
                end
            end
        end

        TX_READ_ADDRESS_LOW: begin
            if (!waiting_for_access) begin
                read_word(transmit_descriptor);
            end else if (access_finished) begin
                waiting_for_access <= 1'b0;
                descriptor_word0   <= access_read_data;
                state              <= TX_READ_COUNT;
            end
        end

        TX_READ_COUNT: begin
            if (!waiting_for_access) begin
                read_word(transmit_descriptor + 23'd2);
            end else if (access_finished) begin
                waiting_for_access <= 1'b0;
                buffer_address     <= {descriptor_word1[7:0], descriptor_word0};
                buffer_byte_count  <= -access_read_data[11:0];          /* 12 bit counter: 4096 wraps to 0 */
                buffer_byte_index  <= 12'd0;
                transmit_buffer_address_high[transmit_buffer_count[2:0]] <= descriptor_word1[7:0];
                transmit_ring_slot[transmit_buffer_count[2:0]]  <= transmit_ring_index;
                transmit_buffer_count <= transmit_buffer_count + 4'd1;
                state              <= TX_GATHER_DATA;
            end
        end

        TX_GATHER_DATA: begin
            if (buffer_byte_index >= buffer_byte_count) begin
                transmit_first_buffer <= 1'b0;

                if (last_buffer || transmit_buffer_count == 4'd8) begin
                    state <= TX_PAD;                            /* ENP: the frame is complete (or the buffer table is full) */
                end else begin
                    transmit_ring_index <= (transmit_ring_index + 8'd1) & transmit_ring_mask;
                    transmit_descriptor <= ring_address(transmit_ring_base, (transmit_ring_index + 8'd1) & transmit_ring_mask);
                    state               <= TX_READ_FLAGS;
                end
            end else if (!waiting_for_access) begin
                read_data_word(next_byte_address[23:1]);
            end else if (access_finished) begin : copy_to_transmit_buffer
                reg [11:0] bytes_copied;

                waiting_for_access <= 1'b0;

                if (access_error) begin
                    csr0_merr <= 1'b1;
                    state     <= IDLE;
                end else begin
                    if (next_byte_address[0]) begin
                        /* odd start: only the upper byte of the word belongs to the buffer */
                        if (transmit_byte_position < MAXIMUM_STORED_BYTES)
                            write_transmit_byte(transmit_byte_position[10:0], access_read_data[15:8]);

                        bytes_copied = 12'd1;
                    end else begin
                        if (transmit_byte_position < MAXIMUM_STORED_BYTES)
                            write_transmit_byte(transmit_byte_position[10:0], access_read_data[7:0]);

                        if (buffer_byte_index + 12'd1 < buffer_byte_count) begin
                            /* the upper byte is written one clock later */
                            if (transmit_byte_position < MAXIMUM_STORED_BYTES - 12'd1) begin
                                transmit_second_byte_pending <= 1'b1;
                                transmit_second_byte_address <= transmit_byte_position[10:0] + 11'd1;
                                transmit_second_byte_data    <= access_read_data[15:8];
                            end

                            bytes_copied = 12'd2;
                        end else begin
                            bytes_copied = 12'd1;
                        end
                    end

                    buffer_byte_index      <= buffer_byte_index + bytes_copied;
                    transmit_byte_position <= transmit_byte_position + bytes_copied;
                end
            end
        end

        TX_PAD: begin
            if (transmit_byte_position < MINIMUM_FRAME_BYTES) begin
                write_transmit_byte(transmit_byte_position[10:0], 8'h00);
                transmit_byte_position <= transmit_byte_position + 12'd1;
            end else begin
                transmit_frame_length  <= transmit_byte_position;
                transmit_byte_position <= 12'd0;
                transmit_byte_clock    <= 5'd0;
                transmit_crc           <= 32'hffffffff;
                state                  <= TX_SEND;
            end
        end

        TX_SEND: begin
            /* one byte per 16 clocks */
            transmit_byte_clock <= transmit_byte_clock + 5'd1;

            if (transmit_byte_clock == 5'd15) begin
                transmit_byte_clock <= 5'd0;

                if (mode_loopback) begin
                    /* loopback: the frame goes through the CRC generator into the receiver */
                    if (transmit_byte_position < transmit_frame_length) begin
                        write_receive_byte(transmit_byte_position[10:0], transmit_read_data);
                        transmit_crc           <= crc_after_byte(transmit_crc, transmit_read_data);
                        transmit_byte_position <= transmit_byte_position + 12'd1;
                    end else begin
                        if (!mode_disable_transmit_crc) begin
                            fcs_bytes_left       <= 3'd4;                           /* FCS, LSB first, one byte per clock */
                            fcs_write_address    <= transmit_frame_length[10:0];
                            fcs_shifter          <= ~transmit_crc;
                            receive_frame_length <= transmit_frame_length + 12'd4;
                        end else begin
                            receive_frame_length <= transmit_frame_length;
                        end

                        if (csr0_rxon) begin
                            frame_ready   <= 1'b1;
                            frame_has_fcs <= 1'b1;
                        end

                        write_back_index   <= 4'd0;
                        waiting_for_access <= 1'b0;
                        state              <= TX_WRITE_BACK_FLAGS;
                    end
                end else begin
                    if (transmit_byte_position < transmit_frame_length) begin
                        tx_vld                 <= 1'b1;
                        tx_dat                 <= transmit_read_data;
                        tx_end                 <= (transmit_byte_position + 12'd1 == transmit_frame_length);
                        transmit_byte_position <= transmit_byte_position + 12'd1;
                    end else begin
                        write_back_index   <= 4'd0;
                        waiting_for_access <= 1'b0;
                        state              <= TX_WRITE_BACK_FLAGS;
                    end
                end
            end
        end

        TX_WRITE_BACK_FLAGS: begin
            if (write_back_index >= transmit_buffer_count) begin
                csr0_tint <= 1'b1;
                state     <= TX_NEXT_DESCRIPTOR;
            end else if (!waiting_for_access) begin
                write_word(ring_address(transmit_ring_base, transmit_ring_slot[write_back_index[2:0]]) + 23'd1,
                           {6'd0,
                            write_back_index == 4'd0,                                          /* STP on the first buffer */
                            write_back_index == transmit_buffer_count - 4'd1,                  /* ENP on the last */
                            transmit_buffer_address_high[write_back_index[2:0]]},
                           2'b11);
            end else if (access_finished) begin
                waiting_for_access <= 1'b0;
                state              <= TX_WRITE_BACK_STATUS;
            end
        end

        TX_WRITE_BACK_STATUS: begin
            if (!waiting_for_access) begin
                write_word(ring_address(transmit_ring_base, transmit_ring_slot[write_back_index[2:0]]) + 23'd3, 16'h0000, 2'b11);
            end else if (access_finished) begin
                waiting_for_access <= 1'b0;
                write_back_index   <= write_back_index + 4'd1;
                state              <= TX_WRITE_BACK_FLAGS;
            end
        end

        TX_NEXT_DESCRIPTOR: begin
            transmit_ring_index <= (transmit_ring_index + 8'd1) & transmit_ring_mask;
            transmit_demand     <= 1'b1;                            /* look at the next descriptor at once */
            waiting_for_access  <= 1'b0;
            state               <= IDLE;
        end

        /* ---------------- receive: pad, FCS, filter, then store into the ring */
        RX_PREPARE: begin
            case (prepare_phase)
                3'd0: begin
                    /* pad to the minimum frame size (frames from the wire have no FCS yet) */
                    if (!frame_has_fcs && receive_frame_length < MINIMUM_FRAME_BYTES) begin
                        write_receive_byte(receive_frame_length[10:0], 8'h00);
                        receive_frame_length <= receive_frame_length + 12'd1;
                    end else begin
                        prepare_phase    <= 3'd1;
                        receive_position <= 12'd0;
                        receive_crc      <= 32'hffffffff;
                    end
                end

                3'd1: begin
                    /* CRC over the frame: set the read address, wait one clock, take the byte */
                    if (receive_position < receive_frame_length) begin
                        case (crc_read_phase)
                            2'd0: begin
                                receive_read_address <= receive_position[10:0];
                                crc_read_phase       <= 2'd1;
                            end
                            2'd1: crc_read_phase <= 2'd2;
                            default: begin
                                receive_crc      <= crc_after_byte(receive_crc, receive_read_data);
                                receive_position <= receive_position + 12'd1;
                                crc_read_phase   <= 2'd0;
                            end
                        endcase
                    end else begin
                        prepare_phase <= 3'd2;
                    end
                end

                3'd2: begin
                    if (!frame_has_fcs) begin
                        /* append the FCS, LSB first */
                        fcs_bytes_left       <= 3'd4;
                        fcs_write_address    <= receive_frame_length[10:0];
                        fcs_shifter          <= ~receive_crc;
                        receive_frame_length <= receive_frame_length + 12'd4;
                        crc_error            <= 1'b0;
                        prepare_phase        <= 3'd4;
                    end else begin
                        crc_error     <= (receive_crc != CRC_GOOD_FRAME_RESIDUE);      /* the residue of a good frame */
                        prepare_phase <= 3'd3;
                    end
                end

                3'd4: begin
                    /* wait until the FCS bytes are in the buffer */
                    if (fcs_bytes_left == 3'd0)
                        prepare_phase <= 3'd3;
                end

                default: begin
                    if (!frame_accepted) begin
                        frame_ready <= 1'b0;
                        state       <= IDLE;
                    end else begin
                        receive_position        <= 12'd0;
                        receive_start_of_packet <= 1'b1;
                        receive_descriptor      <= ring_address(receive_ring_base, receive_ring_index);
                        waiting_for_access      <= 1'b0;
                        state                   <= RX_READ_FLAGS;
                    end
                end
            endcase
        end

        RX_READ_FLAGS: begin
            if (!waiting_for_access) begin
                read_word(receive_descriptor + 23'd1);
            end else if (access_finished) begin
                waiting_for_access <= 1'b0;
                descriptor_word1   <= access_read_data;

                if (access_error) begin
                    csr0_merr   <= 1'b1;
                    frame_ready <= 1'b0;
                    state       <= IDLE;
                end else if (!access_read_data[15]) begin
                    /* no buffer: a missed packet; if it was not the first buffer of the frame the receiver interrupts as well */
                    if (receive_start_of_packet) begin
                        csr0_miss <= 1'b1;
                    end else begin
                        csr0_miss <= 1'b1;
                        csr0_rint <= 1'b1;
                    end

                    frame_ready <= 1'b0;
                    state       <= IDLE;
                end else begin
                    state <= RX_READ_ADDRESS_LOW;
                end
            end
        end

        RX_READ_ADDRESS_LOW: begin
            if (!waiting_for_access) begin
                read_word(receive_descriptor);
            end else if (access_finished) begin
                waiting_for_access <= 1'b0;
                descriptor_word0   <= access_read_data;
                state              <= RX_READ_COUNT;
            end
        end

        RX_READ_COUNT: begin
            if (!waiting_for_access) begin
                read_word(receive_descriptor + 23'd2);
            end else if (access_finished) begin
                waiting_for_access <= 1'b0;
                buffer_address     <= {descriptor_word1[7:0], descriptor_word0};
                buffer_byte_count  <= -access_read_data[11:0];          /* 12 bit counter: 4096 wraps to 0 */
                buffer_byte_index  <= 12'd0;
                state              <= RX_STORE_DATA;
            end
        end

        RX_STORE_DATA: begin
            if (buffer_byte_index >= buffer_byte_count || frame_stored) begin
                state              <= RX_WRITE_BACK_FLAGS;
                waiting_for_access <= 1'b0;
            end else if (!waiting_for_access) begin
                /* read the byte at the receive position, then the one after it, then build the word and write it */
                case (store_read_phase)
                    3'd0: begin
                        receive_read_address <= receive_position[10:0];
                        store_read_phase     <= 3'd1;
                    end
                    3'd1: store_read_phase <= 3'd2;
                    3'd2: begin
                        store_low_byte       <= receive_read_data;
                        receive_read_address <= receive_position[10:0] + 11'd1;
                        store_read_phase     <= 3'd3;
                    end
                    3'd3: store_read_phase <= 3'd4;
                    default: begin
                        store_read_phase <= 3'd0;

                        if (next_byte_address[0])
                            write_data_word(next_byte_address[23:1], {store_low_byte, 8'h00}, 2'b10);
                        else if (buffer_byte_index + 12'd1 < buffer_byte_count && receive_position + 12'd1 < receive_frame_length)
                            write_data_word(next_byte_address[23:1], {receive_read_data, store_low_byte}, 2'b11);
                        else
                            write_data_word(next_byte_address[23:1], {8'h00, store_low_byte}, 2'b01);
                    end
                endcase
            end else if (access_finished) begin : advance_in_buffer
                reg [11:0] bytes_stored;

                waiting_for_access <= 1'b0;

                if (next_byte_address[0])
                    bytes_stored = 12'd1;
                else if (buffer_byte_index + 12'd1 < buffer_byte_count && receive_position + 12'd1 < receive_frame_length)
                    bytes_stored = 12'd2;
                else
                    bytes_stored = 12'd1;

                buffer_byte_index <= buffer_byte_index + bytes_stored;
                receive_position  <= receive_position + bytes_stored;

                if (access_error) begin
                    csr0_merr   <= 1'b1;
                    frame_ready <= 1'b0;
                    state       <= IDLE;
                end
            end
        end

        RX_WRITE_BACK_FLAGS: begin
            /* RMD1: OWN = 0, ERR and CRC on a CRC error, STP on the first buffer, ENP on the last */
            if (!waiting_for_access) begin
                write_word(receive_descriptor + 23'd1,
                           {1'b0, crc_error & frame_stored, 1'b0, 1'b0, crc_error & frame_stored, 1'b0,
                            receive_start_of_packet, frame_stored, descriptor_word1[7:0]},
                           2'b11);
            end else if (access_finished) begin
                waiting_for_access <= 1'b0;
                state              <= RX_WRITE_BACK_STATUS;
            end
        end

        RX_WRITE_BACK_STATUS: begin
            /* RMD3: the message count (the whole frame including the FCS) */
            if (!waiting_for_access) begin
                write_word(receive_descriptor + 23'd3, {4'd0, frame_stored ? receive_frame_length : 12'd0}, 2'b11);
            end else if (access_finished) begin
                waiting_for_access      <= 1'b0;
                receive_start_of_packet <= 1'b0;
                receive_ring_index      <= (receive_ring_index + 8'd1) & receive_ring_mask;
                receive_descriptor      <= ring_address(receive_ring_base, (receive_ring_index + 8'd1) & receive_ring_mask);

                if (frame_stored) begin
                    csr0_rint   <= 1'b1;
                    frame_ready <= 1'b0;
                    state       <= IDLE;
                end else begin
                    state <= RX_READ_FLAGS;
                end
            end
        end

        default: state <= IDLE;
    endcase

    if (rst) begin
        start_after_init    <= 1'b0;
        csr0_stop           <= 1'b1;
        csr0_strt           <= 1'b0;
        csr0_init           <= 1'b0;
        csr0_txon           <= 1'b0;
        csr0_rxon           <= 1'b0;
        csr0_inea           <= 1'b0;
        csr0_idon           <= 1'b0;
        csr0_tint           <= 1'b0;
        csr0_rint           <= 1'b0;
        csr0_merr           <= 1'b0;
        csr0_miss           <= 1'b0;
        csr0_cerr           <= 1'b0;
        csr0_babl           <= 1'b0;
        state               <= IDLE;
        waiting_for_access  <= 1'b0;
        frame_ready         <= 1'b0;
        receiving_from_wire <= 1'b0;
        rap                 <= 2'd0;
        csr3                <= 3'd0;
        transmit_demand     <= 1'b0;
        access_pending      <= 1'b0;
    end
end

endmodule
