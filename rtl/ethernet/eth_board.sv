/*
    Copyright (c) 2026 by Hrvoje Cavrak

    3B1 Ethernet expansion board (Convergent D-08-00243-00): the LANCE and the glue logic around it, one 256 KB slot block at C00000 + 40000 * SLOT.

      base+0000  RW  LANCE register data port (RDP)
      base+0002  RW  LANCE register address port (RAP)
      base+0004  RW  control register {INTSEL, RESERVED, DMAEN} (D2 D1 D0); reading it resets the board interrupt
      base+0006  R   LANCE data latch (second step of a CPU LANCE read: read 0000 (or 0002) first, then 0006)
      base+0006  W   status ring address (A21..A9); writing it clears the pending interrupt
      base+3FFC0..3FFFF  R  ID PROM (odd bytes); a WRITE to 3FFF8..3FFFF resets the board

    The LANCE has a 16 bit bus that is multiplexed, so the CPU cannot read its registers in one step: a read of RDP or RAP latches the value, a second
    read at base+0006 returns it.

    Interrupt sequence (all in hardware): LANCE INTR -> read CSR0 (STLRD) -> write the 16 bit status word to the status ring in main memory (STDMAWR,
    address = the ring address register in A21..A9 + a free running 8 bit counter in A8..A1, 256 words) -> write the status bits back to CSR0 to clear
    them (STLWR) -> interrupt the CPU (IPL 1 or 5, autovectored, chosen by INTSEL). So the CPU finds the reason for the interrupt in the ring instead
    of having to read the LANCE.
*/
module eth_board
#(
    parameter [2:0]  SLOT     = 3'd0,
    parameter [15:0] BOARD_ID = 16'h1003,                 /* from the WIN/3B driver (ether.o: getslot(0x1003)) */
    parameter [47:0] MAC      = 48'h023b10000001          /* station address, first byte on the wire = bits 47:40 */
)
(
    input         clk,
    input         rst,

    input  [23:1] a,                /* CPU word address */
    input         wr_go,            /* one clock pulse per CPU write */
    input         rd_go,            /* one clock pulse per CPU read */
    input  [15:0] wdata,
    output [15:0] rdata,
    output        hit,              /* the address is in this board's slot block */

    output        irq1,             /* interrupt request, level 1 or level 5 depending on INTSEL */
    output        irq5,

    /* DMA master (arbitrated by the board) */
    output        br,
    input         bg,               /* bus granted to this master */
    output        act,              /* this master owns the bus (BGACK) */
    output        m_req,
    output        m_we,
    output [23:1] m_addr,
    output [15:0] m_wdata,
    output [1:0]  m_be,
    input         m_done,
    input         m_fault,
    input  [15:0] m_rdata,

    /* the wire */
    output        tx_vld,
    output [7:0]  tx_dat,
    output        tx_end,
    input         rx_vld,
    input  [7:0]  rx_dat,
    input         rx_end,
    output        rx_rdy
);

/* ---------------------------------------------------------------- address decoding inside the slot block */
wire [7:0] high_byte      = a[23:16];
wire       in_id_prom     = (a[17:6] == 12'hfff);
wire       in_reset_area  = (a[17:3] == 15'h7fff);
wire [1:0] register_select = a[2:1];

localparam [1:0] REGISTER_RDP     = 2'd0;
localparam [1:0] REGISTER_RAP     = 2'd1;
localparam [1:0] REGISTER_CONTROL = 2'd2;
localparam [1:0] REGISTER_LATCH   = 2'd3;               /* read: the LANCE data latch; write: the status ring address */

assign hit = (high_byte[7:5] == 3'b110) & (high_byte[4:2] == SLOT);

/* ---------------------------------------------------------------- registers */
reg [3:0]  control_register = 4'd0;                     /* D2 INTSEL (0 = level 5, 1 = level 1), D0 DMAEN */
reg [12:0] status_ring_address = 13'd0;                 /* A21..A9 of the status ring */
reg [7:0]  status_ring_counter = 8'd0;                  /* A8..A1: the ring position of the next status word */
reg [15:0] lance_data_latch = 16'd0;
reg        interrupt_pending = 1'b0;

wire        lance_interrupt;
wire [15:0] lance_csr;
wire [15:0] lance_csr0;
wire [15:0] lance_rap;

wire software_reset   = hit & wr_go & in_reset_area;
wire board_reset      = rst | software_reset;

wire lance_access     = hit & ~in_id_prom & ~in_reset_area;
wire rdp_write        = lance_access & wr_go & (register_select == REGISTER_RDP);
wire rap_write        = lance_access & wr_go & (register_select == REGISTER_RAP);
wire lance_read       = hit & rd_go & ~in_id_prom & (register_select == REGISTER_RDP || register_select == REGISTER_RAP);

reg [6:0]  csr0_acknowledge = 7'd0;                     /* the board's write to CSR0 that clears the reported status bits */
reg [15:0] status_word = 16'd0;                         /* CSR0 as read for the status ring */
reg        status_written = 1'b0;

/* ---------------------------------------------------------------- ID PROM: bytes at odd offsets 3FFC1..3FFFF */
function [7:0] id_prom_byte(input [4:0] word_index);    /* word index within the 32 word PROM (offset bits 5..1) */
    case (word_index)
        5'd0:  id_prom_byte = MAC[7:0];                 /* 3FFC1 LSB of the Ethernet address ... 3FFCB MSB */
        5'd1:  id_prom_byte = MAC[15:8];
        5'd2:  id_prom_byte = MAC[23:16];
        5'd3:  id_prom_byte = MAC[31:24];
        5'd4:  id_prom_byte = MAC[39:32];
        5'd5:  id_prom_byte = MAC[47:40];
        /*
            layout from the kernel (os/drv.c slotinit / getslot): 3FFF9 id low, 3FFFB id high, 3FFFD check low, 3FFFF check high, with idlo + chklo ==
            0 and idhi + chkhi == 0 (8 bit)
        */
        5'd28: id_prom_byte = BOARD_ID[7:0];
        5'd29: id_prom_byte = BOARD_ID[15:8];
        5'd30: id_prom_byte = -BOARD_ID[7:0];
        5'd31: id_prom_byte = -BOARD_ID[15:8];
        default: id_prom_byte = 8'hff;
    endcase
endfunction

reg [15:0] read_data;

always @* begin
    read_data = 16'hffff;

    if (in_id_prom) begin
        read_data = {8'h00, id_prom_byte(a[5:1])};
    end else begin
        case (register_select)
            REGISTER_RDP, REGISTER_RAP: read_data = lance_data_latch;
            REGISTER_CONTROL:           read_data = {12'd0, control_register};
            REGISTER_LATCH:             read_data = lance_data_latch;
        endcase
    end
end

assign rdata = read_data;

always @(posedge clk) begin
    if (board_reset) begin
        control_register     <= 4'd0;
        lance_data_latch     <= 16'd0;
        interrupt_pending    <= 1'b0;
        status_ring_address  <= 13'd0;
    end else begin
        if (lance_read)
            lance_data_latch <= (register_select == REGISTER_RDP) ? lance_csr : lance_rap;

        if (lance_access & wr_go) begin
            if (register_select == REGISTER_CONTROL)
                control_register <= wdata[3:0];

            if (register_select == REGISTER_LATCH) begin
                status_ring_address <= wdata[12:0];
                interrupt_pending   <= 1'b0;
            end
        end

        if (hit & rd_go & ~in_id_prom & register_select == REGISTER_CONTROL)
            interrupt_pending <= 1'b0;

        if (status_written)
            interrupt_pending <= 1'b1;
    end
end

wire interrupt_select_level1 = control_register[2];
wire dma_enable              = control_register[0];

assign irq1 = interrupt_pending &  interrupt_select_level1;
assign irq5 = interrupt_pending & ~interrupt_select_level1;

/* ---------------------------------------------------------------- the LANCE */
wire        lance_bus_request;
wire        lance_bus_grant;
wire        lance_m_req;
wire        lance_m_we;
wire [23:1] lance_m_addr;
wire [15:0] lance_m_wdata;
wire [1:0]  lance_m_be;
wire        status_dma_wanted;

lance lan
(
    .clk        (clk),
    .rst        (board_reset),
    .rap_we     (rap_write),
    .rdp_we     (rdp_write),
    .wdata      (wdata),
    .rap_q      (lance_rap),
    .csr_q      (lance_csr),
    .intr       (lance_interrupt),
    .csr0_clear (csr0_acknowledge),
    .csr0_q     (lance_csr0),
    .br         (lance_bus_request),
    .bg         (lance_bus_grant),
    .m_req      (lance_m_req),
    .m_we       (lance_m_we),
    .m_addr     (lance_m_addr),
    .m_wdata    (lance_m_wdata),
    .m_be       (lance_m_be),
    .m_done     (m_done),
    .m_fault    (m_fault),
    .m_rdata    (m_rdata),
    .tx_vld     (tx_vld),
    .tx_dat     (tx_dat),
    .tx_end     (tx_end),
    .rx_vld     (rx_vld),
    .rx_dat     (rx_dat),
    .rx_end     (rx_end),
    .rx_rdy     (rx_rdy)
);

/* ---------------------------------------------------------------- interrupt status DMA */
typedef enum logic [2:0] {
    STATUS_IDLE        = 3'd0,
    STATUS_READ_CSR0   = 3'd1,   /* STLRD: take the LANCE's CSR0 */
    STATUS_REQUEST_BUS = 3'd3,   /* wait until the board owns the bus and the LANCE has no access running */
    STATUS_WAIT_WRITE  = 3'd4,   /* STDMAWR: write the status word to the ring */
    STATUS_CLEAR_CSR0  = 3'd5,   /* STLWR: write the reported bits back to CSR0 to clear them */
    STATUS_INTERRUPT   = 3'd6   /* interrupt the CPU */
} status_state_t;

status_state_t status_state = STATUS_IDLE;
reg       status_write_request = 1'b0;
reg       status_write_active  = 1'b0;

assign status_dma_wanted = (status_state == STATUS_READ_CSR0) | (status_state == STATUS_REQUEST_BUS) | (status_state == STATUS_WAIT_WRITE);

always @(posedge clk) begin
    status_written   <= 1'b0;
    csr0_acknowledge <= 7'd0;

    if (board_reset) begin
        status_state         <= STATUS_IDLE;
        status_write_request <= 1'b0;
        status_write_active  <= 1'b0;
        status_ring_counter  <= 8'd0;
    end else begin
        case (status_state)
            STATUS_IDLE: begin
                if (lance_interrupt && dma_enable)
                    status_state <= STATUS_READ_CSR0;
            end

            STATUS_READ_CSR0: begin
                status_word  <= lance_csr0;
                status_state <= STATUS_REQUEST_BUS;
            end

            STATUS_REQUEST_BUS: begin
                if (act && !lance_m_req) begin
                    status_write_request <= 1'b1;
                    status_write_active  <= 1'b1;
                    status_state         <= STATUS_WAIT_WRITE;
                end
            end

            STATUS_WAIT_WRITE: begin
                if (m_done) begin
                    status_write_request <= 1'b0;
                    status_write_active  <= 1'b0;
                    status_ring_counter  <= status_ring_counter + 8'd1;
                    status_state         <= STATUS_CLEAR_CSR0;
                end
            end

            STATUS_CLEAR_CSR0: begin
                csr0_acknowledge <= status_word[14:8];          /* clear what was reported */
                status_state     <= STATUS_INTERRUPT;
            end

            STATUS_INTERRUPT: begin
                status_written <= 1'b1;
                status_state   <= STATUS_IDLE;
            end

            default: status_state <= STATUS_IDLE;
        endcase
    end
end

/* ---------------------------------------------------------------- bus ownership and master port */
wire status_bus_request = (status_state == STATUS_REQUEST_BUS) | (status_state == STATUS_WAIT_WRITE);
reg  bus_owned = 1'b0;

assign br = (lance_bus_request & dma_enable) | status_bus_request;

always @(posedge clk)
    bus_owned <= board_reset ? 1'b0 : (br & (bg | bus_owned));

assign act              = bus_owned;
assign lance_bus_grant  = bus_owned & dma_enable & ~status_dma_wanted;

/* The status word write takes the master port over from the LANCE while it runs. */
assign m_req   = status_write_active ? status_write_request : lance_m_req;
assign m_we    = status_write_active ? 1'b1                 : lance_m_we;
assign m_addr  = status_write_active ? {2'b00, status_ring_address, status_ring_counter} : lance_m_addr;
assign m_wdata = status_write_active ? status_word          : lance_m_wdata;
assign m_be    = status_write_active ? 2'b11                : lance_m_be;

endmodule
