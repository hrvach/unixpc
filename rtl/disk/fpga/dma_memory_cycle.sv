/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The memory cycle of one disk DMA word (FPGA plumbing between the DMA chips and the board's memory port).

    The DMA address IC opens a window when the disk bus grant arrives (nDKBGA low) and keeps it open until this module says the memory timing is over
    (nENRAS goes high again). In that window exactly one word moves: disk -> RAM writes the word the DMA data IC holds, RAM -> disk reads a word that
    the data IC takes at the end of the window. The board's memory port does the page map lookup and the RAM access and answers with m_done (and
    m_fault when the page is not present or not writable).

      IDLE      wait for a window
      REQUEST   m_req is up: the memory port is working
      FINISH    the cycle is over for one clock: nENRAS rises, the window closes

    What it deliberately does not model: the real memory timing sequence (RAS / CAS), the board's memory port gives m_done when the access is
    complete.
*/
module dma_memory_cycle
(
    input         clk,
    input         rst,

    /* from the DMA chips */
    input         window_open,          /* the disk DMA window (DKBGA* asserted) */
    input         dma_read,             /* direction: 1 = RAM -> disk (the cycle reads), 0 = disk -> RAM (the cycle writes) */
    input  [15:0] write_data,           /* the word the DMA data IC puts on the bus for a write */

    /* the board's memory port */
    output reg        m_req,
    output reg        m_we,
    output reg [15:0] m_wdata,
    input             m_done,
    input             m_fault,
    input      [15:0] m_rdata,

    /* to the DMA chips and the engine */
    output reg [15:0] read_data,        /* the word read from RAM (valid after the cycle): the DMA data IC takes it while the window is open */
    output            memory_timing_idle, /* nENRAS: low while the memory cycle runs, high again when it is over */
    output            window_already_served, /* this window has had its memory cycle */
    output            cycle_starting,   /* pulse: the memory cycle starts in this clock */
    output            memory_fault,     /* pulse: the memory port reported a fault for this cycle */
    output [1:0]      state_code        /* 0 idle, 1 request, 2 finish (debug) */
);

typedef enum logic [1:0] {
    IDLE    = 2'd0,
    REQUEST = 2'd1,
    FINISH  = 2'd2
} memory_cycle_state_t;

memory_cycle_state_t state = IDLE;
reg       served = 1'b0;                /* set at the end of the cycle, cleared when the window closes */

assign memory_timing_idle    = (state == IDLE);
assign window_already_served = served;
assign state_code            = state;
assign cycle_starting        = (state == IDLE) && window_open && !served;
assign memory_fault          = (state == REQUEST) && m_done && m_fault;

always @(posedge clk) begin
    case (state)
        IDLE: begin
            if (window_open && !served) begin
                m_req   <= 1'b1;
                m_we    <= ~dma_read;
                m_wdata <= write_data;
                state   <= REQUEST;
            end
        end

        REQUEST: begin
            if (m_done) begin
                m_req     <= 1'b0;
                read_data <= m_rdata;
                state     <= FINISH;
            end
        end

        FINISH: begin
            state  <= IDLE;
            served <= 1'b1;
        end

        default: state <= IDLE;
    endcase

    if (!window_open) begin
        served <= 1'b0;
    end

    if (rst) begin
        state  <= IDLE;
        m_req  <= 1'b0;
        served <= 1'b0;
    end
end

endmodule
