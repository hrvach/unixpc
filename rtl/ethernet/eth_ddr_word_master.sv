/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The mailbox's DDR master (FPGA only): one 64 bit Avalon access at a time, burst count 1, all byte enables set.

    The sequencer sets write_request or read_request (and the address and data on the DDR output registers); the master starts the access when it is
    idle and tells the sequencer to clear its request (write_starts / read_starts). A write is finished when the memory is not busy at the clock edge
    where WE is high; a read when the data comes back (ddr_dready), or after about a million clocks if it never does (then the data reads as zero).
    access_done is a one clock pulse; for a read the data is in read_data, which the sequencer can shift down a byte at a time (shift_read_data) while
    it unpacks it.
*/
module eth_ddr_word_master
(
    input         clk,
    input         rst,

    input         write_request,
    input         read_request,
    input         shift_read_data,              /* read_data <= read_data >> 8 */
    output        write_starts,
    output        read_starts,
    output reg    access_done = 1'b0,
    output reg [63:0] read_data = 64'd0,

    /* DDRAM (Avalon, 64 bit words) */
    input         ddr_busy,
    output reg [7:0]  ddr_burstcnt = 8'd0,
    input  [63:0] ddr_dout,
    input         ddr_dready,
    output reg    ddr_rd = 1'b0,
    output reg [7:0]  ddr_be = 8'd0,
    output reg    ddr_we = 1'b0
);

typedef enum logic [1:0] {
    MASTER_IDLE      = 2'd0,
    MASTER_WRITE     = 2'd1,   /* WE is high, waiting for BUSY to be low */
    MASTER_READ      = 2'd2,   /* RD is high, waiting for BUSY to be low */
    MASTER_READ_WAIT = 2'd3   /* the read was accepted, waiting for the data */
} master_state_t;
localparam [19:0] READ_TIMEOUT_CLOCKS = 20'hfffff;

master_state_t master_state = MASTER_IDLE;
reg [19:0] read_wait_clocks = 20'd0;

assign write_starts = (master_state == MASTER_IDLE) && write_request;
assign read_starts  = (master_state == MASTER_IDLE) && !write_request && read_request;

always @(posedge clk) begin
    access_done <= 1'b0;

    if (shift_read_data)
        read_data <= {8'd0, read_data[63:8]};

    case (master_state)
        MASTER_IDLE: begin
            read_wait_clocks <= 20'd0;

            if (write_request) begin
                ddr_be       <= 8'hff;
                ddr_burstcnt <= 8'd1;
                ddr_we       <= 1'b1;
                master_state <= MASTER_WRITE;
            end else if (read_request) begin
                ddr_burstcnt <= 8'd1;
                ddr_rd       <= 1'b1;
                master_state <= MASTER_READ;
            end
        end

        MASTER_WRITE: begin
            /* accepted: WE was high and BUSY low at this clock edge */
            if (!ddr_busy) begin
                ddr_we       <= 1'b0;
                access_done  <= 1'b1;
                master_state <= MASTER_IDLE;
            end
        end

        MASTER_READ: begin
            if (!ddr_busy) begin
                ddr_rd       <= 1'b0;
                master_state <= MASTER_READ_WAIT;
            end
        end

        MASTER_READ_WAIT: begin
            read_wait_clocks <= read_wait_clocks + 20'd1;

            if (ddr_dready) begin
                read_data    <= ddr_dout;
                access_done  <= 1'b1;
                master_state <= MASTER_IDLE;
            end else if (read_wait_clocks == READ_TIMEOUT_CLOCKS) begin
                read_data    <= 64'd0;                      /* stuck: give up */
                access_done  <= 1'b1;
                master_state <= MASTER_IDLE;
            end
        end
    endcase

    if (rst) begin
        ddr_we       <= 1'b0;
        ddr_rd       <= 1'b0;
        master_state <= MASTER_IDLE;
    end
end

endmodule
