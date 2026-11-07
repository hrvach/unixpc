/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Output queue of the keyboard converter (FPGA only): a 32 byte FIFO that is filled a whole report at a time and emptied one byte every 2 ms.

    The 3B1 kernel reads one keyboard byte per interrupt, so the bytes of a report must not come faster than it can take them: after a byte is
    presented (kbd_strobe, 4 clocks long) the next one waits 2 ms. A report is written at the write pointer plus the byte's index in the report and
    becomes visible to the reader only when the converter commits it ("everything or nothing"): the write pointer then moves past the whole report.
    The converter asks room_for_report before it starts one.
*/
module kbd_byte_fifo
(
    input            clk,                 /* 20 MHz */
    input            rst,

    input            report_write,        /* write report_byte at position report_index of the report being built */
    input      [3:0] report_index,
    input      [7:0] report_byte,
    input            report_commit,       /* the report is complete: it has report_index + 1 bytes, make it visible */

    output           room_for_report,     /* a report of up to 10 bytes (a key report with 9 keys and a prefix) fits */

    output reg [7:0] kbd_data,            /* byte for the keyboard port */
    output reg       kbd_strobe           /* high for 4 clocks while kbd_data is valid */
);

localparam [15:0] BYTE_INTERVAL_CLOCKS  = 16'd40000;     /* 2 ms at 20 MHz between two bytes */
localparam [1:0]  STROBE_EXTRA_CLOCKS   = 2'd3;          /* the strobe is 1 + 3 clocks long */
localparam [5:0]  LARGEST_FILL_FOR_ROOM = 6'd21;         /* 32 - 10 (the largest report) - 1 */

reg [7:0] fifo_ram [0:31];
reg [7:0] fifo_head_byte = 8'd0;                          /* RAM output: the byte at the read pointer, one clock after the pointer changes */
reg [5:0] write_pointer = 6'd0;                           /* one bit more than the RAM address so that full and empty differ */
reg [5:0] read_pointer  = 6'd0;

wire [5:0] bytes_in_fifo = write_pointer - read_pointer;

assign room_for_report = (bytes_in_fifo <= LARGEST_FILL_FOR_ROOM);

always @(posedge clk) begin
    if (report_write)
        fifo_ram[write_pointer[4:0] + {1'b0, report_index}] <= report_byte;

    fifo_head_byte <= fifo_ram[read_pointer[4:0]];
end

reg [15:0] interval_clocks_left = 16'd0;
reg [1:0]  strobe_clocks_left   = 2'd0;
/* the FIFO is not empty, one clock late: the last byte of a report is written in the clock of its commit */
reg        byte_available       = 1'b0;

always @(posedge clk) begin
    if (rst) begin
        write_pointer        <= 6'd0;
        read_pointer         <= 6'd0;
        kbd_strobe           <= 1'b0;
        interval_clocks_left <= 16'd0;
        strobe_clocks_left   <= 2'd0;
        byte_available       <= 1'b0;
    end else begin
        if (report_commit)
            write_pointer <= write_pointer + {2'b00, report_index} + 6'd1;

        byte_available <= (bytes_in_fifo != 6'd0);
        kbd_strobe     <= 1'b0;

        if (strobe_clocks_left != 2'd0) begin
            strobe_clocks_left <= strobe_clocks_left - 2'd1;
            kbd_strobe         <= 1'b1;
        end else if (interval_clocks_left != 16'd0) begin
            interval_clocks_left <= interval_clocks_left - 16'd1;
        end else if (byte_available) begin
            kbd_data             <= fifo_head_byte;
            read_pointer         <= read_pointer + 6'd1;
            strobe_clocks_left   <= STROBE_EXTRA_CLOCKS;
            kbd_strobe           <= 1'b1;
            interval_clocks_left <= BYTE_INTERVAL_CLOCKS;
        end
    end
end

endmodule
