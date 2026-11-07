/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Page map RAM (four 1K x 4 static RAMs, ICs 19C-22C on the logic board): 1024 entries of 16 bits, one per 4 KB page of the 4 MB address space.

      bit 15      WE   the page may be written by a user program
      bit 14      PS1  page status, high bit
      bit 13      PS0  page status, low bit
      bits 12:10  PS2-PS4 (unused by the shipped software, kept as stored)
      bits 9:0    MA21..MA12: the physical page the logical page maps to

    Page status PS1 PS0: 00 not present, 01 present but not accessed, 10 accessed but not written, 11 written (dirty).

    The RAM has one read port (the entry of the page being accessed, available one clock after the index) and two writers:
      - the CPU, in supervisor mode, writes entries through addresses 400000-4007FF (byte wise: the 68010's UDS / LDS strobes pick the byte);
      - the page status update the MMU does after a successful access (writes the whole entry back). Both can want the RAM in the same clock: the CPU
        wins. While reset is high the RAM is cleared entry by entry (reset must last at least 1024 clocks).

    Two forms of the same RAM: a plain array for simulation, and for Quartus (ALTERA_RESERVED_QIS or SYNTH_RAM) one 1K x 8 RAM per byte lane with its
    own write enable. A single 16 bit array with byte writes and two write ports was turned into a RAM without byte enables by Quartus, and the CPU's
    upper byte never reached the map on the board.
*/
module mmu_map
(
    input             clk,
    input             rst,                  /* clears every entry (hold for at least 1024 clocks) */

    /* read port */
    input      [9:0]  read_index,           /* logical page number of the access */
    output reg [15:0] read_entry,           /* its map entry, one clock later */

    /* write port of the CPU (supervisor writes to 400000-4007FF) */
    input             cpu_write,
    input      [9:0]  cpu_index,
    input      [15:0] cpu_data,
    input      [1:0]  cpu_byte_enable,      /* [1] upper byte (UDS*), [0] lower byte (LDS*) */

    /* write port of the page status update after an access */
    input             status_write,
    input      [9:0]  status_index,
    input      [15:0] status_data
);

`ifdef ALTERA_RESERVED_QIS
 `define MAP_M10K
`endif
`ifdef SYNTH_RAM
 `define MAP_M10K
`endif

/* the entry the reset clears next */
reg [9:0] clear_index = 10'd0;

always @(posedge clk) begin
    if (rst) begin
        clear_index <= clear_index + 10'd1;
    end
end

`ifdef MAP_M10K

/*
    Quartus form: one write port, shared by the three writers in the order clear, CPU, status update (the same priority as the array form below). Each
    byte lane has its own write enable.
*/
reg  [9:0] write_index;
reg        write_upper_byte;
reg        write_lower_byte;
reg  [7:0] write_upper_data;
reg  [7:0] write_lower_data;

always @* begin
    write_index      = clear_index;
    write_upper_byte = rst;
    write_lower_byte = rst;
    write_upper_data = 8'h00;
    write_lower_data = 8'h00;

    if (!rst) begin
        if (cpu_write) begin
            write_index      = cpu_index;
            write_upper_byte = cpu_byte_enable[1];
            write_lower_byte = cpu_byte_enable[0];
            write_upper_data = cpu_data[15:8];
            write_lower_data = cpu_data[7:0];
        end
        else if (status_write) begin
            write_index      = status_index;
            write_upper_byte = 1'b1;
            write_lower_byte = 1'b1;
            write_upper_data = status_data[15:8];
            write_lower_data = status_data[7:0];
        end
    end
end

wire [15:0] lane_output;

mmu_map_lane upper_byte_lane
(
    .clk          (clk),
    .write        (write_upper_byte),
    .write_index  (write_index),
    .write_data   (write_upper_data),
    .read_index   (read_index),
    .read_data    (lane_output[15:8])
);

mmu_map_lane lower_byte_lane
(
    .clk          (clk),
    .write        (write_lower_byte),
    .write_index  (write_index),
    .write_data   (write_lower_data),
    .read_index   (read_index),
    .read_data    (lane_output[7:0])
);

always @* begin
    read_entry = lane_output;
end

`else

/* Simulation form: one array, the simulator reads and writes it by name (verilator public). */
reg [15:0] entries [0:1023] /*verilator public_flat_rw*/;

always @(posedge clk) begin
    if (rst) begin
        entries[clear_index] <= 16'h0000;
    end
    else if (cpu_write) begin
        if (cpu_byte_enable[1]) begin
            entries[cpu_index][15:8] <= cpu_data[15:8];
        end
        if (cpu_byte_enable[0]) begin
            entries[cpu_index][7:0] <= cpu_data[7:0];
        end
    end
    else if (status_write) begin
        entries[status_index] <= status_data;
    end

    read_entry <= entries[read_index];
end

`endif

endmodule

`ifdef MAP_M10K
/*
    One byte lane of the page map: 1K x 8 simple dual port RAM with one clock and a registered read. A read of the entry that is being written in the
    same clock returns the old data.
*/
module mmu_map_lane
(
    input            clk,
    input            write,
    input      [9:0] write_index,
    input      [7:0] write_data,
    input      [9:0] read_index,
    output reg [7:0] read_data
);

reg [7:0] lane_entries [0:1023];

always @(posedge clk) begin
    if (write) begin
        lane_entries[write_index] <= write_data;
    end

    read_data <= lane_entries[read_index];
end

endmodule
`endif
