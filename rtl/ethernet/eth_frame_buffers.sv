/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Two frame buffers of 2 KB (FPGA only), used by the LANCE model (lance.sv) and by the DDR frame mailbox (eth_ddr_mbox.sv).

      transmit buffer  holds a frame on its way to the wire: in the LANCE model it is gathered from the transmit descriptors' buffers, in the
                       mailbox it is the frame captured from the LANCE's stream until it has been published to the HPS
      receive buffer   holds a frame coming from the wire: in the LANCE model it is padded, gets its CRC, is filtered and stored into the receive
                       ring; in the mailbox it is the frame fetched from the HPS until it has been streamed to the LANCE

    Each has one write port and one registered read port, so they become block RAM. A write lands one clock after it was asked for (the module is
    written to be driven by registered requests). A read gives the byte at the address of the previous clock: set the address, wait one clock, use the
    data.
*/
module eth_frame_buffers
(
    input         clk,

    input         transmit_write,
    input  [10:0] transmit_write_address,
    input  [7:0]  transmit_write_data,
    input  [10:0] transmit_read_address,
    output reg [7:0] transmit_read_data = 8'd0,

    input         receive_write,
    input  [10:0] receive_write_address,
    input  [7:0]  receive_write_data,
    input  [10:0] receive_read_address,
    output reg [7:0] receive_read_data = 8'd0
);

reg [7:0] transmit_buffer [0:2047];
reg [7:0] receive_buffer  [0:2047];

always @(posedge clk) begin
    if (transmit_write)
        transmit_buffer[transmit_write_address] <= transmit_write_data;

    transmit_read_data <= transmit_buffer[transmit_read_address];

    if (receive_write)
        receive_buffer[receive_write_address] <= receive_write_data;

    receive_read_data <= receive_buffer[receive_read_address];
end

endmodule
