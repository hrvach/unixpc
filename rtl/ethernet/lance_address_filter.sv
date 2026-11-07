/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Receive address filter of the LANCE (lance.sv): decides from the destination address of a received frame whether the frame is kept.

    A frame is accepted when any of these holds:
      - the chip is in promiscuous mode (PROM) or in loopback (LOOP)
      - the destination is the chip's own physical address
      - the destination is the broadcast address
      - the destination is a group (multicast) address and its bit in the 64 bit logical address filter is set. The bit is picked by the upper six
        bits of the CRC-32 of the destination address (what Linux calls ether_crc_le() >> 26).

    The decision is needed only when the frame has been received completely, many clocks after its first six bytes. So the CRC and the comparison
    are done in two clocks, each registered, instead of one long path: the verdict follows a change of the destination address two clocks later.
*/
module lance_address_filter
import ethernet_pkg::*;
(
    input         clk,

    input  [47:0] destination_address,          /* the first six bytes of the frame; the first byte on the wire is bits 7:0 */
    input  [47:0] physical_address,             /* from the initialisation block, same byte order */
    input  [63:0] logical_address_filter,
    input         mode_promiscuous,
    input         mode_loopback,

    output reg    frame_accepted = 1'b0
);

/* ---- clock 1: the CRC of the destination address, which picks one of the 64 filter bits */
reg [5:0]  filter_index = 6'd0;
reg        is_group_address = 1'b0;             /* bit 0 of the first byte: set for multicast and broadcast */
reg [47:0] destination_address_registered = 48'd0;

reg [31:0] destination_crc;
integer    byte_number;

always @* begin
    destination_crc = 32'hffffffff;

    for (byte_number = 0; byte_number < 6; byte_number = byte_number + 1)
        destination_crc = crc_after_byte(destination_crc, destination_address[8 * byte_number +: 8]);
end

always @(posedge clk) begin
    filter_index                   <= destination_crc[31:26];
    is_group_address               <= destination_address[0];
    destination_address_registered <= destination_address;
end

/* ---- clock 2: the verdict */
always @(posedge clk) begin
    frame_accepted <= mode_promiscuous
                    | mode_loopback
                    | (destination_address_registered == physical_address)
                    | (destination_address_registered == 48'hffffffffffff)
                    | (is_group_address & logical_address_filter[filter_index]);
end

endmodule
