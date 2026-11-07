/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Bus grant logic of the board: the arbiter PAL (rtl/board/arbiter/arbiter.sv, logic board sheet 3) and the gating that turns its grants into the two
    grants the DMA masters use.

    Two DMA masters share the bus: the disk controller (priority) and the expansion slot with the Ethernet board. The PAL grants in the order disk >
    expansion 3 > refresh > expansion 2, 1, 0. "Bus not in use" is the CPU having let go of the bus (BG) with no bus cycle running; the extra gating
    on the grants keeps a stale grant from reaching a master while the CPU is in a cycle, because a DMA master may only take the bus while the CPU is
    between bus cycles (otherwise the RAM / map address multiplexer flips under the CPU's read and it latches the DMA's word: seen as a bogus opcode
    with the LANCE's periodic descriptor poll). DRAM refresh is not modelled (the RAM refreshes itself).

    The arbiter PAL expects BGACK within the clock of the grant; here it runs on the 20 MHz system clock and a master needs one or two clocks to
    assert it (the disk's address IC up to one 10 MHz period). A request that arrives in that gap, and the disk asks for the bus for every word, would
    get the second grant while the first master is starting. So a master takes its grant only while the other one does not own the bus.
*/
module board_bus_grants
#(
    parameter [2:0] ETH_SLOT = 3'd0                 /* the expansion slot of the Ethernet board: its bus request goes to that slot's line */
)
(
    input  clk,
    input  cpu_bg_n,                                /* BG*: the CPU's bus grant output */
    input  cpu_as_n,
    input  dma_active,                              /* a DMA master owns the bus (BGACK) */
    input  disk_active,
    input  ethernet_active,
    input  disk_bus_request,
    input  ethernet_bus_request,

    output disk_bus_grant,
    output ethernet_bus_grant
);

wire cpu_idle = ~cpu_bg_n & cpu_as_n;

wire arbiter_output_enable;
wire n_refresh_bus_grant_cpu;
wire n_cpu_bus_grant;
wire n_expansion_0_bus_grant;
wire n_expansion_1_bus_grant;
wire n_expansion_2_bus_grant;
wire n_expansion_3_bus_grant;
wire n_disk_bus_grant;
wire n_refresh_bus_grant;

arbiter arb
(
    .clk     (clk),
    .nRFRQ   (1'b1),                                              /* refresh is not requested */
    .nEXP3RQ (~(ethernet_bus_request & (ETH_SLOT == 3'd3))),      /* the Ethernet board's request goes to the line of its slot */
    .nEXP2RQ (~(ethernet_bus_request & (ETH_SLOT == 3'd2))),
    .nEXP1RQ (~(ethernet_bus_request & (ETH_SLOT == 3'd1))),
    .nEXP0RQ (~(ethernet_bus_request & (ETH_SLOT == 3'd0))),
    .nDKRQ   (~disk_bus_request),
    .nBGACK  (~dma_active),
    .nENRAS  (cpu_idle),                                          /* "bus not in use" */
    .OE      (arbiter_output_enable),
    .nNRFBGC (n_refresh_bus_grant_cpu),
    .nBGC    (n_cpu_bus_grant),
    .nEXP0BG (n_expansion_0_bus_grant),
    .nEXP1BG (n_expansion_1_bus_grant),
    .nEXP2BG (n_expansion_2_bus_grant),
    .nDKBG   (n_disk_bus_grant),
    .nEXP3BG (n_expansion_3_bus_grant),
    .nRFBG   (n_refresh_bus_grant)
);

/* the bus grant line of the slot the Ethernet board sits in */
reg n_ethernet_slot_bus_grant;

always @* begin
    case (ETH_SLOT)
        3'd0:    n_ethernet_slot_bus_grant = n_expansion_0_bus_grant;
        3'd1:    n_ethernet_slot_bus_grant = n_expansion_1_bus_grant;
        3'd2:    n_ethernet_slot_bus_grant = n_expansion_2_bus_grant;
        default: n_ethernet_slot_bus_grant = n_expansion_3_bus_grant;
    endcase
end

assign disk_bus_grant     = ~n_disk_bus_grant & cpu_idle & ~ethernet_active;
assign ethernet_bus_grant = cpu_idle & ~n_ethernet_slot_bus_grant & ~disk_active;

endmodule
