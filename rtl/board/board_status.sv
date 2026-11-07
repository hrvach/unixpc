/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Status registers GSR (410000), BSR0 (430000) and BSR1 (440000), the parity error flag, and the level 7 interrupt (NMI). Logic board sheet 5.

    What they hold:
      GSR   (general status register)   D14 R/W*, D13 NPC+ (the access was a DMA master's, not the processor's), D12 PE* (page fault),
                                        D10 PIE+ (parity interrupt enabled), D8 UIE* (user access outside RAM); the other bits read as 1.
      BSR0  (bus status register 0)     D15 MMUERR*, D14 DKBG* (disk was the master), D13..D10 EXP0BG*..EXP3BG* (which expansion slot was the master),
                                        D9..D8 UDS* / LDS*, D7..D6 PA23:PA22, D5..D0 A21:A16
      BSR1  (bus status register 1)     A15:A0 of the access.

    The registers sample the access once per CPU bus cycle (or once per DMA access). After a fault or a parity error they freeze, so that the kernel's
    trap handler reads the state of the access that failed, until the CPU writes the clear register CSR (4C0000). A fault is always captured, even
    when the faulting access is itself a read of GSR or a BSR.
*/
module board_status
import unixpc_registers_pkg::*;
#(
    parameter [2:0] ETH_SLOT = 3'd0
)
(
    input         clk,

    /* the CPU bus cycle */
    input         cycle_active,
    input         cpu_space,
    input  [3:0]  cycle_age,
    input         write_waiting_for_strobes,    /* a faulting write is still waiting for its byte strobes */
    input         cpu_read,
    input         cpu_upper_strobe_n,           /* UDS* */
    input         cpu_lower_strobe_n,           /* LDS* */
    input         cpu_write_go,                 /* one clock pulse per CPU write */
    input         status_register_selected,     /* the CPU is accessing GSR / BSR0 / BSR1 */
    input         clear_register_selected,      /* the CPU is accessing the clear register CSR */

    /* DMA */
    input         dma_active,
    input         disk_active,
    input         ethernet_active,
    input         dma_access_checked,           /* a DMA access is evaluated in this clock */
    input         dma_write,

    /* MMU and addresses */
    input         nMMUERR,
    input         nPGF,
    input         user_access_outside_ram,
    input  [23:0] effective_address,            /* the CPU address after ROMLMAP forcing */
    input  [23:0] logical_address,              /* of whoever owns the bus */

    /* control */
    input         error_enable,                 /* GCR bit 0 */
    input         parity_interrupt_enable,      /* GCR bit 1 */
    input         parity_error_now,             /* a CPU read hit a byte with bad parity */

    output [15:0] gsr,                          /* layouts: unixpc_registers_pkg.sv */
    output [15:0] bsr0,
    output [15:0] bsr1,
    output reg        nperr = 1'b1,             /* PERR*: low = parity error */
    output reg        nmi   = 1'b0              /* level 7 interrupt request */
);

general_status_reg_t gsr_fields  = 16'h0000;
bus_status_reg0_t    bsr0_fields = 16'hffff;
bus_status_reg1_t    bsr1_fields = 16'hffff;

assign gsr  = gsr_fields;
assign bsr0 = bsr0_fields;
assign bsr1 = bsr1_fields;

reg frozen_after_error     = 1'b0;      /* an error is held in the registers until CSR is written */
reg sampled_this_cycle     = 1'b0;      /* the CPU's access of this bus cycle has been captured already */

wire cpu_access_faults     = ~nMMUERR | user_access_outside_ram;
wire dma_access_faults     = ~nMMUERR;

/*
    The CPU access is captured one clock into the cycle, once, and not while a write still waits for its strobes. A read of the status registers
    themselves is only captured when it faults, so that reading them does not overwrite what they hold.
*/
wire capture_cpu_access = cycle_active & ~cpu_space & (cycle_age >= 4'd1) & ~write_waiting_for_strobes & ~sampled_this_cycle
                        & (~status_register_selected | cpu_access_faults);
wire capture_dma_access = dma_access_checked;

always @(posedge clk) begin
    if (~frozen_after_error & (capture_cpu_access | capture_dma_access)) begin
        gsr_fields.unused_15  <= 1'b1;
        gsr_fields.RW         <= dma_active ? ~dma_write : cpu_read;
        gsr_fields.NPC        <= dma_active;
        gsr_fields.PF         <= nPGF;
        gsr_fields.unused_11  <= 1'b1;
        gsr_fields.PIE        <= parity_interrupt_enable;
        gsr_fields.unused_9   <= 1'b1;
        gsr_fields.UIE        <= ~user_access_outside_ram;
        gsr_fields.unused_7_0 <= 8'hff;

        bsr0_fields.MMUERR  <= nMMUERR;
        bsr0_fields.DKBG    <= ~disk_active;
        bsr0_fields.EXP0BG  <= ~(ethernet_active & ETH_SLOT == 3'd0);
        bsr0_fields.EXP1BG  <= ~(ethernet_active & ETH_SLOT == 3'd1);
        bsr0_fields.EXP2BG  <= ~(ethernet_active & ETH_SLOT == 3'd2);
        bsr0_fields.EXP3BG  <= ~(ethernet_active & ETH_SLOT == 3'd3);
        bsr0_fields.UDS     <= dma_active ? 1'b0 : cpu_upper_strobe_n;
        bsr0_fields.LDS     <= dma_active ? 1'b0 : cpu_lower_strobe_n;
        bsr0_fields.PA23    <= dma_active ? 1'b0 : effective_address[23];
        bsr0_fields.PA22    <= dma_active ? 1'b0 : effective_address[22];
        bsr0_fields.A21_A16 <= logical_address[21:16];

        /* the 68010 has no A0 pin: a byte access on the lower lane only (LDS* without UDS*) is an odd address, so A0 reads 1 */
        bsr1_fields.A15_A1  <= logical_address[15:1];
        bsr1_fields.A0      <= logical_address[0] | (~dma_active & cpu_upper_strobe_n & ~cpu_lower_strobe_n);

        if (capture_dma_access ? dma_access_faults : cpu_access_faults)
            frozen_after_error <= 1'b1;
    end

    /* a DMA page fault is a level 7 interrupt when errors are enabled */
    if (capture_dma_access & dma_access_faults & error_enable)
        nmi <= 1'b1;

    /* a parity error on a CPU read freezes the registers too, and interrupts when the parity interrupt is enabled */
    if (parity_error_now) begin
        frozen_after_error <= 1'b1;
        nperr              <= 1'b0;

        if (parity_interrupt_enable)
            nmi <= 1'b1;
    end

    if (capture_cpu_access)
        sampled_this_cycle <= 1'b1;

    if (~cycle_active)
        sampled_this_cycle <= 1'b0;

    /* writing CSR clears the error, the level 7 interrupt and the parity flag */
    if (cpu_write_go & clear_register_selected) begin
        frozen_after_error <= 1'b0;
        nmi                <= 1'b0;
        nperr              <= 1'b1;
    end
end

endmodule
