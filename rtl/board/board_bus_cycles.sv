/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Timing of the bus cycles of the board (FPGA only: the real board schedules its signals with the PALs and the 10 MHz clock phases). Everything on
    the board is timed in system clocks (20 MHz) from the start of a bus cycle, for the CPU and for the DMA masters alike.

    ---- CPU bus cycles The cycle age counts the system clocks since the CPU asserted AS* (address strobe), saturating at 15; while a DMA master owns
    the bus the CPU's signals are ignored and no CPU cycle is reported. The answers of the board are timed from it:

      age 0  address stable, page map read in flight
      age 1  page map output valid: fault evaluation, status registers, map status update; a write may fire
      age 2  read strobe for side effects, RAM / ROM output valid
      age 3  DTACK* / BERR* asserted (the answer is registered one clock after the condition holds from age 2 on)

    An interrupt acknowledge (FC = 111, "CPU space") is answered with VPA*, the 68010's autovector request, and gets no DTACK*. A write drives its
    byte strobes late in the cycle, so a faulting write is only reported (BERR*, status registers) once UDS* / LDS* are valid: the status registers
    must see the real access size.

    ---- DMA accesses A DMA master (disk or Ethernet) presents a request; the board counts clocks since the request began so that the page map lookup,
    the MMU PAL evaluation and the RAM access happen in order, and answers with a done strobe (or a fault when the MMU PAL rejects the access).

      age 1  the page map entry is valid: the MMU PAL result is checked (access_checked), the page status updated, the RAM access started
      age 2  the RAM data is valid (with the SDRAM interface: when its transaction is finished): done
*/
module board_bus_cycles
import unixpc_registers_pkg::*;
#(
    parameter EXT_RAM = 0
)
(
    input         clk,

    /* the CPU */
    input         cpu_as_n,                     /* AS*: address strobe of the 68010 */
    input  [2:0]  cpu_fc,                       /* FC2..FC0: function code */
    input         cpu_read,                     /* R/W* high */
    input         cpu_write,
    input         strobes_valid,                /* UDS* or LDS* is asserted */
    input         cpu_bus_error,                /* bus error condition, enabled by GCR EE */
    input         cpu_ram_not_ready,            /* external RAM transaction of this cycle not finished (EXT_RAM) */

    output        cycle_active,                 /* a CPU bus cycle is in progress */
    output        cpu_space,                    /* FC = 111: interrupt acknowledge (the board answers with VPA*, no DTACK*) */
    output [3:0]  cycle_age,                    /* system clocks since AS* was first seen low */
    output        cpu_dtack_n,
    output        cpu_berr_n,
    output        cpu_vpa_n,
    output        write_go,                     /* one clock pulse per CPU write, when its strobes are valid */
    output        read_go,                      /* one clock pulse per CPU read, for registers with read side effects */
    output        write_waiting_for_strobes,    /* a write whose strobes have not arrived yet */

    /* the DMA masters */
    input         dma_active,                   /* a DMA master owns the bus */
    input         dma_request,
    input         nMMUERR,
    input         dma_ram_not_ready,            /* external RAM transaction not finished (EXT_RAM) */

    output reg [2:0] dma_access_age = 3'd0,     /* clocks since the request began, saturating at 7 */
    output        dma_access_checked,           /* the access is evaluated in this clock */
    output        dma_access_faults,            /* the MMU PAL rejected it */
    output        dma_access_done               /* the master may take the data / the write has gone to RAM */
);

/* ---- the age of the CPU cycle */
reg [3:0] age_counter = 4'd0;

assign cycle_active = ~cpu_as_n & ~dma_active;
assign cpu_space    = (cpu_fc == FUNCTION_CODE_CPU_SPACE);
assign cycle_age    = age_counter;

always @(posedge clk) begin
    if (!cycle_active)
        age_counter <= 4'd0;
    else if (age_counter != 4'hf)
        age_counter <= age_counter + 4'd1;
end

/* ---- DTACK*, BERR*, VPA* and the one-shot strobes of the CPU cycle */
reg write_done_in_this_cycle = 1'b0;
reg dtack_ready              = 1'b0;
reg berr_ready               = 1'b0;

assign write_waiting_for_strobes = cpu_write & ~strobes_valid;

/* DTACK* / BERR* follow the condition one clock later and only while AS* is still asserted. A faulting write waits for its strobes, then BERR*. */
always @(posedge clk) begin
    dtack_ready <= cycle_active & ~cpu_space & (cycle_age >= 4'd2) & ~cpu_bus_error & ~cpu_ram_not_ready;
    berr_ready  <= cycle_active & ~cpu_space & (cycle_age >= 4'd2) &  cpu_bus_error & ~write_waiting_for_strobes;
end

assign cpu_dtack_n = ~(dtack_ready & ~cpu_as_n);
assign cpu_berr_n  = ~(berr_ready  & ~cpu_as_n);
assign cpu_vpa_n   = ~(cycle_active & cpu_space);

assign write_go = cycle_active & cpu_write & strobes_valid & ~write_done_in_this_cycle & (cycle_age >= 4'd1) & ~cpu_bus_error;
assign read_go  = cycle_active & cpu_read & (cycle_age == 4'd2) & ~cpu_space;

always @(posedge clk)
    write_done_in_this_cycle <= cycle_active & (write_done_in_this_cycle | write_go);

/* ---- the age of a DMA access */
always @(posedge clk) begin
    if (dma_active & dma_request)
        dma_access_age <= (dma_access_age == 3'd7) ? 3'd7 : dma_access_age + 3'd1;
    else
        dma_access_age <= 3'd0;
end

assign dma_access_checked = dma_active & dma_request & (dma_access_age == 3'd1);
assign dma_access_faults  = dma_access_checked & ~nMMUERR;
assign dma_access_done    = dma_active & dma_request &
                            (dma_access_faults | (EXT_RAM ? ((dma_access_age >= 3'd2) & ~dma_ram_not_ready) : (dma_access_age == 3'd2)));

endmodule
