/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Main RAM port of the board: puts the CPU's and the DMA masters' accesses onto the RAM (FPGA only: on the real board the DRAM controller PALs do
    this).

      EXT_RAM = 0 : an internal behavioural array (tb/sim/models/dram.sv, simulation only); it answers in one clock and never makes anyone wait. EXT_RAM = 1 : the external
      SDRAM through the toggle handshake of rtl/ram/sdram_bridge.sv. One transaction is started per bus access, when the bridge is idle. The CPU's
                    DTACK (cpu_must_wait) and the DMA done strobe (dma_must_wait) are held back until it is finished. A transaction starts when
                    sdram_request_toggle flips; the address, write flag, byte enables and data stay stable until sdram_ack_toggle equals
                    sdram_request_toggle.
*/
module ram_port
#(
    parameter EXT_RAM = 0
)
(
    input         clk,
    input         rst,

    /* what the board decided about this access */
    input         dma_active,                   /* a DMA master owns the bus */
    input         dma_request,                  /* the DMA master asks for a memory access */
    input         dma_write,                    /* ... and it is a write */
    input  [2:0]  dma_access_age,               /* clocks since the DMA access began */
    input         cpu_cycle_active,             /* a CPU bus cycle is in progress */
    input  [3:0]  cpu_cycle_age,
    input         cpu_ram_cycle,                /* ... and it addresses RAM */
    input         cpu_read,                     /* R/W* high */
    input         cpu_write,
    input         address_in_installed_ram,     /* the mapped address is inside the RAM that is installed (OSD option) */
    input         nCASDIS,                      /* MMU PAL: low = a write to RAM is allowed */
    input         nMMUERR,                      /* MMU PAL: low = the access is an MMU error */
    input         cpu_ram_write_strobe,         /* RAM write of the CPU, already qualified */
    input         dma_ram_write_strobe,         /* RAM write of the DMA master, already qualified */
    input  [21:1] ram_word_address,
    input  [1:0]  ram_byte_enables,
    input  [15:0] ram_write_data,

    output [15:0] ram_read_data,                /* for the CPU: registered, valid one clock after the SDRAM answered (DTACK* is timed to it) */
    output [15:0] dma_read_data,                /* for the DMA masters: valid in the clock the acknowledge shows (they sample it together with their done strobe) */
    output        cpu_must_wait,                /* the SDRAM transaction of the CPU access is not finished */
    output        dma_must_wait,                /* the same for a DMA access */
    output        sdram_access_done,            /* the SDRAM transaction of this access is finished */

    /* the SDRAM bridge (EXT_RAM = 1) */
    output        sdram_request_toggle,
    output        sdram_write,
    output [21:1] sdram_address,
    output [1:0]  sdram_byte_enables,
    output [15:0] sdram_write_data,
    input  [15:0] sdram_read_data,
    input         sdram_ack_toggle,
    input         sdram_fast_ok,                /* a read launched now is answered in the shortest time (sdram_bridge.sv) */
    output        cpu_dtack_early_ok            /* DTACK* may be released before the data is back: it will be there in time */
);

generate
    if (EXT_RAM == 0) begin : internal_ram
        dram #(.AW(21)) ram
        (
            .clk(clk),
            .addr(ram_word_address),
            .we(dma_active ? dma_ram_write_strobe : cpu_ram_write_strobe),
            .be(ram_byte_enables),
            .wdata(ram_write_data),
            .q(ram_read_data)
        );

        assign dma_read_data        = ram_read_data;
        assign cpu_must_wait        = 1'b0;
        assign dma_must_wait        = 1'b0;
        assign sdram_access_done    = 1'b0;
        assign cpu_dtack_early_ok   = 1'b0;
        assign sdram_request_toggle = 1'b0;
        assign sdram_write          = 1'b0;
        assign sdram_address        = 21'd0;
        assign sdram_byte_enables   = 2'b00;
        assign sdram_write_data     = 16'd0;
    end else begin : sdram_interface
        reg  request_toggle         = 1'b0;
        reg  cpu_write_strobed      = 1'b0;     /* the CPU's write strobe has been seen in this cycle: the data is valid, a write can go out */
        reg  transaction_started    = 1'b0;     /* a transaction was launched for this access */
        wire transaction_in_flight  = request_toggle ^ sdram_ack_toggle;

`ifndef FAST_LAUNCH_AGE
 `define FAST_LAUNCH_AGE 0
`endif
        /* a CPU read goes to the SDRAM as early as its physical address is known (the page map entry is registered: valid at age 1, not at age 0) */
        /*
            The page map entry is registered (valid one clock after the map index changed). The index follows the CPU address only while no DMA master owns
            the bus, so for two clocks after a DMA access the entry can still be the DMA's: a CPU read then waits for age 1 and gets no early DTACK*.
        */
        reg  [1:0] dma_history = 2'b00;
        always @(posedge clk) dma_history <= {dma_history[0], dma_active};
        wire page_entry_stale       = dma_active | (|dma_history);
        wire cpu_read_needs_ram     = cpu_ram_cycle & address_in_installed_ram & cpu_read & (page_entry_stale ? (cpu_cycle_age >= 4'd1) : (cpu_cycle_age >= 4'd`FAST_LAUNCH_AGE));
        wire cpu_write_needs_ram    = cpu_ram_cycle & address_in_installed_ram & cpu_write & nCASDIS & cpu_write_strobed;
        wire dma_needs_ram          = dma_active & dma_request & address_in_installed_ram & (dma_access_age >= 3'd1) & nMMUERR
                                    & (~dma_write | nCASDIS);
        wire access_needs_ram       = dma_active ? dma_needs_ram : (cpu_read_needs_ram | cpu_write_needs_ram);
        wire access_in_progress     = dma_active ? dma_request : cpu_cycle_active;
        wire launch_transaction     = access_needs_ram & ~transaction_started & ~transaction_in_flight;

        /*
            The payload of the transaction is registered at the edge that flips the request toggle, so everything the bridge (100 MHz domain) looks at is
            a flip flop output on this side, stable for the whole transaction. The read data is registered on the way back for the same reason: the
            bridge's 16 bits and the acknowledge toggle change on the same 100 MHz edge and both are captured by the same 20 MHz edge, which keeps the
            path from the SDRAM domain into the board down to one flip flop to flip flop connection (see 3B1.sdc).
        */
        reg  [21:1] request_address;
        reg         request_write;
        reg  [1:0]  request_byte_enables;
        reg  [15:0] request_write_data;
        reg  [15:0] read_data_register;
        reg         fast_launch = 1'b0;     /* this CPU read was launched while the controller promised the shortest answer */

        always @(posedge clk) begin
            read_data_register <= sdram_read_data;

            if (launch_transaction)
                fast_launch <= sdram_fast_ok & ~dma_active & cpu_read & (cpu_cycle_age == 4'd0);
            else if (~access_in_progress)
                fast_launch <= 1'b0;

            if (launch_transaction) begin
                request_address      <= ram_word_address;
                request_write        <= dma_active ? dma_write : cpu_write;
                request_byte_enables <= ram_byte_enables;
                request_write_data   <= ram_write_data;
            end

            if (cpu_ram_write_strobe)
                cpu_write_strobed <= 1'b1;
            else if (~cpu_cycle_active)
                cpu_write_strobed <= 1'b0;

            if (rst) begin
                request_toggle      <= 1'b0;
                transaction_started <= 1'b0;
            end else begin
                if (launch_transaction)
                    request_toggle <= ~request_toggle;

                transaction_started <= access_in_progress ? (transaction_started | launch_transaction) : 1'b0;
            end
        end

        assign sdram_access_done    = transaction_started & ~transaction_in_flight;
        wire fast_read_ok           = ~dma_active & cpu_read & ((transaction_started & fast_launch) | (launch_transaction & sdram_fast_ok & (cpu_cycle_age == 4'd0)));
        assign cpu_dtack_early_ok   = fast_read_ok & cpu_ram_cycle & address_in_installed_ram;
        assign cpu_must_wait        = (cpu_ram_cycle & address_in_installed_ram & cpu_read & ~sdram_access_done & ~fast_read_ok)
                                    | (cpu_ram_cycle & address_in_installed_ram & cpu_write & nCASDIS & ~transaction_started);
        assign dma_must_wait        = dma_needs_ram & ~sdram_access_done;
        assign sdram_request_toggle = request_toggle;
        assign ram_read_data        = read_data_register;
        assign dma_read_data        = sdram_read_data;
        assign sdram_write          = request_write;
        assign sdram_address        = request_address;
        assign sdram_byte_enables   = request_byte_enables;
        assign sdram_write_data     = request_write_data;
    end
endgenerate

endmodule
