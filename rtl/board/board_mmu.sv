/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Memory management of the board: the page map RAM (rtl/mmu/mmu_map.sv), the MMU PAL (rtl/mmu/mmu.sv, 24G) and the logic around them that the real
    board spreads over sheets 4 and 5.

    Every access, from the CPU or from a DMA master, is looked up in the page map: the page number (address bits A21..A12) selects one of 1024
    entries, and the entry gives the physical page, a write enable bit and two page status bits. The PAL compares the access (supervisor / user, read
    / write, kernel area, page status) with the entry and says whether RAM may be accessed, whether the access is a page fault or an MMU error, and
    whether a bus error is to be raised. After a successful RAM access the entry's status bits are updated: "accessed" and, for a write, "written".

    Page map entry:  D15 WE (write enable), D14 PS1 and D13 PS0 (page status; both 0 = page not present), D12..D10 unused, D9..D0 physical page.
*/
module board_mmu
import unixpc_registers_pkg::*;
(
    input         clk,
    input         rst,

    /* the CPU bus cycle */
    input         cycle_active,
    input         cpu_space,
    input  [3:0]  cycle_age,
    input  [2:0]  cpu_function_code,
    input         cpu_read,                         /* R/W* high */
    input         cpu_as_n,
    input         ram_region,                       /* the effective CPU address is in the RAM range */
    input  [23:0] effective_address,                /* CPU address after ROMLMAP forcing */
    input         error_enable,                     /* GCR EE: a bus error condition raises BERR* */

    /* CPU access to the page map itself (40xxxx) */
    input         page_map_selected,
    input         cpu_write_go,
    input  [10:1] cpu_map_index,
    input  [15:0] cpu_data,
    input  [1:0]  cpu_byte_enables,

    /* DMA master */
    input         dma_active,
    input         dma_write,
    input  [21:12] dma_page,                        /* page number of the DMA master's logical address */
    input         dma_access_checked,

    input  [23:0] logical_address,                  /* of whoever owns the bus */

    output [15:0] page_map_entry,                   /* the entry for the access (one clock after the address) */
    output        page_present,                     /* the entry's page status bits are not both 0 */
    output        nMMUERR,                          /* PAL: low = MMU error */
    output        nCASDIS,                          /* PAL: low = a write to RAM is allowed */
    output        nPGF,                             /* PAL: low = page fault */
    output        user_access_outside_ram,          /* a user mode access outside the RAM range */
    output        cpu_ram_cycle,                    /* the CPU cycle addresses RAM */
    output        cpu_bus_error                     /* bus error condition, enabled by GCR EE */
);


wire user_mode = ~cpu_function_code[2];
wire cpu_write = ~cpu_read;

assign user_access_outside_ram = user_mode & ~ram_region & ~cpu_space;
assign cpu_ram_cycle           = cycle_active & ram_region & ~cpu_space;

/*
    ---- page map: the CPU can write entries (supervisor mode only); the entry looked up is the DMA master's page, or the CPU's own map access, or the
    page of the logical address
*/
wire [9:0] map_index = dma_active       ? dma_page
                     : page_map_selected ? cpu_map_index
                     :                     logical_address[21:12];

wire cpu_map_write = cpu_write_go & page_map_selected & cpu_function_code[2];

assign page_present = |page_map_entry[14:13];

/* The status update after a successful access, one clock into the cycle (CPU) or at the DMA check. */
wire update_after_cpu_access = cpu_ram_cycle & (cycle_age == 4'd1) & page_present & nMMUERR & ~user_access_outside_ram;
wire update_after_dma_access = dma_access_checked & page_present & nMMUERR;
wire access_is_write         = dma_active ? dma_write : cpu_write;

/*
    The entry is written back with PS1 set (the page was accessed) and PS0 set once it has been written (a write now, or already dirty). WE and the
    physical page bits stay as they are.
*/
wire [15:0] updated_map_entry = {page_map_entry[15],                                                /* WE */
                                 1'b1,                                                              /* PS1: accessed */
                                 access_is_write | (page_map_entry[14] & page_map_entry[13]),       /* PS0: written */
                                 page_map_entry[12:0]};

mmu_map pmap
(
    .clk              (clk),
    .rst              (rst),
    .read_index       (map_index),
    .read_entry       (page_map_entry),
    .cpu_write        (cpu_map_write),
    .cpu_index        (cpu_map_index),
    .cpu_data         (cpu_data),
    .cpu_byte_enable  (cpu_byte_enables),
    .status_write     (update_after_cpu_access | update_after_dma_access),
    .status_index     (map_index),
    .status_data      (updated_map_entry)
);

/* the kernel area: the first 8 bytes of page zero stay readable */
wire kernel_address = ~|logical_address[21:19] & |logical_address[18:3];

/* an access outside RAM is answered by an I/O device: the "I/O DTACK" input of the PAL */
wire io_acknowledge = cycle_active & ~ram_region & (cycle_age >= 4'd1);

wire n_bus_error_enable;

mmu pal
(
    .PA22    (dma_active ? 1'b0 : effective_address[22]),
    .KADDR   (kernel_address),
    .SUPV    (cpu_function_code[2]),
    .nBGC    (~dma_active),
    .nLPS0   (page_map_entry[14]),                      /* PS1 */
    .nLPS1   (page_map_entry[13]),                      /* PS0 */
    .nRW     (dma_active ? ~dma_write : cpu_read),
    .LWE     (page_map_entry[15]),
    .SPA23   (dma_active ? 1'b0 : effective_address[23]),
    .T90     (1'b1),
    .nIODTACK(~io_acknowledge),
    .nPAS    (dma_active ? 1'b1 : cpu_as_n),
    .nPCK    (clk),
    .nENRAS  (1'b1),
    .nCASDIS (nCASDIS),
    .nBERREN (n_bus_error_enable),
    .nPGF    (nPGF),
    .nMMUERR (nMMUERR),
    .nUIE    (),
    .PPS0    ()
);

wire cpu_fault = cycle_active & ~cpu_space & (~n_bus_error_enable | user_access_outside_ram);

assign cpu_bus_error = cpu_fault & error_enable;

endmodule
