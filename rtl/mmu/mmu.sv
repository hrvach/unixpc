/* Copyright (c) 2026 by Hrvoje Cavrak */
`timescale 1ns / 1ps

/*
    MMU PAL (16L8A, IC 24G, logic board sheet 2).

    Looks at the page status that the page map RAM returned for the current access (LPS0, LPS1, LWE), at who is accessing (the 68010 or a DMA master),
    in which mode (supervisor or user) and at which address, and decides whether the access is a fault. It produces the page fault and MMU error flags
    for the status registers, the bus error enable that is ANDed with EE* to make the 68010's BERR*, the column address disable that blocks a write to
    a bad page, and the updated page status bit.

    The outputs are the PAL equations as printed in the manual, one term per line with the condition they stand for. Pin names (and pin numbers) are
    the manual's; active-low pins start with n. A few inputs of the 16L8A are not used by any equation (nPCK).

    Page status: the map RAM keeps two bits per page (PS1, PS0). The board wires PS1 to pin nLPS0 and PS0 to pin nLPS1, so
      nLPS0 nLPS1 (pin levels) = 00 page not present, 01 present but not accessed, 10 accessed (read) but not written, 11 written (dirty).
*/
module mmu
(
    input PA22,      /* pin 1:  address line 22 (low: RAM and ROM space, high: I/O space) */
    input KADDR,     /* pin 2:  kernel access: A19, A20 and A21 all low */
    input SUPV,      /* pin 3:  the 68010 is in supervisor mode */
    input nBGC,      /* pin 4:  bus grant common (low: a DMA master owns the bus) */
    input nLPS0,     /* pin 5:  latched page status bit 0 */
    input nLPS1,     /* pin 6:  latched page status bit 1 */
    input nRW,       /* pin 7:  high = read, low = write */
    input LWE,       /* pin 8:  latched page write enable (low: the page is read only) */
    input SPA23,     /* pin 9:  highest address bit */
    input T90,       /* pin 11: delayed clock */
    input nIODTACK,  /* pin 13: input/output data transfer acknowledge */
    input nPAS,      /* pin 14: processor address strobe */
    input nPCK,      /* processor clock input (not used by an equation) */
    input nENRAS,    /* enable row address strobe */

    output nCASDIS,  /* pin 12: column address strobe disable */
    output nBERREN,  /* pin 15: bus error enable, ANDed with EE* to make BERR* of the 68010 */
    output nPGF,     /* pin 16: page fault */
    output nMMUERR,  /* pin 17: MMU error */
    output nUIE,     /* pin 18: user access to an address outside memory */
    output PPS0      /* pin 19: updated processor page status bit 0 */
);

/* ---- the page status the map RAM returned (the pin names carry an n, the table above is in terms of the pin levels) */
wire page_not_present            = ~nLPS0 & ~nLPS1;
wire page_present_not_accessed   = ~nLPS0 &  nLPS1;
wire page_accessed_not_written   =  nLPS0 & ~nLPS1;

/* ---- who is accessing */
wire dma_access  = ~nBGC;
wire cpu_access  =  nBGC;
wire user_mode   = ~SUPV;

/* ---- where */
wire access_to_ram         = ~PA22 & ~SPA23;         /* 000000-3FFFFF (RAM and ROM space) */
wire access_to_io          =  PA22;
wire access_to_low_io      =  PA22 & ~SPA23;         /* 400000-7FFFFF */
wire access_to_low_memory  = ~SPA23;                 /* 000000-7FFFFF */
wire access_to_kernel_area = KADDR;

/* ---- what kind of cycle */
wire write_cycle              = ~nRW;
wire page_is_read_only        = ~LWE;
wire cpu_address_strobe       = ~nPAS;
wire io_transfer_acknowledged = ~nIODTACK;
wire row_address_strobe_enabled = ~nENRAS;


/* ---- MMU error: the access is not allowed (the cause goes to the status registers) */
wire cpu_access_to_page_not_present = cpu_access & access_to_ram & page_not_present;
wire dma_access_to_page_not_present = dma_access & page_not_present;
wire user_access_to_kernel_area     = cpu_access & user_mode & access_to_ram & access_to_kernel_area;
wire user_write_to_read_only_page   = cpu_access & user_mode & access_to_ram & write_cycle & page_is_read_only;

assign nMMUERR = ~(cpu_access_to_page_not_present |
                   dma_access_to_page_not_present |
                   user_access_to_kernel_area     |
                   user_write_to_read_only_page);


/* ---- page fault: the 68010 or a DMA master touched a page that is not present */
assign nPGF = ~(cpu_access_to_page_not_present |
                dma_access_to_page_not_present);


/*
    ---- updated page status bit 0: set by a write to a present page, and held while there is an MMU error. The map RAM writes it back after the
    access.
*/
wire write_to_present_page_not_accessed = write_cycle & page_present_not_accessed & nMMUERR;   /* first access of the page is a write */
wire write_to_page_not_written          = write_cycle & page_accessed_not_written & nMMUERR;   /* the page was read before, now it is written */
wire status_held_during_error           = (page_not_present | page_present_not_accessed) & ~nMMUERR;

assign PPS0 = write_to_present_page_not_accessed |
              write_to_page_not_written          |
              status_held_during_error;


/*
    User access to an address outside memory. Printed as two terms, the second one (any user access with the row address strobe enabled) already
    covers the first (the same with an I/O address); both are kept as printed.
*/
assign nUIE = ~((cpu_access & user_mode & row_address_strobe_enabled & access_to_io) |
                (cpu_access & user_mode & row_address_strobe_enabled));


/* ---- bus error enable: five ways the 68010 gets a bus error (when EE* allows it) */
wire user_page_not_present_cycle = cpu_access & access_to_ram & page_not_present & cpu_address_strobe & T90;
wire user_kernel_area_cycle      = cpu_access & user_mode & access_to_ram & access_to_kernel_area & cpu_address_strobe & T90;
wire user_read_only_cycle        = cpu_access & user_mode & access_to_ram & write_cycle & page_is_read_only & T90;
wire user_io_not_in_memory       = cpu_access & user_mode & access_to_low_io & io_transfer_acknowledged;      /* 400000-7FFFFF, answered by I/O */
wire user_low_memory_not_in_ram  = cpu_access & user_mode & access_to_low_memory & io_transfer_acknowledged;  /* 000000-7FFFFF, answered by I/O */

assign nBERREN = ~(user_page_not_present_cycle |
                   user_kernel_area_cycle      |
                   user_read_only_cycle        |
                   user_io_not_in_memory       |
                   user_low_memory_not_in_ram);


/* ---- column address disable: a write that is an MMU error never reaches the RAM */
assign nCASDIS = ~(~nMMUERR & write_cycle);

endmodule
