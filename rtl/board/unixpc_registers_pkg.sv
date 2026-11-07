/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Bit layouts of the logic board's registers, written from the hardware manual (bit names and polarities as printed there: a trailing - in the
    manual is an active low bit and is written n... or noted in the comment). The modules that own a register use these types, so the code names the
    field (gsr.PF) instead of counting bits.

    A packed struct lists its fields from the most significant bit down, so the first field is the highest bit of the register.
*/
package unixpc_registers_pkg;

    /*
        General Control Register GCR: write only, address E4x000. It is an addressable latch (IC 7K): address bits A14..A12 choose the latch bit and
        data bit 15 is the value written to it, so the kernel sets or clears one bit with a single store. The latch address is the bit number: EE is
        written at E40000, PIE at E41000, BP at E42000, and so on.
    */
    typedef struct packed {
        logic DN_CONNECT;       /* latch 6: 0 = dial network connected to line 1 */
        logic L2_MODEM;         /* latch 5: 0 = modem connected to line 2 */
        logic L1_MODEM;         /* latch 4: 0 = modem connected to line 1 */
        logic ROMLMAP;          /* latch 3: 0 = the processor is forced into the ROM address space, 1 = normal addressing */
        logic BP;               /* latch 2: 1 = write memory with bad parity (diagnostic) */
        logic PIE;              /* latch 1: 1 = parity error circuit and interrupt enabled, 0 = parity disabled */
        logic EE;               /* latch 0: 1 = errors enabled, 0 = no level 7 interrupt or bus error can occur */
    } general_control_reg_t;


    /*
        General Status Register GSR: read only, address 410000. The unused bits read as 1.
    */
    typedef struct packed {
        logic unused_15;
        logic RW;               /* D14, R/W*: 0 = write cycle, 1 = read cycle */
        logic NPC;              /* D13, NPC+: 1 = non-processor cycle (a DMA master owns the bus) */
        logic PF;               /* D12, PF*: 0 = page fault; applies to the processor (supervisor or user) and to the DMA controller */
        logic unused_11;
        logic PIE;              /* D10, PIE+: 1 = parity interrupts enabled */
        logic unused_9;
        logic UIE;              /* D8, UIE*: 0 = user access to memory above 4 MB */
        logic [7:0] unused_7_0;
    } general_status_reg_t;


    /*
        Bus Status Register 0, BSR0: read only, address 430000. Who owned the bus and which part of the address the access used.
    */
    typedef struct packed {
        logic MMUERR;           /* D15, MMUERR*: 0 = MMU error: a processor or DMA page fault, a user write to a write protected page,
                                   or a user access to the kernel space (0-512K) */
        logic DKBG;             /* D14, DKBG*: 0 = disk DMA cycle */
        logic EXP0BG;           /* D13, EXP0BG*: 0 = expansion board 0 DMA cycle */
        logic EXP1BG;           /* D12, EXP1BG*: 0 = expansion board 1 DMA cycle */
        logic EXP2BG;           /* D11, EXP2BG*: 0 = expansion board 2 DMA cycle */
        logic EXP3BG;           /* D10, EXP3BG*: 0 = expansion board 3 DMA cycle */
        logic UDS;              /* D9, UDS*: 0 = upper byte access */
        logic LDS;              /* D8, LDS*: 0 = lower byte access */
        logic PA23;             /* D7: processor address bit 23 */
        logic PA22;             /* D6: processor address bit 22 */
        logic [5:0] A21_A16;    /* D5..D0: logical address bits 21 to 16 */
    } bus_status_reg0_t;


    /*
        Bus Status Register 1, BSR1: read only, address 440000. The low 16 address bits of the access.
    */
    typedef struct packed {
        logic [15:1] A15_A1;
        logic        A0;
    } bus_status_reg1_t;


    /*
        Miscellaneous Control Register MCR: write only, address 4A0000.
    */
    typedef struct packed {
        logic CLRSINT;          /* D15, CLRSINT*: dismisses the level 6 60 Hz interrupt. To dismiss it the bit must be toggled from high to low and
                                   back to high; while it is low the interrupt is masked */
        logic DMARW;            /* D14, DMAR/W*: 0 = disk DMA write operation, 1 = disk DMA read operation. Set it, together with the IDMAR/W bit of
                                   the disk DMA count register, before a disk DMA operation */
        logic LPSTR;            /* D13, LPSTR+: strobes the data of the line printer data register through the Centronics protocol. After the data is
                                   set up, LPSTR must be toggled from low to high and back to low to strobe it to the printer */
        logic MCKSEL;           /* D12, MCKSEL*: 0 = the modem RX and TX clocks are gated to the communication controller's RX and TX clock inputs,
                                   1 = the programmable timer generates the clock pulses for the communication controller */
        logic LED3;             /* D11, LED3*: 0 = red LED 3 (CR21) on */
        logic LED2;             /* D10, LED2*: 0 = green LED 2 (CR22) on */
        logic LED1;             /* D9, LED1*: 0 = yellow LED 1 (CR23) on */
        logic LED0;             /* D8, LED0*: 0 = red LED 0 (CR24) on */
        logic [7:0] unused_7_0;
    } misc_control_reg_t;


    /*
        Telephone Status Register: read only, address 450000. The unused bits read as 1.
    */
    typedef struct packed {
        logic [15:4] unused_15_4;
        logic MSG_WAIT;         /* D3, MSG WAIT*: toggles every time a message waiting pulse is detected */
        logic L2_RING;          /* D2, L2 RING*: 0 = line 2 ring */
        logic L1_RING;          /* D1, L1 RING*: 0 = line 1 ring */
        logic OFF_HOOK;         /* D0, OFF HOOK*: 0 = handset off hook */
    } telephone_status_reg_t;


    /*
        The function code of the 68010 (FC2..FC0): which kind of access the CPU makes. The MMU takes the user / supervisor decision from FC2 (a 1 is
        supervisor), and an access with all three high is not a memory access at all but an interrupt acknowledge (CPU space).
    */
    typedef enum logic [2:0] {
        FUNCTION_CODE_USER_DATA          = 3'd1,
        FUNCTION_CODE_USER_PROGRAM       = 3'd2,
        FUNCTION_CODE_SUPERVISOR_DATA    = 3'd5,
        FUNCTION_CODE_SUPERVISOR_PROGRAM = 3'd6,
        FUNCTION_CODE_CPU_SPACE          = 3'd7
    } cpu_function_code_t;


    /*
        Disk Control Register: write only, address 4E0000. It selects and resets the two disk controllers. The motor, side and precompensation bits
        are not modelled (nothing in the FPGA looks at them).
    */
    typedef struct packed {
        logic       FDRST;      /* D7: 0 = floppy disk controller reset, 1 = not reset */
        logic       FDR0;       /* D6: 0 = floppy drive 0 not selected, 1 = selected */
        logic       FDMTR;      /* D5: 0 = floppy drive motor off, 1 = on */
        logic       HDRST;      /* D4: 0 = hard disk controller reset, 1 = not reset */
        logic       HDR0;       /* D3: 0 = hard disk drive 0 not selected, 1 = selected */
        logic [2:0] HDHSEL;     /* D2..D0: the hard disk head, binary */
    } disk_control_reg_t;


    /*
        Disk DMA Count Register: address 460000. Written by the kernel before a transfer; on a read bit 15 is the under / overrun error flag instead
        of DMAEN.
    */
    typedef struct packed {
        logic        DMAEN;     /* D15: 1 = disk DMA enabled, 0 = disabled (a 0 to 1 change makes one bus request at once) */
        logic        IDMARW;    /* D14: 0 = DMA write operation (disk to RAM), 1 = DMA read operation (RAM to disk) */
        logic [13:0] DC;        /* D13..D0: the transfer count, one more for every word moved; 3FFF is terminal count */
    } disk_dma_count_reg_t;


    /*
        Printer and disk status, read only, address 470000. The register is seen in both byte lanes; the low lane has the dial tone detect in bit 0.
    */
    typedef struct packed {
        logic LPBUSY;           /* D7: 1 = line printer busy */
        logic LPSELECT;         /* D6: 1 = line printer selected */
        logic LPNOPAPER;        /* D5: 1 = line printer out of paper */
        logic nERROR;           /* D4: 0 = line printer error condition */
        logic FDINTRQ;          /* D3: 1 = floppy disk controller interrupt */
        logic HDINTRQ;          /* D2: 1 = hard disk controller interrupt */
        logic nPERR;            /* D1: 0 = main memory parity error detected; cleared with CSR (a write to 4C0000) */
        logic DTDET;            /* D0: 0 = dial tone detected */
    } printer_status_reg_t;

endpackage
