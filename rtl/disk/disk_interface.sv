/* Copyright (c) 2026 by Hrvoje Cavrak */
`timescale 1ns / 1ps

/*
    PAL 16R4A Disk Interface IC - SH 2
*/

module disk_interface (
    input PCK,       /* (PIN 1) Processor Clock */
    /* clock enable (FPGA: PCK is the faster system clock and CE one pulse per processor clock period; tie high when PCK is the 10 MHz clock) */
    input CE,
    input nRD,       /* (PIN 2) 68010 or DMA device is reading memory or an I/O device */
    input nWR,       /* (PIN 3) 68010 or DMA device is writing to memory or an I/O device */
    input nDMARW,    /* (PIN 4) DMA read/write, high = RAM is being read, low = RAM is being written */
    input FDDRQL,    /* (PIN 5) Floppy disk drive request latched, (latched pin 38 DRQ) */
                     /* FDC data register has 16 bits of data (on reads) or is empty (on writes) */
    input nHDBCS,    /* (PIN 6) Hard disk buffer chip select */
    input nFDCS,     /* (PIN 7) Floppy disk chip select, active when CPU writing/reading floppy controller data reg */
    input nHDRE,     /* (PIN 8) Hard disk read - asserted when disk controller is reading a byte */
    input nHDWE,     /* (PIN 9) Hard disk write - asserted when disk controller is writing a byte */
                     /* (PIN 10 GND) */
    input OE,        /* (PIN 11) Output Enable (connected through 330R to GND) */
    
    output nFDRE,    /* (PIN 12) Floppy disk read */
    output nFDWE,    /* (PIN 13) Floppy disk write */
    output nFDTFER,  /* (PIN 14) Floppy disk transfer */
    output nTFER,    /* (PIN 18) Transfer request to disk DMA bus interface unit */
    
    output reg debug,
    output nqb, nqc, nqd
);

/*  Defining these so logic equations are not with active-low logic.  */

assign nFDRE = ~FDRE;
assign nFDWE = ~FDWE;
assign nTFER = ~TFER;
assign nFDTFER = ~FDTFER;

assign nqb = ~QB;
assign nqc = ~QC;
assign nqd = ~QD;

reg FDRE = 0, 
    FDWE = 0, 
    TFER = 0, 
    FDTFER = 0; 

reg QB = 0, 
    QC = 0, 
    QD = 0; 

always @(posedge PCK) if (CE) begin
  FDTFER <= (FDDRQL & ~FDTFER)  /* Floppy request and transfer not already active OR */
          | (FDTFER & ~QD);     /* Floppy transfer active and transfer + 3 clocks not active (de-latch system) */
  
  QB <= FDTFER;                 /* Propagate fd transfer + 1 clock */
  QC <= QB;                     /* Propagate fd transfer + 2 clock */
  QD <= QC;                     /* Propagate fd transfer + 3 clock */

  FDRE <= (~nFDCS & ~nRD)       /* Floppy chip select and read active */
        | (FDTFER & ~nDMARW);   /* Floppy transfer in progress and is DMA transfer */
  
  FDWE <= (~nFDCS & ~nWR)       /* Floppy chip select and write active */
        | (FDTFER & nDMARW);    /* Floppy transfer in progress and is NOT DMA transfer */

  TFER <= (QD & nDMARW)         /* Not DMA transfer and FDTFER + 3 clocks active */
        | (~QD & ~nDMARW)       /* DMA transfer and FDTFER + 3 clocks inactive */
        | (~nHDBCS & ~nHDRE)    /* HDD chip selected and HDD reading */
        | (~nHDBCS & ~nHDWE)    /* HDD chip selected and HDD writing */
        | QB;                   /* FDTFER + 1 clock */
 
 debug <= QB;
end

endmodule

