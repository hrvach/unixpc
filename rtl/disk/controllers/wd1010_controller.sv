/*
    Copyright (c) 2026 by Hrvoje Cavrak

    WD1010 hard disk controller (the task file at E00000, registers on even addresses), behavioural.

    Source: the logic board theory of operation (task file registers, Table 2-17, status and error registers, the six commands) and the test
    procedures (command bytes, the status values a read-back shows, the SDH value after Scan ID).

    The CPU loads the task file (cylinder, head, sector, sector count) and writes a command byte (command in bits 7:4):
      1x  Restore       returns the heads to track 0 (the task file cylinder becomes 0)
      7x  Seek          moves the heads to the cylinder of the task file
      2x  Read Sector   the sector goes through the DMA channel (the strobes and the data path are in wd_dma_port.sv and the DMA chips,
                        started through transfer_request)
      3x  Write Sector  the same, RAM -> disk
      4x  Scan ID       reads the next ID field under the head: the sector number register and the SDH register are updated
                        (SDH = 21: 512 byte sectors)
      5x  Write Format  formats one track: the sector table (2 bytes per sector) comes through the DMA channel, then every sector of the track is
                        written as zeros
    every other code is an undefined command: the controller aborts it (error register bit 2, status ERROR).

    While a command runs the status register shows BUSY (D2, with DRQ: DA for a write); when it is over it shows READY and SEEK COMPLETE (50, with
    ERROR: 51), INTRQ is raised and a read of the status register clears it. A command written while the controller is busy is ignored.

    What it deliberately does not model: ECC, the write fault and track 0 error conditions, ID-not-found for a wrong sector or cylinder (the image is
    as large as the file), data request timing inside a transfer. The sector count register is stored but the transfer length comes from the DMA count
    register (the kernel moves several sectors with one command and says how many words in the count register).
*/
module wd1010_controller
import disk_controllers_pkg::*;
(
    input         clk,
    input         rst,                       /* system reset */
    input         controller_enabled,        /* disk control register D4: 0 holds the controller in reset */
    input         head_select_load,          /* the CPU writes the disk control register: D2:D0 become the head bits of SDH */
    input  [2:0]  head_select_data,
    input  [2:0]  selected_head,             /* the head the drive has selected (disk control register D2:D0): Scan ID reads it from the ID field */
    input  [4:0]  sectors_per_track,         /* 16 or 17 (OSD setting): the sector numbers Scan ID rotates through */

    /* CPU bus */
    input         selected,                  /* the address decoder selects the task file */
    input  [3:1]  register_select,           /* A3:A1: register 0..7 */
    input         write,
    input         read,
    input  [7:0]  write_data,
    output reg [15:0] read_data,

    /* the DMA engine */
    input         engine_idle,               /* no transfer running or starting */
    input         busy_with_hard_disk,       /* the engine is transferring for this controller */
    input         transfer_done,             /* pulse: the transfer for this controller is over */
    output        transfer_request,          /* pulse: a command that moves data was accepted */
    output        transfer_is_write,         /* with transfer_request: 1 = RAM -> disk */
    output        transfer_is_format,        /* with transfer_request: Write Format */
    /* with transfer_request: bytes to move (the sector table of a format), 0 = until the DMA count ends it */
    output [15:0] transfer_total_bytes,
    output [10:0] cylinder,                  /* task file, for the sector address */
    output [2:0]  head,
    output [7:0]  sector_number,
    output        interrupt,                 /* INTRQ: level, cleared by a read of the status register */

    /* debug */
    output        command_ignored,           /* pulse: a command arrived while the engine could not take it */
    output [3:0]  last_command_code
);

/* ---- register numbers (address bits A3:A1) and commands (bits 7:4 of the command byte): disk_controllers_pkg.sv */

/* ---- status register bits */
localparam [7:0] STATUS_BUSY                  = 8'h80;
localparam [7:0] STATUS_READY                 = 8'h40;
localparam [7:0] STATUS_SEEK_COMPLETE         = 8'h10;
localparam [7:0] STATUS_DATA_REQUEST          = 8'h08;
localparam [7:0] STATUS_COMMAND_IN_PROGRESS   = 8'h02;
localparam [7:0] STATUS_ERROR                 = 8'h01;
localparam [7:0] STATUS_IDLE                  = STATUS_READY | STATUS_SEEK_COMPLETE;                            /* 50 */
localparam [7:0] STATUS_EXECUTING             = STATUS_BUSY | STATUS_READY | STATUS_SEEK_COMPLETE | STATUS_COMMAND_IN_PROGRESS;   /* D2 */

/* ---- error register bits */
localparam [7:0] ERROR_ABORTED_COMMAND = 8'h04;

/* ---- how long the commands that move no data take (20 MHz clocks): the image file has no mechanics, but software reads the busy status back */
localparam [15:0] SEEK_CLOCKS    = 16'd4000;          /* 200 us */
localparam [15:0] SCAN_ID_CLOCKS = 16'd8000;          /* 400 us: one ID field passes under the head */

localparam [7:0] SECTOR_SIZE_512 = 8'h20;             /* SDH bits 6:5 = 01 */

reg [7:0] data_register          = 8'h00;
reg [7:0] error_register         = 8'h00;
reg [7:0] sector_count           = 8'h00;
reg [7:0] sector_number_register = 8'h00;
reg [7:0] cylinder_low           = 8'h00;
reg [2:0] cylinder_high          = 3'd0;               /* 3 bits: the image geometry has 1040 cylinders */
reg [7:0] sdh                    = 8'h00;
reg [7:0] idle_status            = 8'h00;              /* the status when no command runs */
reg       interrupt_request      = 1'b0;

reg       executing              = 1'b0;               /* a command is running: BUSY */
reg       executing_with_data    = 1'b0;               /* ... and it moves data from RAM to the disk: DRQ */
reg       timed_command          = 1'b0;               /* ... and it ends by itself after the timer */
reg [15:0] execution_timer       = 16'd0;
reg [3:0] running_command        = 4'd0;
reg [4:0] next_id_sector         = 5'd0;               /* sector number of the next ID field under the head (rotates) */

reg [3:0] last_command           = 4'd0;

assign interrupt         = interrupt_request;
assign cylinder          = {cylinder_high, cylinder_low};
assign head              = sdh[2:0];
assign sector_number     = sector_number_register;
assign last_command_code = last_command;

/* ---- what a command write does, decided in the clock of the write (the DMA engine latches the request at the same clock edge) */
wire [3:0] command_code        = write_data[7:4];
wire       command_write       = write && selected && register_select == WD1010_REGISTER_STATUS_COMMAND;
wire       is_transfer_command = (command_code == WD1010_COMMAND_READ_SECTOR) ||
                                 (command_code == WD1010_COMMAND_WRITE_SECTOR) ||
                                 (command_code == WD1010_COMMAND_WRITE_FORMAT);
wire       command_accepted    = command_write && !busy_with_hard_disk && !executing;

assign transfer_request     = command_accepted && is_transfer_command && engine_idle;
assign transfer_is_write    = (command_code == WD1010_COMMAND_WRITE_SECTOR) || (command_code == WD1010_COMMAND_WRITE_FORMAT);
assign transfer_is_format   = (command_code == WD1010_COMMAND_WRITE_FORMAT);
/* two bytes per sector in the format table */
assign transfer_total_bytes = (command_code == WD1010_COMMAND_WRITE_FORMAT) ? {7'd0, sector_count, 1'b0} : 16'd0;
assign command_ignored      = command_write && (busy_with_hard_disk || executing || (is_transfer_command && !engine_idle));

always @(posedge clk) begin
    /* ---- timed commands (restore, seek, scan ID) end by themselves */
    if (executing && timed_command) begin
        if (execution_timer == 16'd0) begin
            executing         <= 1'b0;
            timed_command     <= 1'b0;
            idle_status       <= STATUS_IDLE;
            interrupt_request <= 1'b1;
            if (running_command == WD1010_COMMAND_SCAN_ID) begin
                /* the ID field found: sector number, and the SDH register shows 512 byte sectors and the head the drive has selected */
                sector_number_register <= {3'd0, next_id_sector};
                sdh[6:5]               <= SECTOR_SIZE_512[6:5];
                sdh[2:0]               <= selected_head;
                next_id_sector         <= (next_id_sector + 5'd1 >= sectors_per_track) ? 5'd0 : next_id_sector + 5'd1;
            end
        end
        else begin
            execution_timer <= execution_timer - 16'd1;
        end
    end

    /* ---- the transfer is over: ready, seek complete, interrupt */
    if (transfer_done) begin
        executing           <= 1'b0;
        executing_with_data <= 1'b0;
        idle_status         <= STATUS_IDLE;
        interrupt_request   <= 1'b1;
    end

    /* ---- reading the status register acknowledges the interrupt */
    if (read && selected && register_select == WD1010_REGISTER_STATUS_COMMAND) begin
        interrupt_request <= 1'b0;
    end

    /* ---- register writes */
    if (write) begin
        if (head_select_load) begin
            sdh[2:0] <= head_select_data;
        end

        if (selected) begin
            case (register_select)
                WD1010_REGISTER_DATA:           data_register          <= write_data;
                WD1010_REGISTER_SECTOR_COUNT:   sector_count           <= write_data;
                WD1010_REGISTER_SECTOR_NUMBER:  sector_number_register <= write_data;
                WD1010_REGISTER_CYLINDER_LOW:   cylinder_low           <= write_data;
                WD1010_REGISTER_CYLINDER_HIGH:  cylinder_high          <= write_data[2:0];
                WD1010_REGISTER_SDH:            sdh                    <= write_data;

                WD1010_REGISTER_STATUS_COMMAND: begin
                    if (command_accepted) begin
                        last_command   <= command_code;
                        running_command <= command_code;
                        error_register <= 8'h00;
                        case (command_code)
                            WD1010_COMMAND_RESTORE: begin
                                cylinder_high     <= 3'd0;
                                cylinder_low      <= 8'd0;
                                executing         <= 1'b1;
                                timed_command     <= 1'b1;
                                execution_timer   <= SEEK_CLOCKS;
                                interrupt_request <= 1'b0;
                            end

                            WD1010_COMMAND_SEEK: begin
                                executing         <= 1'b1;
                                timed_command     <= 1'b1;
                                execution_timer   <= SEEK_CLOCKS;
                                interrupt_request <= 1'b0;
                            end

                            WD1010_COMMAND_SCAN_ID: begin
                                executing         <= 1'b1;
                                timed_command     <= 1'b1;
                                execution_timer   <= SCAN_ID_CLOCKS;
                                interrupt_request <= 1'b0;
                            end

                            WD1010_COMMAND_READ_SECTOR,
                            WD1010_COMMAND_WRITE_SECTOR,
                            WD1010_COMMAND_WRITE_FORMAT: begin
                                if (engine_idle) begin
                                    executing           <= 1'b1;
                                    executing_with_data <= (command_code != WD1010_COMMAND_READ_SECTOR);
                                    interrupt_request   <= 1'b0;      /* the interrupt comes when the transfer is over */
                                end
                            end

                            default: begin
                                /* undefined command: aborted */
                                error_register    <= ERROR_ABORTED_COMMAND;
                                idle_status       <= STATUS_IDLE | STATUS_ERROR;
                                interrupt_request <= 1'b1;
                            end
                        endcase
                    end
                end

                default: ;
            endcase
        end
    end

    /* ---- the disk control register holds the controller in reset */
    if (!controller_enabled) begin
        sector_count           <= 8'h00;
        sector_number_register <= 8'h00;
        cylinder_low           <= 8'h00;
        cylinder_high          <= 3'd0;
        error_register         <= 8'h00;
        data_register          <= 8'h00;
        interrupt_request      <= 1'b0;
        idle_status            <= STATUS_IDLE;
        executing              <= 1'b0;
        executing_with_data    <= 1'b0;
        timed_command          <= 1'b0;
        sdh                    <= {5'b00000, sdh[2:0]};           /* the head bits come from the disk control register and stay */
    end

    /* ---- system reset */
    if (rst) begin
        sector_count           <= 8'h00;
        sector_number_register <= 8'h00;
        cylinder_low           <= 8'h00;
        cylinder_high          <= 3'd0;
        sdh                    <= 8'h00;
        idle_status            <= 8'h00;
        error_register         <= 8'h00;
        data_register          <= 8'h00;
        interrupt_request      <= 1'b0;
        executing              <= 1'b0;
        executing_with_data    <= 1'b0;
        timed_command          <= 1'b0;
        next_id_sector         <= 5'd0;
    end
end

/* ---- what the CPU reads */
reg [7:0] status;

always @* begin
    if (!executing)
        status = idle_status;
    else if (executing_with_data)
        status = STATUS_EXECUTING | STATUS_DATA_REQUEST;
    else
        status = STATUS_EXECUTING;
end

always @* begin
    case (register_select)
        WD1010_REGISTER_DATA:          read_data = {8'h00, data_register};
        WD1010_REGISTER_ERROR:         read_data = {8'h00, error_register};
        WD1010_REGISTER_SECTOR_COUNT:  read_data = {8'h00, sector_count};
        WD1010_REGISTER_SECTOR_NUMBER: read_data = {8'h00, sector_number_register};
        WD1010_REGISTER_CYLINDER_LOW:  read_data = {8'h00, cylinder_low};
        WD1010_REGISTER_CYLINDER_HIGH: read_data = {8'h00, 5'b00000, cylinder_high};
        WD1010_REGISTER_SDH:           read_data = {8'h00, sdh};
        default:           read_data = {status, status};
    endcase
end

endmodule
