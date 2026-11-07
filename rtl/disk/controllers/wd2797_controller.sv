/*
    Copyright (c) 2026 by Hrvoje Cavrak

    WD2797 floppy disk controller (registers at E10000, one every two addresses), behavioural.

    What it does: the CPU loads track / sector / data and writes a command. Seeks, steps and restore only move the head position and give the status
    and the interrupt; Read Sector, Write Sector, Read Address and Write Track move data through the DMA channel (started through transfer_request
    below, the strobes are in wd_dma_port.sv). Without a floppy in the drive every command except Force Interrupt reports "not ready".

    Status register as the data sheet describes it: BUSY (bit 0) while a transfer command runs, DRQ (bit 1) while a byte waits for the DMA channel,
    the index pulse in bit 1 after a Type I command (restore, seek, step, force interrupt), WRITE PROTECT (bit 6) from the image's read-only flag (a
    write command on a protected disk ends at once with that bit and an interrupt). Force Interrupt aborts a running transfer; with all four interrupt
    condition bits 0 the abort gives no interrupt.

    What it deliberately does not model: head load / settle times, Read Track (reports "record not found"), the multiple sector flag (the DMA count
    says how many sectors the kernel moves, the media pointer just goes on), side compare and the data CRC (the image has no CRC; the CRC of the ID
    field is computed because the diagnostic checks it).
*/
module wd2797_controller
import disk_controllers_pkg::*;
(
    input         clk,
    input         rst,                       /* system reset */
    input         controller_enabled,        /* disk control register D7: 0 holds the controller in reset */
    input         disk_present,              /* a floppy image is mounted (0: the drive is empty) */
    input         eight_sector_tracks,       /* 1: 8 sectors per track, 0: 10 (OSD setting) */
    input         write_protected,           /* the mounted floppy image is read-only (the write protect notch) */
    input         data_request,              /* DRQ pin: a byte waits for the DMA channel */

    /* CPU bus */
    input         selected,                  /* the address decoder selects the controller */
    input  [2:1]  register_select,           /* A2:A1: register 0..3 */
    input         write,
    input         read,
    input  [7:0]  write_data,
    output reg [15:0] read_data,

    /* the DMA engine */
    input         engine_idle,
    input         busy_with_floppy,          /* the engine is transferring for this controller */
    input         transfer_done,             /* pulse: the transfer for this controller is over */
    output        transfer_request,          /* pulse: a command that moves data was accepted */
    output reg    transfer_is_write,         /* with transfer_request: 1 = RAM -> disk */
    output        transfer_is_read_address,  /* with transfer_request: Read Address (6 bytes of the ID field) */
    output        transfer_is_write_track,   /* with transfer_request: Write Track (format) */
    output        abort_transfer,            /* pulse: Force Interrupt while a transfer runs */
    /* with transfer_request: bytes to move: 6 for Read Address (the ID field), 0 = until the DMA count ends it */
    output [15:0] transfer_total_bytes,
    output [7:0]  physical_track,            /* head position, for the image address */
    output        side,
    output [7:0]  sector_register,
    output [7:0]  id_track,                  /* the ID field the next Read Address returns: track, side, sector, size code, CRC */
    output [7:0]  id_side,
    output [7:0]  id_sector,
    output [7:0]  id_size_code,
    output [15:0] id_crc,
    output        interrupt                  /* INTRQ: level, cleared by a read of the status register */
);

/* ---- register numbers (address bits A2:A1) and commands (bits 7:4 of the command byte): disk_controllers_pkg.sv */

/* ---- status bits as the commands leave them */
localparam [7:0] STATUS_NONE                  = 8'h00;
localparam [7:0] STATUS_RECORD_NOT_FOUND      = 8'h10;   /* bit 4 */
localparam [7:0] STATUS_HEAD_LOADED           = 8'h20;   /* bit 5 */
localparam [7:0] STATUS_HEAD_LOADED_TRACK0    = 8'h24;   /* bit 5 and bit 2 (track 00) */
localparam [7:0] STATUS_HEAD_LOADED_SEEK_ERROR = 8'h30;  /* bit 5 and bit 4: the verify found no ID field */
localparam [7:0] STATUS_WRITE_PROTECT         = 8'h40;   /* bit 6 */
localparam [7:0] STATUS_NOT_READY             = 8'h80;   /* bit 7 */
/* bit 7 and bit 6: an empty drive (the driver detects a disk change from this bit toggling) */
localparam [7:0] STATUS_NOT_READY_PROTECTED   = 8'hc0;

localparam [7:0] LAST_TRACK_PLUS_ONE = 8'd40;            /* 40 cylinders */

/* The index hole passes once per turn: 300 rpm = 200 ms, 20 MHz clock; the pulse is 1 ms wide */
localparam [21:0] INDEX_PERIOD_CLOCKS = 22'd4000000;
localparam [21:0] INDEX_PULSE_CLOCKS  = 22'd20000;

reg [7:0] status            = 8'h00;
reg [7:0] track_register    = 8'h00;
reg [7:0] sector_number     = 8'd1;
reg [7:0] data_register     = 8'h00;
reg [7:0] head_track        = 8'h00;             /* where the head really is */
reg       side_register     = 1'b0;
reg       step_direction_in = 1'b0;              /* the last step went inward */
reg       interrupt_request = 1'b0;
/* the status is a Type I status: bit 1 is the index pulse, bit 6 write protect (otherwise bit 1 is DRQ) */
reg       status_is_type_one = 1'b1;
reg       abort_without_interrupt = 1'b0;        /* Force Interrupt with no condition bits aborted the transfer: it ends silently */
reg [21:0] index_counter    = 22'd0;

reg [3:0] next_id_sector    = 4'd1;              /* sector number of the next ID field under the head (rotates) */
reg [7:0] id_track_register = 8'h00;
reg [7:0] id_side_register  = 8'h00;
reg [7:0] id_sector_register = 8'h00;
reg [7:0] id_size_register  = 8'd2;              /* size code 2: 512 bytes */
reg [15:0] id_crc_register  = 16'h0000;

wire [3:0] sectors_per_track = eight_sector_tracks ? 4'd8 : 4'd10;

assign interrupt         = interrupt_request;
assign physical_track    = head_track;
assign side              = side_register;
assign sector_register   = sector_number;
assign id_track          = id_track_register;
assign id_side           = id_side_register;
assign id_sector         = id_sector_register;
assign id_size_code      = id_size_register;
assign id_crc            = id_crc_register;

/* CRC-CCITT (x^16 + x^12 + x^5 + 1) of one byte, as the WD2797 uses it for ID and data fields */
function [15:0] crc_ccitt_byte(input [15:0] crc_in, input [7:0] data_byte);
    integer bit_index;
    reg [15:0] crc;
    begin
        crc = crc_in ^ {data_byte, 8'h00};
        for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
            crc = crc[15] ? ((crc << 1) ^ 16'h1021) : (crc << 1);
        end
        crc_ccitt_byte = crc;
    end
endfunction

/* ---- what a command write does, decided in the clock of the write (the DMA engine latches the request at the same clock edge) */
wire [3:0] command_code     = write_data[7:4];
wire       command_write    = write && selected && register_select == WD2797_REGISTER_STATUS_COMMAND;
wire       controller_free  = !busy_with_floppy;
wire       is_force_interrupt = (command_code == WD2797_COMMAND_FORCE_INTERRUPT);
wire       drive_not_ready  = !disk_present && !is_force_interrupt;
wire       command_executes = command_write && controller_free && !drive_not_ready;
wire       is_write_command = (command_code == WD2797_COMMAND_WRITE_SECTOR)
                           || (command_code == WD2797_COMMAND_WRITE_SECTOR_MULTIPLE)
                           || (command_code == WD2797_COMMAND_WRITE_TRACK);
wire       write_blocked    = write_protected && is_write_command;
assign     abort_transfer   = command_write && is_force_interrupt && busy_with_floppy;
wire       index_pulse      = (index_counter < INDEX_PULSE_CLOCKS);

wire is_read_or_write_sector = (command_code == WD2797_COMMAND_READ_SECTOR) || (command_code == WD2797_COMMAND_READ_SECTOR_MULTIPLE) ||
                             (command_code == WD2797_COMMAND_WRITE_SECTOR) || (command_code == WD2797_COMMAND_WRITE_SECTOR_MULTIPLE);
wire record_exists           = !(sector_number == 8'd0 || sector_number > {4'd0, sectors_per_track} || head_track >= LAST_TRACK_PLUS_ONE);

assign transfer_request = command_executes && engine_idle &&
                          !write_blocked &&
                          ((is_read_or_write_sector && record_exists) ||
                           command_code == WD2797_COMMAND_READ_ADDRESS ||
                           command_code == WD2797_COMMAND_WRITE_TRACK);

/* Write Track writes, Read Address reads; for a sector command bit 5 of the command byte says which (0 = Read Sector, 1 = Write Sector) */
always @* begin
    case (command_code)
        WD2797_COMMAND_WRITE_TRACK:  transfer_is_write = 1'b1;
        WD2797_COMMAND_READ_ADDRESS: transfer_is_write = 1'b0;
        default:              transfer_is_write = write_data[5];
    endcase
end

assign transfer_is_read_address  = (command_code == WD2797_COMMAND_READ_ADDRESS);
assign transfer_is_write_track   = (command_code == WD2797_COMMAND_WRITE_TRACK);
assign transfer_total_bytes      = (command_code == WD2797_COMMAND_READ_ADDRESS) ? 16'd6 : 16'd0;

/* the CRC of the ID field the next Read Address returns: three A1 sync bytes, the ID address mark FE, track, side, sector, size code */
reg [15:0] next_id_crc;
always @* begin
    next_id_crc = 16'hffff;
    next_id_crc = crc_ccitt_byte(next_id_crc, 8'ha1);
    next_id_crc = crc_ccitt_byte(next_id_crc, 8'ha1);
    next_id_crc = crc_ccitt_byte(next_id_crc, 8'ha1);
    next_id_crc = crc_ccitt_byte(next_id_crc, 8'hfe);
    next_id_crc = crc_ccitt_byte(next_id_crc, head_track);
    next_id_crc = crc_ccitt_byte(next_id_crc, {7'b0, side_register});
    next_id_crc = crc_ccitt_byte(next_id_crc, {4'b0, next_id_sector});
    next_id_crc = crc_ccitt_byte(next_id_crc, 8'd2);
end

always @(posedge clk) begin
    index_counter <= (index_counter == INDEX_PERIOD_CLOCKS - 22'd1) ? 22'd0 : index_counter + 22'd1;

    /* ---- the transfer is over */
    if (transfer_done) begin
        status                  <= STATUS_NONE;
        interrupt_request       <= !abort_without_interrupt;
        abort_without_interrupt <= 1'b0;
    end

    /* ---- reading the status register acknowledges the interrupt */
    if (read && selected && register_select == WD2797_REGISTER_STATUS_COMMAND) begin
        interrupt_request <= 1'b0;
    end

    /* ---- register writes */
    if (write && selected) begin
        case (register_select)
            WD2797_REGISTER_TRACK:  track_register <= write_data;
            WD2797_REGISTER_SECTOR: sector_number  <= write_data;
            WD2797_REGISTER_DATA:   data_register  <= write_data;

            WD2797_REGISTER_STATUS_COMMAND: begin
                if (controller_free && drive_not_ready) begin
                    status             <= STATUS_NOT_READY;
                    status_is_type_one <= 1'b0;
                    interrupt_request  <= 1'b1;
                end
                else if (controller_free || is_force_interrupt) begin
                    if (!is_force_interrupt) begin
                        interrupt_request  <= 1'b0;
                        status_is_type_one <= (command_code <= 4'd7);       /* restore, seek, step: Type I */
                    end
                    case (command_code)
                        WD2797_COMMAND_RESTORE: begin
                            track_register    <= 8'd0;
                            head_track        <= 8'd0;
                            interrupt_request <= 1'b1;
                            status            <= STATUS_HEAD_LOADED_TRACK0;
                        end

                        WD2797_COMMAND_SEEK: begin
                            if (write_data[2] && data_register >= LAST_TRACK_PLUS_ONE) begin
                                /* verify (V) beyond the last track: no ID field, seek error (without V the step just happens, manual 4-21) */
                                interrupt_request <= 1'b1;
                                status            <= STATUS_HEAD_LOADED_SEEK_ERROR;
                            end
                            else begin
                                track_register    <= data_register;
                                head_track        <= data_register;
                                interrupt_request <= 1'b1;
                                status            <= (data_register == 8'd0) ? STATUS_HEAD_LOADED_TRACK0 : STATUS_HEAD_LOADED;
                            end
                        end

                        WD2797_COMMAND_STEP, WD2797_COMMAND_STEP_UPDATE: begin
                            head_track        <= step_direction_in ? head_track + 8'd1 : head_track - 8'd1;
                            interrupt_request <= 1'b1;
                            status            <= (head_track == 8'd1 && !step_direction_in) ? STATUS_HEAD_LOADED_TRACK0 : STATUS_HEAD_LOADED;
                            if (write_data[4]) begin
                                track_register <= step_direction_in ? head_track + 8'd1 : head_track - 8'd1;
                            end
                        end

                        WD2797_COMMAND_STEP_IN, WD2797_COMMAND_STEP_IN_UPDATE: begin
                            head_track <= head_track + 8'd1;
                            if (write_data[4]) begin
                                track_register <= head_track + 8'd1;
                            end
                            step_direction_in <= 1'b1;
                            interrupt_request <= 1'b1;
                            status            <= STATUS_HEAD_LOADED;
                        end

                        WD2797_COMMAND_STEP_OUT, WD2797_COMMAND_STEP_OUT_UPDATE: begin
                            head_track <= head_track - 8'd1;
                            if (write_data[4]) begin
                                track_register <= head_track - 8'd1;
                            end
                            step_direction_in <= 1'b0;
                            interrupt_request <= 1'b1;
                            status            <= (head_track == 8'd1) ? STATUS_HEAD_LOADED_TRACK0 : STATUS_HEAD_LOADED;
                        end

                        WD2797_COMMAND_READ_SECTOR, WD2797_COMMAND_READ_SECTOR_MULTIPLE,
                        WD2797_COMMAND_WRITE_SECTOR, WD2797_COMMAND_WRITE_SECTOR_MULTIPLE: begin
                            side_register <= write_data[1];
                            if (write_blocked) begin
                                status            <= STATUS_WRITE_PROTECT;
                                interrupt_request <= 1'b1;
                            end
                            else if (!record_exists) begin
                                status            <= STATUS_RECORD_NOT_FOUND;
                                interrupt_request <= 1'b1;
                            end
                            else if (engine_idle) begin
                                status <= STATUS_NONE;                   /* the interrupt comes when the transfer is over */
                            end
                        end

                        WD2797_COMMAND_READ_ADDRESS: begin
                            if (engine_idle) begin
                                id_track_register  <= head_track;
                                id_side_register   <= {7'b0, side_register};
                                id_sector_register <= {4'b0, next_id_sector};
                                id_size_register   <= 8'd2;
                                id_crc_register    <= next_id_crc;
                                sector_number      <= head_track;        /* the ID's track byte is copied to the sector register */
                                next_id_sector     <= (next_id_sector >= sectors_per_track) ? 4'd1 : next_id_sector + 4'd1;
                                status             <= STATUS_NONE;
                            end
                        end

                        WD2797_COMMAND_READ_TRACK: begin
                            /* not supported */
                            interrupt_request <= 1'b1;
                            status            <= STATUS_RECORD_NOT_FOUND;
                        end

                        WD2797_COMMAND_WRITE_TRACK: begin
                            if (write_blocked) begin
                                status            <= STATUS_WRITE_PROTECT;
                                interrupt_request <= 1'b1;
                            end
                            else if (engine_idle) begin
                                status <= STATUS_NONE;                   /* the format stream comes through the DMA channel */
                            end
                        end

                        WD2797_COMMAND_FORCE_INTERRUPT: begin
                            if (busy_with_floppy) begin
                                /* the running transfer is stopped; its end gives the interrupt (unless no condition bit is set) and the status */
                                abort_without_interrupt <= (write_data[3:0] == 4'd0);
                                if (write_data[3:0] != 4'd0) begin
                                    interrupt_request <= 1'b1;
                                end
                            end
                            else begin
                                status             <= disk_present ? STATUS_HEAD_LOADED_TRACK0 : STATUS_NOT_READY_PROTECTED;
                                status_is_type_one <= 1'b1;
                                if (write_data[3:0] != 4'd0) begin
                                    interrupt_request <= 1'b1;
                                end
                                else begin
                                    interrupt_request <= 1'b0;
                                end
                            end
                        end

                        default: begin
                            interrupt_request <= 1'b1;
                            status            <= STATUS_NONE;
                        end
                    endcase
                end
            end
        endcase
    end

    /* ---- the disk control register holds the controller in reset */
    if (!controller_enabled) begin
        status            <= 8'h00;
        track_register    <= 8'h00;
        sector_number     <= 8'd1;
        data_register     <= 8'h00;
        head_track        <= 8'h00;
        side_register     <= 1'b0;
        step_direction_in <= 1'b0;
        interrupt_request <= 1'b0;
    end

    /* ---- system reset */
    if (rst) begin
        status            <= 8'h00;
        track_register    <= 8'h00;
        sector_number     <= 8'd1;
        data_register     <= 8'h00;
        head_track        <= 8'h00;
        side_register     <= 1'b0;
        step_direction_in <= 1'b0;
        interrupt_request <= 1'b0;
        next_id_sector    <= 4'd1;
    end
end

/* ---- what the CPU reads: the stored result of the command plus the live bits */
wire [7:0] status_read = status
                       | {7'b0, busy_with_floppy}
                       | {6'b0, status_is_type_one ? index_pulse : (busy_with_floppy && data_request), 1'b0}
                       | {1'b0, status_is_type_one && write_protected, 6'b0};

always @* begin
    case (register_select)
        WD2797_REGISTER_STATUS_COMMAND: read_data = {status_read, status_read};
        WD2797_REGISTER_TRACK:          read_data = {8'h00, track_register};
        WD2797_REGISTER_SECTOR:         read_data = {8'h00, sector_number};
        default:            read_data = {8'h00, data_register};
    endcase
end

endmodule
