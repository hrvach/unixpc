/*
    Copyright (c) 2026 by Hrvoje Cavrak

    AT&T UNIX PC (3B1) logic board: everything except the CPU.

    One 20 MHz clock. The CPU (68010) is attached through a plain 68000 style asynchronous bus (cpu_bus_in / cpu_bus_out). The board is built from these parts:

      board_address_decoder   which device an address selects (ROMLMAP forcing, register blocks)
      board_control_registers GCR (addressable latch), MCR, MCR2, the 60 Hz interrupt
      board_mmu               page map RAM + MMU PAL (24G) + page fault / bus error logic
      board_status            GSR, BSR0, BSR1, parity flag, level 7 interrupt
      board_bus_cycles        bus cycle age counters, DTACK*, BERR*, VPA*, the read / write strobes, the timing of a DMA master's access
      board_bus_grants        arbiter PAL and the two DMA master grants
      ram_port, board_parity  main RAM (internal array or SDRAM interface) and the bad parity flags
      rom16k, vram16          boot ROM and video RAM
      disk_ctrl_pins          WD1010 + WD2797 + the disk DMA path, images behind a DDR port
      tc8250, upd7201, telephony, eth_board, acia6850, irq_ctrl
                  RTC, serial controller, modem registers, Ethernet board, keyboard port, interrupt encoder
      board_read_mux          the CPU's data input

    Bus cycle schedule (age counts clocks from AS*): see board_bus_cycles.sv.

    The ports are bundles: one packed struct per direction of each group of signals (CPU bus, keyboard, SDRAM, disk image, serial lines, printer,
    telephone, boot ROM write port, Ethernet PHY stream), defined in board_ports_pkg.sv. Right after the port list each field gets a plain wire with
    the name the rest of this file uses.
*/
module unixpc_board
import unixpc_registers_pkg::*, board_ports_pkg::*;
#(
    parameter ROM_FILE = "",
    parameter ROM_BUILT_IN = 1,                 /* 0 = no ROM in the design: the boot ROM comes from a file (MiSTer) */
    parameter EXT_RAM = 0,                           /* 1: main RAM lives outside (SDRAM, see the mem_* ports); 0: internal behavioural array */
    /* 0: no per-byte parity flags (saves the 4 Mbit array; the ROM diagnostic parity test then fails) */
    parameter PARITY  = 1,
    parameter DMA_TRACE = 0,                    /* 1: the disk DMA trace of the simulator (tb/sim/models/disk_dma_debug.sv) */
    parameter [2:0] ETH_SLOT = 3'd0,                 /* expansion slot of the Ethernet board (C00000 + 40000 * slot) */
    parameter [15:0] ETH_BOARD_ID = 16'h1003,
    parameter [47:0] ETH_MAC = 48'h023b10000001
)
(
    input         clk,                               /* 20 MHz */
    input         rst,                               /* synchronous, hold >= 1024 clocks (page map clear) */
    input         rtc_set_en,                        /* load the RTC from rtc_set_val (BCD nibbles, register 0 in bits 3:0) */
    input  [51:0] rtc_set_val,

    /* 68010 bus, signals from the CPU */
    input  cpu_bus_from_cpu_t cpu_bus_in,

    /* 68010 bus, signals to the CPU */
    output cpu_bus_to_cpu_t cpu_bus_out,

    /* keyboard: a 3B1 key code and its strobe */
    input  keyboard_to_board_t keyboard_in,

    /* keyboard: the byte the CPU wrote to the keyboard ACIA */
    output keyboard_from_board_t keyboard_out,

    /* disk DMA debug view and self-check (disk_dma_debug.sv) */
    output disk_dma_debug_t disk_dma_debug_out,

    /* configuration (same encoding as the MiSTer OSD) */
    input  board_configuration_t configuration,

    /* external main RAM (EXT_RAM = 1), requests; see rtl/ram/sdram_bridge.sv */
    output sdram_request_t sdram_request,

    /* external main RAM, answers */
    input  sdram_response_t sdram_response,

    /* disk image storage (64 bit DDRAM style port), requests */
    output disk_image_request_t disk_image_request,

    /* disk image storage, answers */
    input  disk_image_response_t disk_image_response,

    /* serial lines, from the 7201 (channel A = RS-232 port, channel B = modem; character level, see upd7201.sv) */
    output serial_from_board_t serial_out,

    /* serial lines, to the 7201 */
    input  serial_to_board_t serial_in,

    /* Centronics printer port (rtl/printer/lp_printer.sv), signals from the board */
    output printer_from_board_t printer_out,

    /* Centronics printer port, status to the board */
    input  printer_to_board_t printer_in,

    /* telephone line (harness side of the modem): incoming call and external hook */
    input  telephone_to_board_t telephone_in,

    /* telephone line: hook state, dialled words, modem registers */
    output telephone_from_board_t telephone_out,

    /* boot ROM write port (replace the ROM from a file, rtl/rom/rom_loader.sv) */
    input  boot_rom_write_t boot_rom_write,

    /* Ethernet expansion board: frame byte stream from the PHY */
    input  ethernet_phy_to_board_t ethernet_phy_in,

    /* Ethernet expansion board: frame byte stream to the PHY */
    output ethernet_phy_from_board_t ethernet_phy_out,

    /* video scan-out port (word address in the 32 KB bitmap) */
    input  [13:0] video_scan_address,
    output [15:0] video_scan_data,

    input         eth_present,                       /* Ethernet board plugged in (0: the slot reads as empty) */

    output [3:0]  leds,
    output        dma_active
);

/* the address buses count from bit 1 (A1; the CPU has no A0), the fields of the bundles from bit 0 */
wire [23:1]  cpu_addr;
wire [21:1]  sdram_word_address;

assign cpu_addr                = cpu_bus_in.address;
assign sdram_request.word_address = sdram_word_address;

/* ====================================================================== CPU side */
wire        cycle_active;                       /* a CPU bus cycle is in progress */
wire        cpu_space;                          /* interrupt acknowledge cycle */
wire [3:0]  cycle_age;
wire        disk_dma_active;
wire        ethernet_dma_active;

assign dma_active = disk_dma_active | ethernet_dma_active;

wire        cpu_write        = ~cpu_bus_in.read_not_write;
wire        cpu_read         = cpu_bus_in.read_not_write;
wire        strobes_valid    = ~cpu_bus_in.upper_data_strobe_n | ~cpu_bus_in.lower_data_strobe_n;
wire [23:0] cpu_address      = {cpu_addr, 1'b0};
wire [1:0]  cpu_byte_enables = {~cpu_bus_in.upper_data_strobe_n, ~cpu_bus_in.lower_data_strobe_n};
wire        user_mode        = ~cpu_bus_in.function_code[2];

wire real_time_interrupt_pending;                /* the 60 Hz interrupt (level 6) */

/* ====================================================================== address decoding and control registers */
wire        error_enable;                       /* GCR bits */
wire        parity_interrupt_enable;
wire        force_bad_parity;
wire        rom_low_map;

wire [23:0] effective_address;
wire [7:0]  high_byte;
wire        ram_region;
wire        rom_region;
wire        sel_page_map;
wire        sel_general_status;
wire        sel_video_ram;
wire        sel_bus_status_0;
wire        sel_bus_status_1;
wire        sel_telephony;
wire        sel_printer_status;
wire        sel_modem_control;
wire        sel_clear_status;
wire        sel_printer_data;
wire        sel_rtc_write;
wire        sel_modem_control_2;
wire        sel_rtc_read;
wire        sel_general_control;
wire        sel_serial;
wire        sel_modem_status;
wire        sel_keyboard;

board_address_decoder decoder
(
    .cpu_address       (cpu_address),
    .rom_low_map       (rom_low_map),
    .effective_address (effective_address),
    .high_byte         (high_byte),
    .ram_region        (ram_region),
    .rom_region        (rom_region),
    .page_map          (sel_page_map),
    .general_status    (sel_general_status),
    .video_ram         (sel_video_ram),
    .bus_status_0      (sel_bus_status_0),
    .bus_status_1      (sel_bus_status_1),
    .telephony_status  (sel_telephony),
    .printer_status    (sel_printer_status),
    .modem_control     (sel_modem_control),
    .clear_status      (sel_clear_status),
    .printer_data      (sel_printer_data),
    .rtc_write         (sel_rtc_write),
    .modem_control_2   (sel_modem_control_2),
    .rtc_read          (sel_rtc_read),
    .general_control   (sel_general_control),
    .serial_7201       (sel_serial),
    .modem_status      (sel_modem_status),
    .keyboard_6850     (sel_keyboard)
);

wire        write_go;                           /* one clock pulse per CPU write, strobes valid */
wire        read_go;                            /* one clock pulse per CPU read */
wire [15:0] mcr;
wire [15:0] mcr2;

board_control_registers control_registers
(
    .clk                      (clk),
    .rst                      (rst),
    .write_go                 (write_go),
    .general_control_selected (sel_general_control),
    .modem_control_selected   (sel_modem_control),
    .modem_control_2_selected (sel_modem_control_2),
    .latch_select             (cpu_address[14:12]),
    .cpu_data                 (cpu_bus_in.data_out),
    .error_enable             (error_enable),
    .parity_interrupt_enable  (parity_interrupt_enable),
    .force_bad_parity         (force_bad_parity),
    .rom_low_map              (rom_low_map),
    .mcr                      (mcr),
    .mcr2                     (mcr2),
    .leds                     (leds),
    .real_time_interrupt_pending (real_time_interrupt_pending)
);

/* the printer port: the data register and MCR (strobe bit 13) are forwarded */
assign printer_out.data_written = write_go & sel_printer_data;
assign printer_out.data    = cpu_bus_in.lower_data_strobe_n ? cpu_bus_in.data_out[15:8] : cpu_bus_in.data_out[7:0];        /* as for the 7201: an even address byte write puts the byte on D15..D8 */
assign printer_out.control_written  = write_go & sel_modem_control;
assign printer_out.strobe   = cpu_bus_in.data_out[13];

/* ====================================================================== bus masters */
/* the two DMA masters: the disk controller (priority) and the Ethernet board; whoever is active owns the memory port */
wire        disk_request;
wire        disk_write;
wire [23:1] disk_address;
wire [15:0] disk_write_data;
wire        ethernet_request;
wire        ethernet_write;
wire [23:1] ethernet_address;
wire [15:0] ethernet_write_data;
wire [1:0]  ethernet_byte_enables;
wire        disk_bus_request;
wire        ethernet_bus_request;
wire        disk_bus_grant;
wire        ethernet_bus_grant;

wire        dma_request       = ethernet_dma_active ? ethernet_request     : disk_request;
wire        dma_write         = ethernet_dma_active ? ethernet_write       : disk_write;
wire [23:1] dma_word_address  = ethernet_dma_active ? ethernet_address     : disk_address;
wire [15:0] dma_write_data    = ethernet_dma_active ? ethernet_write_data  : disk_write_data;
wire [1:0]  dma_byte_enables  = ethernet_dma_active ? ethernet_byte_enables : 2'b11;

/* the logical address of whoever owns the bus */
wire [23:0] logical_address   = dma_active ? {dma_word_address, 1'b0} : effective_address;

board_bus_grants #(.ETH_SLOT(ETH_SLOT)) bus_grants
(
    .clk                  (clk),
    .cpu_bg_n             (cpu_bus_in.bus_grant_n),
    .cpu_as_n             (cpu_bus_in.address_strobe_n),
    .dma_active           (dma_active),
    .disk_active          (disk_dma_active),
    .ethernet_active      (ethernet_dma_active),
    .disk_bus_request     (disk_bus_request),
    .ethernet_bus_request (ethernet_bus_request),
    .disk_bus_grant       (disk_bus_grant),
    .ethernet_bus_grant   (ethernet_bus_grant)
);

assign cpu_bus_out.bus_request_n    = ~(disk_bus_request | ethernet_bus_request);
assign cpu_bus_out.bus_grant_acknowledge_n = ~dma_active;

/* ====================================================================== memory management */
wire [15:0] page_map_entry;
wire        page_present;
wire        nMMUERR;
wire        nCASDIS;
wire        nPGF;
wire        user_access_outside_ram;
wire        cpu_ram_cycle;
wire        cpu_bus_error;

wire [2:0]  dma_access_age;
wire        dma_access_checked;
wire        dma_access_faults;
wire        dma_access_done;
wire        cpu_ram_not_ready;                  /* external RAM transaction of the CPU access not finished (EXT_RAM) */
wire        cpu_dtack_early_ok;                 /* the external RAM read of this CPU cycle is sure to be back in time (EXT_RAM) */
wire        dma_ram_not_ready;
wire        sdram_access_done;

board_mmu mmu_block
(
    .clk                      (clk),
    .rst                      (rst),
    .cycle_active             (cycle_active),
    .cpu_space                (cpu_space),
    .cycle_age                (cycle_age),
    .cpu_function_code        (cpu_bus_in.function_code),
    .cpu_read                 (cpu_read),
    .cpu_as_n                 (cpu_bus_in.address_strobe_n),
    .ram_region               (ram_region),
    .effective_address        (effective_address),
    .error_enable             (error_enable),
    .page_map_selected        (sel_page_map),
    .cpu_write_go             (write_go),
    .cpu_map_index            (cpu_address[10:1]),
    .cpu_data                 (cpu_bus_in.data_out),
    .cpu_byte_enables         (cpu_byte_enables),
    .dma_active               (dma_active),
    .dma_write                (dma_write),
    .dma_page                 (dma_word_address[21:12]),
    .dma_access_checked       (dma_access_checked),
    .logical_address          (logical_address),
    .page_map_entry           (page_map_entry),
    .page_present             (page_present),
    .nMMUERR                  (nMMUERR),
    .nCASDIS                  (nCASDIS),
    .nPGF                     (nPGF),
    .user_access_outside_ram  (user_access_outside_ram),
    .cpu_ram_cycle            (cpu_ram_cycle),
    .cpu_bus_error            (cpu_bus_error)
);

/* ====================================================================== bus cycle timing: DTACK / BERR, the one-shot strobes, DMA accesses */
wire write_waiting_for_strobes;

board_bus_cycles #(.EXT_RAM(EXT_RAM)) bus_cycles
(
    .clk                       (clk),
    .cpu_as_n                  (cpu_bus_in.address_strobe_n),
    .cpu_fc                    (cpu_bus_in.function_code),
    .cpu_read                  (cpu_read),
    .cpu_write                 (cpu_write),
    .strobes_valid             (strobes_valid),
    .cpu_bus_error             (cpu_bus_error),
    .cpu_ram_not_ready         (cpu_ram_not_ready),
    .cpu_dtack_early_ok        (cpu_dtack_early_ok),
    .cycle_active              (cycle_active),
    .cpu_space                 (cpu_space),
    .cycle_age                 (cycle_age),
    .cpu_dtack_n               (cpu_bus_out.data_acknowledge_n),
    .cpu_berr_n                (cpu_bus_out.bus_error_n),
    .cpu_vpa_n                 (cpu_bus_out.valid_peripheral_address_n),
    .write_go                  (write_go),
    .read_go                   (read_go),
    .write_waiting_for_strobes (write_waiting_for_strobes),
    .dma_active                (dma_active),
    .dma_request               (dma_request),
    .nMMUERR                   (nMMUERR),
    .dma_ram_not_ready         (dma_ram_not_ready),
    .dma_access_age            (dma_access_age),
    .dma_access_checked        (dma_access_checked),
    .dma_access_faults         (dma_access_faults),
    .dma_access_done           (dma_access_done)
);

/* ====================================================================== main RAM */
wire [21:1] ram_word_address         = {page_map_entry[9:0], (dma_active ? dma_word_address[11:1] : cpu_addr[11:1])};
wire        address_in_installed_ram = ((ram_word_address[21:19] >> configuration.ram_size) == 3'd0);
wire [15:0] ram_read_data;
wire [15:0] dma_read_data;

wire        cpu_ram_write_strobe = write_go & ram_region & nCASDIS & address_in_installed_ram;
wire        dma_ram_write_strobe = dma_access_checked & dma_write & nCASDIS & address_in_installed_ram;
wire [1:0]  ram_byte_enables     = dma_active ? dma_byte_enables : cpu_byte_enables;
wire [15:0] ram_write_data       = dma_active ? dma_write_data   : cpu_bus_in.data_out;
wire        ram_write            = dma_active ? dma_ram_write_strobe : cpu_ram_write_strobe;

ram_port #(.EXT_RAM(EXT_RAM)) rport
(
    .clk                      (clk),
    .rst                      (rst),
    .dma_active               (dma_active),
    .dma_request              (dma_request),
    .dma_write                (dma_write),
    .dma_access_age           (dma_access_age),
    .cpu_cycle_active         (cycle_active),
    .cpu_cycle_age            (cycle_age),
    .cpu_ram_cycle            (cpu_ram_cycle),
    .cpu_read                 (cpu_read),
    .cpu_write                (cpu_write),
    .address_in_installed_ram (address_in_installed_ram),
    .nCASDIS                  (nCASDIS),
    .nMMUERR                  (nMMUERR),
    .cpu_ram_write_strobe     (cpu_ram_write_strobe),
    .dma_ram_write_strobe     (dma_ram_write_strobe),
    .ram_word_address         (ram_word_address),
    .ram_byte_enables         (ram_byte_enables),
    .ram_write_data           (ram_write_data),
    .ram_read_data            (ram_read_data),
    .dma_read_data            (dma_read_data),
    .cpu_must_wait            (cpu_ram_not_ready),
    .dma_must_wait            (dma_ram_not_ready),
    .sdram_access_done        (sdram_access_done),
    .sdram_request_toggle     (sdram_request.request_toggle),
    .sdram_write              (sdram_request.write),
    .sdram_address            (sdram_word_address),
    .sdram_byte_enables       (sdram_request.byte_enables),
    .sdram_write_data         (sdram_request.write_data),
    .sdram_read_data          (sdram_response.read_data),
    .sdram_ack_toggle         (sdram_response.acknowledge_toggle),
    .sdram_fast_ok            (sdram_response.fast_ok),
    .cpu_dtack_early_ok       (cpu_dtack_early_ok)
);

/* ---- bad parity flags (board_parity.sv): a CPU read of base RAM that hits a flagged byte raises PERR* */
wire bad_parity_hit;

assign sdram_request.store_bad_parity = force_bad_parity;

board_parity #(.PARITY(PARITY), .EXT_RAM(EXT_RAM)) parity
(
    .clk                    (clk),
    .rst                    (rst),
    .force_bad_parity       (force_bad_parity),
    .ram_word_address       (ram_word_address),
    .ram_write              (ram_write),
    .ram_write_byte_enables (ram_byte_enables),
    .sdram_request_toggle   (sdram_request.request_toggle),
    .sdram_write            (sdram_request.write),
    .sdram_flags_read       (sdram_response.bad_parity_flags),
    .sdram_flags_in_use     (sdram_request.parity_flags_in_use),
    .bad_parity_hit         (bad_parity_hit)
);

/* A read of the first 8 bytes of page zero from user mode returns 0 and is not checked (the kernel area starts after them). */
wire low_page_user_read = user_mode & (logical_address[21:3] == 19'd0);

/*
    With the flags in the SDRAM the check happens when the read transaction of this bus cycle is back (once per cycle); with the internal flags at the
    read strobe.
*/
reg  parity_checked_this_cycle = 1'b0;
wire parity_check_now_trigger  = (PARITY && EXT_RAM != 0) ? (cycle_active & cpu_read & ~cpu_space & sdram_access_done & ~parity_checked_this_cycle)
                                                          : read_go;

always @(posedge clk)
    parity_checked_this_cycle <= cycle_active & (parity_checked_this_cycle | parity_check_now_trigger);

wire parity_error_now = parity_check_now_trigger & ram_region & address_in_installed_ram & ~dma_active & nMMUERR & ~user_access_outside_ram
                      & page_present & ~low_page_user_read & bad_parity_hit;

/* ---- boot ROM and video RAM */
wire [15:0] rom_data;

rom16k #(.INIT_FILE(ROM_FILE), .BUILT_IN(ROM_BUILT_IN)) rom
(
    .clk   (clk),
    .addr  (cpu_addr[13:1]),
    .q     (rom_data),
    .we_high    (boot_rom_write.write_enable_high),
    .waddr_high (boot_rom_write.address_high),
    .wdata_high (boot_rom_write.data_high),
    .we_low     (boot_rom_write.write_enable_low),
    .waddr_low  (boot_rom_write.address_low),
    .wdata_low  (boot_rom_write.data_low)
);

wire [15:0] video_ram_data;

vram16 vram
(
    .clk     (clk),
    .a_addr  (cpu_addr[14:1]),
    .a_we    (write_go & sel_video_ram),
    .a_be    (cpu_byte_enables),
    .a_wdata (cpu_bus_in.data_out),
    .a_q     (video_ram_data),
    .b_addr  (video_scan_address),
    .b_q     (video_scan_data)
);

/* ====================================================================== status registers */
wire [15:0] gsr;
wire [15:0] bsr0;
wire [15:0] bsr1;
wire        nperr;
wire        nmi;

board_status #(.ETH_SLOT(ETH_SLOT)) status
(
    .clk                       (clk),
    .cycle_active              (cycle_active),
    .cpu_space                 (cpu_space),
    .cycle_age                 (cycle_age),
    .write_waiting_for_strobes (write_waiting_for_strobes),
    .cpu_read                  (cpu_read),
    .cpu_upper_strobe_n        (cpu_bus_in.upper_data_strobe_n),
    .cpu_lower_strobe_n        (cpu_bus_in.lower_data_strobe_n),
    .cpu_write_go              (write_go),
    .status_register_selected  (sel_general_status | sel_bus_status_0 | sel_bus_status_1),
    .clear_register_selected   (sel_clear_status),
    .dma_active                (dma_active),
    .disk_active               (disk_dma_active),
    .ethernet_active           (ethernet_dma_active),
    .dma_access_checked        (dma_access_checked),
    .dma_write                 (dma_write),
    .nMMUERR                   (nMMUERR),
    .nPGF                      (nPGF),
    .user_access_outside_ram   (user_access_outside_ram),
    .effective_address         (effective_address),
    .logical_address           (logical_address),
    .error_enable              (error_enable),
    .parity_interrupt_enable   (parity_interrupt_enable),
    .parity_error_now          (parity_error_now),
    .gsr                       (gsr),
    .bsr0                      (bsr0),
    .bsr1                      (bsr1),
    .nperr                     (nperr),
    .nmi                       (nmi)
);

/* ====================================================================== disk */
wire        floppy_interrupt;
wire        hard_disk_interrupt;
wire [15:0] disk_read_data;
wire        disk_selected;

/* the disk subsystem at the pin level: controller strobes, disk PAL, DMA data IC, arbiter, DMA address IC */
disk_ctrl_pins #(.DMA_TRACE(DMA_TRACE)) disk
(
    .clk                (clk),
    .rst                (rst),
    .hd_geom            (configuration.hard_disk_geometry),
    .fd_sect8           (configuration.floppy_eight_sectors),
    .fd_present         (configuration.floppy_present),
    .fd_write_protected (configuration.floppy_write_protected),
    .a                  (effective_address[23:1]),
    .wr_go              (write_go),
    .rd_go              (read_go),
    .wdata              (cpu_bus_in.data_out),
    .rdata              (disk_read_data),
    .hit                (disk_selected),
    .fdint              (floppy_interrupt),
    .hdint              (hard_disk_interrupt),
    .br                 (disk_bus_request),
    .bg                 (disk_bus_grant),
    .dma_active         (disk_dma_active),
    .m_req              (disk_request),
    .m_we               (disk_write),
    .m_addr             (disk_address),
    .m_wdata            (disk_write_data),
    .m_done             (dma_access_done),
    .m_fault            (dma_access_faults),
    .m_rdata            (address_in_installed_ram ? dma_read_data : 16'hffff),
    .ddr_rd             (disk_image_request.read),
    .ddr_we             (disk_image_request.write),
    .ddr_addr           (disk_image_request.address),
    .ddr_din            (disk_image_request.write_data),
    .ddr_be             (disk_image_request.byte_enables),
    .ddr_dout           (disk_image_response.read_data),
    .ddr_ready          (disk_image_response.ready),
    .ddr_busy           (disk_image_response.busy),
    .dbg_tog            (disk_dma_debug_out.transfer_toggle),
    .dbg_off            (disk_dma_debug_out.write_timing),
    .dbg_n              (disk_dma_debug_out.live_flags),
    .dbg_addr           (disk_dma_debug_out.last_transfer),
    .dbg_hash           (disk_dma_debug_out.status_and_count),
    .dbg_flags          (disk_dma_debug_out.transfer_flags),
    .dbg_log            (disk_dma_debug_out.register_log)
);

/* ====================================================================== real time clock */
wire [15:0] rtc_data;

tc8250 rtc
(
    .clk     (clk),
    .rst     (rst),
    .wr_go   (write_go & sel_rtc_write),
    .rd_go   (read_go & sel_rtc_read),
    .wdata   (cpu_bus_in.data_out),
    .rdata   (rtc_data),
    .set_en  (rtc_set_en),
    .set_val (rtc_set_val)
);

/* ====================================================================== keyboard port */
wire [7:0] acia_read_data;
wire       acia_irq;

acia6850 kbd
(
    .clk             (clk),
    .rst             (rst),
    .chip_select     (sel_keyboard),
    .register_select (cpu_address[1]),
    .read_pulse      (read_go),
    .write_pulse     (write_go),
    .write_data      (cpu_bus_in.data_out[15:8]),
    .read_data       (acia_read_data),
    .received_byte   (keyboard_in.key_code),
    .received_strobe (keyboard_in.key_strobe),
    .transmit_strobe (keyboard_out.byte_valid),
    .transmitted_byte(keyboard_out.byte_data),
    .irq             (acia_irq)
);

/* ====================================================================== serial controller (7201) */
wire [7:0]  scc_read_data;
wire        scc_irq;
wire [1:0]  scc_dtr;
wire [1:0]  scc_rts;
wire [1:0]  scc_brk;
wire [7:0]  tel_baud_n;
wire        modem_loopback;

/*
    Serial clock periods in system clocks. Channel A uses TMOUT = 1.2288 MHz / (4 N) when channel B's DTR is clear (DTR set: an external 9600 baud x16
    clock). Channel B has the fixed 19.2 kHz generator (also what the modem delivers at 1200 baud x16).
*/
wire [15:0] tmout_clocks   = {8'd0, (tel_baud_n == 8'd0) ? 8'd255 : tel_baud_n} * 16'd65 + {11'd0, tel_baud_n[7:3]};
/* channel B DTR: 1 = external clock, 0 = internal (kernel ml/i8274.s, clksel) */
wire [15:0] channel_a_clocks = scc_dtr[1] ? 16'd130 : tmout_clocks;
wire [15:0] channel_b_clocks = 16'd1042;

upd7201 scc
(
    .clk        (clk),
    .rst        (rst),
    .sel        (effective_address[2:1]),
    .cs         (sel_serial),
    .rd_go      (read_go),
    .wr_go      (write_go),
    /* the chip is on D7..D0 (the kernel writes ushorts); an even address byte write (UDS only) puts the byte on D15..D8 */
    .wdata      (cpu_bus_in.lower_data_strobe_n ? cpu_bus_in.data_out[15:8] : cpu_bus_in.data_out[7:0]),
    .rdata      (scc_read_data),
    .loopback   (serial_in.loopback),
    .loopback_b (modem_loopback),
    .dcd_a      (serial_in.loopback ? scc_dtr[0] : serial_in.carrier_detect[0]),
    .cts_a      (serial_in.loopback ? scc_rts[0] : serial_in.clear_to_send[0]),
    .ri_a       (1'b0),
    .dcd_b      (serial_in.loopback ? scc_dtr[0] : serial_in.carrier_detect[1]),
    .cts_b      (serial_in.clear_to_send[1]),
    .clk_per_a  (channel_a_clocks),
    .clk_per_b  (channel_b_clocks),
    .tx_valid   (serial_out.tx_valid),
    .tx_data_a  (serial_out.tx_data_a),
    .tx_data_b  (serial_out.tx_data_b),
    .rx_valid   (serial_in.rx_valid),
    .rx_data_a  (serial_in.rx_data_a),
    .rx_data_b  (serial_in.rx_data_b),
    .rx_ready   (serial_out.rx_ready),
    .dtr        (scc_dtr),
    .rts        (scc_rts),
    .brk        (scc_brk),
    .dbg        (serial_out.debug_view),
    .irq        (scc_irq)
);

assign serial_out.data_terminal_ready = scc_dtr;
assign serial_out.request_to_send = scc_rts;

/* ====================================================================== telephony and modem registers */
wire [15:0] tel_status;
wire [15:0] modem_status_data;
wire        tel_line_select_2;
wire        tel_hook1_w;
wire        tel_hook2_w;
wire        tel_line1_hold;
wire        tel_line2_hold;
wire        tel_line1_a_lead;
wire        tel_line2_a_lead;
wire [15:0] modem_register_0;
wire [15:0] modem_register_1;
wire [15:0] modem_register_8;
wire [15:0] modem_register_3;
wire [15:0] modem_register_4;
wire [15:0] modem_register_5;
wire [15:0] modem_register_6;

telephony tel
(
    .clk         (clk),
    .rst         (rst),
    .a           (effective_address[23:1]),
    .wr_go       (write_go),
    .rd_go       (read_go),
    .wdata       (cpu_bus_in.data_out),
    .ring_l1     (telephone_in.ring_line_1),
    .ring_l2     (telephone_in.ring_line_2),
    .ext_offhook (telephone_in.external_off_hook),
    .msg_wait    (1'b0),
    .carrier     (serial_in.carrier_detect[1]),
    .status      (tel_status),
    .e6_rdata    (modem_status_data),
    .hdset_relay (telephone_out.handset),
    .line_sel2   (tel_line_select_2),
    .hook1       (tel_hook1_w),
    .hook2       (tel_hook2_w),
    .l1_hold     (tel_line1_hold),
    .l2_hold     (tel_line2_hold),
    .l1_alead    (tel_line1_a_lead),
    .l2_alead    (tel_line2_a_lead),
    .baud_n      (tel_baud_n),
    .dial_tog    (telephone_out.dial_toggle),
    .dial_word   (telephone_out.dial_word),
    .mreg0       (modem_register_0),
    .mreg1       (modem_register_1),
    .mreg8       (modem_register_8),
    .mreg3       (modem_register_3),
    .mreg4       (modem_register_4),
    .mreg5       (modem_register_5),
    .mreg6       (modem_register_6),
    .modem_loop  (modem_loopback)
);

assign telephone_out.modem_register_0  = modem_register_0;
assign telephone_out.modem_register_1  = modem_register_1;
assign telephone_out.modem_register_8  = modem_register_8;
assign telephone_out.hook_line_1  = tel_hook1_w;
assign telephone_out.hook_line_2  = tel_hook2_w;
assign telephone_out.modem_baud_code     = tel_baud_n;

/* ====================================================================== Ethernet expansion board */
wire [15:0] ethernet_read_data;
wire        ethernet_selected_raw;
wire        ethernet_irq_level1;
wire        ethernet_irq_level5;
wire        ethernet_selected = ethernet_selected_raw & eth_present;

/* The MiSTer project defines ETH_DISABLE: the LANCE packet buffers are about 33k registers and no Ethernet pins reach the fabric. */
`ifndef ETH_DISABLE
eth_board #(.SLOT(ETH_SLOT), .BOARD_ID(ETH_BOARD_ID), .MAC(ETH_MAC)) eth
(
    .clk     (clk),
    .rst     (rst),
    .a       (effective_address[23:1]),
    .wr_go   (write_go),
    .rd_go   (read_go),
    .wdata   (cpu_bus_in.data_out),
    .rdata   (ethernet_read_data),
    .hit     (ethernet_selected_raw),
    .irq1    (ethernet_irq_level1),
    .irq5    (ethernet_irq_level5),
    .br      (ethernet_bus_request),
    .bg      (ethernet_bus_grant),
    .act     (ethernet_dma_active),
    .m_req   (ethernet_request),
    .m_we    (ethernet_write),
    .m_addr  (ethernet_address),
    .m_wdata (ethernet_write_data),
    .m_be    (ethernet_byte_enables),
    .m_done  (dma_access_done),
    .m_fault (dma_access_faults),
    .m_rdata (address_in_installed_ram ? dma_read_data : 16'hffff),
    .tx_vld  (ethernet_phy_out.tx_valid),
    .tx_dat  (ethernet_phy_out.tx_data),
    .tx_end  (ethernet_phy_out.tx_end),
    .rx_vld  (ethernet_phy_in.rx_valid),
    .rx_dat  (ethernet_phy_in.rx_data),
    .rx_end  (ethernet_phy_in.rx_end),
    .rx_rdy  (ethernet_phy_out.rx_ready)
);
`else
assign ethernet_read_data      = 16'h0000;
assign ethernet_selected_raw   = 1'b0;
assign ethernet_irq_level1     = 1'b0;
assign ethernet_irq_level5     = 1'b0;
assign ethernet_bus_request    = 1'b0;
assign ethernet_dma_active     = 1'b0;
assign ethernet_request        = 1'b0;
assign ethernet_write          = 1'b0;
assign ethernet_address        = 23'd0;
assign ethernet_write_data     = 16'd0;
assign ethernet_byte_enables   = 2'b00;
assign ethernet_phy_out.tx_valid              = 1'b0;
assign ethernet_phy_out.tx_data              = 8'd0;
assign ethernet_phy_out.tx_end              = 1'b0;
assign ethernet_phy_out.rx_ready              = 1'b0;
`endif

/* ====================================================================== interrupts */
irq_ctrl irqc
(
    .level7_parity_or_mmu_error  (nmi),
    .level6_real_time_clock      (real_time_interrupt_pending),
    .level5_expansion            (ethernet_irq_level5),
    .level4_serial_7201          (scc_irq),
    .level3_keyboard_or_modem    (acia_irq),
    .level2_disk_or_printer      (floppy_interrupt | hard_disk_interrupt | printer_in.acknowledge_interrupt),
    .level1_expansion            (ethernet_irq_level1),
    .nIPL                        (cpu_bus_out.interrupt_priority_n)
);

/* ====================================================================== the CPU's data input */
/* bit 0 = DTDET*: the harness / analog side presents dial tone whenever a hook relay is closed (0 = tone detected) */
wire        dial_tone_detect_n = ~(tel_hook1_w | tel_hook2_w);

/* the printer and disk status register 470000 (layout: unixpc_registers_pkg.sv), seen in both byte lanes; only the low lane has dial tone detect */
printer_status_reg_t printer_status_lane;

always @* begin
    printer_status_lane.LPBUSY    = printer_in.busy;
    printer_status_lane.LPSELECT  = printer_in.select;
    printer_status_lane.LPNOPAPER = printer_in.no_paper;
    printer_status_lane.nERROR    = 1'b1;                   /* no printer error */
    printer_status_lane.FDINTRQ   = floppy_interrupt;
    printer_status_lane.HDINTRQ   = hard_disk_interrupt;
    printer_status_lane.nPERR     = nperr;
    printer_status_lane.DTDET     = dial_tone_detect_n;
end

wire [15:0] printer_status = {printer_status_lane[7:1], 1'b1, printer_status_lane};
wire [15:0] telephony_data     = {11'h7ff, mcr2[4], tel_status[3:0]};

board_read_mux read_mux
(
    .rom_region               (rom_region),
    .rom_data                 (rom_data),
    .ram_region               (ram_region),
    .page_zero_area           (low_page_user_read),
    .address_in_installed_ram (address_in_installed_ram),
    .ram_data                 (ram_read_data),
    .page_map_selected        (sel_page_map),
    .page_map_entry           (page_map_entry),
    .general_status_selected  (sel_general_status),
    .gsr                      (gsr),
    .video_ram_selected       (sel_video_ram),
    .video_ram_data           (video_ram_data),
    .bus_status_0_selected    (sel_bus_status_0),
    .bsr0                     (bsr0),
    .bus_status_1_selected    (sel_bus_status_1),
    .bsr1                     (bsr1),
    .telephony_selected       (sel_telephony),
    .telephony_data           (telephony_data),
    .printer_selected         (sel_printer_status),
    .printer_data             (printer_status),
    .disk_selected            (disk_selected),
    .disk_data                (disk_read_data),
    .ethernet_selected        (ethernet_selected),
    .ethernet_data            (ethernet_read_data),
    .modem_control_2_selected (sel_modem_control_2),
    .mcr2                     (mcr2),
    .rtc_selected             (sel_rtc_read),
    .rtc_data                 (rtc_data),
    .serial_selected          (sel_serial),
    .serial_data              (scc_read_data),
    .modem_status_selected    (sel_modem_status),
    .modem_status_data        (modem_status_data),
    .keyboard_selected        (sel_keyboard),
    .keyboard_data            (acia_read_data),
    .cpu_data_in              (cpu_bus_out.data_in)
);

endmodule
