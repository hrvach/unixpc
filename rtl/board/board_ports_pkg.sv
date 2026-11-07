/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The groups of signals that cross the edge of the board (unixpc_board.sv): one packed struct per direction of each group, so that the board has a
    handful of ports instead of a hundred. Fields run from the most significant bit down in the order below; each is named for what the signal is.
*/
package board_ports_pkg;

    /* 68010 bus, signals from the CPU */
    typedef struct packed {
        logic [22:0] address;        /* word address A23..A1 */
        logic [15:0] data_out;        /* data the CPU drives */
        logic address_strobe_n;
        logic upper_data_strobe_n;
        logic lower_data_strobe_n;
        logic read_not_write;        /* high = read */
        logic [2:0] function_code;
        logic bus_grant_n;
    } cpu_bus_from_cpu_t;

    /* 68010 bus, signals to the CPU */
    typedef struct packed {
        logic [15:0] data_in;
        logic data_acknowledge_n;
        logic bus_error_n;
        logic valid_peripheral_address_n;
        logic [2:0] interrupt_priority_n;
        logic bus_request_n;
        logic bus_grant_acknowledge_n;
    } cpu_bus_to_cpu_t;

    /* keyboard: a 3B1 key code and its strobe */
    typedef struct packed {
        logic [7:0] key_code;
        logic key_strobe;
    } keyboard_to_board_t;

    /* keyboard: the byte the CPU wrote to the keyboard ACIA */
    typedef struct packed {
        logic byte_valid;
        logic [7:0] byte_data;
    } keyboard_from_board_t;

    /* disk DMA debug view and self-check (disk_dma_debug.sv) */
    typedef struct packed {
        logic transfer_toggle;        /* toggles at the end of every transfer */
        logic [27:0] write_timing;        /* longest image write, hard disk status reads */
        logic [15:0] live_flags;
        logic [23:0] last_transfer;
        logic [31:0] status_and_count;
        logic [2:0] transfer_flags;        /* {fault, floppy, RAM to disk} */
        logic [127:0] register_log;        /* the last 8 register writes */
    } disk_dma_debug_t;

    /* configuration (same encoding as the MiSTer OSD) */
    typedef struct packed {
        logic [1:0] ram_size;        /* 0 = 512K, 1 = 1M, 2 = 2M, 3 = 4M */
        logic [1:0] hard_disk_geometry;
        logic floppy_eight_sectors;
        logic floppy_present;
        logic floppy_write_protected;        /* the floppy image is read-only */
    } board_configuration_t;

    /* external main RAM (EXT_RAM = 1), requests; see rtl/ram/sdram_bridge.sv */
    typedef struct packed {
        logic request_toggle;        /* a transaction starts when this flips */
        logic write;
        logic [20:0] word_address;
        logic [1:0] byte_enables;        /* bit 1 = D15..D8 */
        logic [15:0] write_data;
        logic parity_flags_in_use;        /* from the first write with bad parity forced */
        logic store_bad_parity;        /* the value to store into the flags of written bytes */
    } sdram_request_t;

    /* external main RAM, answers */
    typedef struct packed {
        logic [15:0] read_data;
        logic acknowledge_toggle;        /* equals request_toggle when the transaction is done */
        logic [1:0] bad_parity_flags;        /* flags of the bytes just read */
    } sdram_response_t;

    /* disk image storage (64 bit DDRAM style port), requests */
    typedef struct packed {
        logic read;
        logic write;
        logic [28:0] address;
        logic [63:0] write_data;
        logic [7:0] byte_enables;
    } disk_image_request_t;

    /* disk image storage, answers */
    typedef struct packed {
        logic [63:0] read_data;
        logic ready;
        logic busy;
    } disk_image_response_t;

    /* serial lines, from the 7201 (channel A = RS-232 port, channel B = modem; character level, see upd7201.sv) */
    typedef struct packed {
        logic [1:0] tx_valid;        /* bit 0 = A, bit 1 = B */
        logic [7:0] tx_data_a;
        logic [7:0] tx_data_b;
        logic [1:0] rx_ready;
        logic [1:0] data_terminal_ready;
        logic [1:0] request_to_send;
        logic [63:0] debug_view;        /* 7201 debug view for the overlay */
    } serial_from_board_t;

    /* serial lines, to the 7201 */
    typedef struct packed {
        logic [1:0] rx_valid;
        logic [7:0] rx_data_a;
        logic [7:0] rx_data_b;
        logic [1:0] carrier_detect;        /* 1 = asserted */
        logic [1:0] clear_to_send;
        logic loopback;
    } serial_to_board_t;

    /* Centronics printer port (rtl/printer/lp_printer.sv), signals from the board */
    typedef struct packed {
        logic data_written;        /* pulse: the CPU wrote the data register 4F0000 */
        logic control_written;        /* pulse: the CPU wrote the MCR */
        logic strobe;        /* MCR bit 13 */
        logic [7:0] data;
    } printer_from_board_t;

    /* Centronics printer port, status to the board */
    typedef struct packed {
        logic busy;
        logic select;
        logic no_paper;
        logic acknowledge_interrupt;
    } printer_to_board_t;

    /* telephone line (harness side of the modem): incoming call and external hook */
    typedef struct packed {
        logic ring_line_1;
        logic ring_line_2;
        logic external_off_hook;
    } telephone_to_board_t;

    /* telephone line: hook state, dialled words, modem registers */
    typedef struct packed {
        logic hook_line_1;
        logic hook_line_2;
        logic handset;
        logic dial_toggle;
        logic [15:0] dial_word;
        logic [15:0] modem_register_1;
        logic [15:0] modem_register_8;
        logic [15:0] modem_register_0;
        logic [7:0] modem_baud_code;
    } telephone_from_board_t;

    /* boot ROM write ports, one per EPROM (replace the ROM from files, rtl/rom/rom_loader.sv) */
    typedef struct packed {
        logic write_enable_high;
        logic [12:0] address_high;
        logic [7:0] data_high;
        logic write_enable_low;
        logic [12:0] address_low;
        logic [7:0] data_low;
    } boot_rom_write_t;

    /* Ethernet expansion board: frame byte stream from the PHY */
    typedef struct packed {
        logic rx_valid;
        logic [7:0] rx_data;
        logic rx_end;
    } ethernet_phy_to_board_t;

    /* Ethernet expansion board: frame byte stream to the PHY */
    typedef struct packed {
        logic tx_valid;
        logic [7:0] tx_data;
        logic tx_end;
        logic rx_ready;        /* idle: a frame may start */
    } ethernet_phy_from_board_t;

endpackage
