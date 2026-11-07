/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Address decoder of the logic board (PAL equations on sheet 4, here as plain comparisons).

    The CPU sees RAM at 000000..3FFFFF, the page map and the board's registers at 400000..4FFFFF, ROM at 800000..BFFFFF, the expansion slots at
    C00000..DFFFFF and the serial, modem, RTC and keyboard chips at E00000..FFFFFF. Until the CPU sets ROMLMAP in the general control register, the
    ROM is forced over the whole low 4 MB (A23 is treated as 1): that is how the 68010 finds its reset vector in ROM. The decoder works on that
    "effective" address.
*/
module board_address_decoder
(
    input  [23:0] cpu_address,                  /* A23..A0, A0 is 0 (the CPU has only A23..A1) */
    input         rom_low_map,                  /* GCR ROMLMAP+: 0 = ROM forced into the low 4 MB */

    output [23:0] effective_address,            /* the address after ROMLMAP forcing */
    output [7:0]  high_byte,                    /* effective_address[23:16] */

    output        ram_region,                   /* 000000..3FFFFF */
    output        rom_region,                   /* 800000..BFFFFF */

    output        page_map,                     /* 40xxxx: page map RAM */
    output        general_status,               /* 41xxxx: GSR */
    output        video_ram,                    /* 42xxxx */
    output        bus_status_0,                 /* 43xxxx: BSR0 */
    output        bus_status_1,                 /* 44xxxx: BSR1 */
    output        telephony_status,             /* 45xxxx */
    output        printer_status,               /* 47xxxx and 4Axxxx */
    output        modem_control,                /* 4Axxxx: MCR */
    output        clear_status,                 /* 4Cxxxx: CSR, writing it clears the status registers */
    output        printer_data,                 /* 4Fxxxx */
    output        rtc_write,                    /* 48xxxx */
    output        modem_control_2,              /* E2xxxx: MCR2 */
    output        rtc_read,                     /* E3xxxx */
    output        general_control,              /* E4xxxx: GCR, an addressable latch */
    output        serial_7201,                  /* E5xxxx */
    output        modem_status,                 /* E6xxxx */
    output        keyboard_6850                 /* the keyboard ACIA: E70000 and E70002 (the decoding is loose, see below) */
);

assign effective_address = {cpu_address[23] | ~rom_low_map, cpu_address[22:0]};
assign high_byte         = effective_address[23:16];

assign ram_region        = ~effective_address[23] & ~effective_address[22];
assign rom_region        =  effective_address[23] & ~effective_address[22];

assign page_map          = (high_byte == 8'h40);
assign general_status    = (high_byte == 8'h41);
assign video_ram         = (high_byte == 8'h42);
assign bus_status_0      = (high_byte == 8'h43);
assign bus_status_1      = (high_byte == 8'h44);
assign telephony_status  = (high_byte == 8'h45);
assign printer_status    = (high_byte == 8'h47) | (high_byte == 8'h4a);
assign modem_control     = (high_byte == 8'h4a);
assign clear_status      = (high_byte == 8'h4c);
assign printer_data      = (high_byte == 8'h4f);
assign rtc_write         = (high_byte == 8'h48);
assign modem_control_2   = (high_byte == 8'he2);
assign rtc_read          = (high_byte == 8'he3);
assign general_control   = (high_byte == 8'he4);
assign serial_7201       = (high_byte == 8'he5);
assign modem_status      = (high_byte == 8'he6);

/*
    The keyboard ACIA decodes loosely: A23..A21 = 111, A18..A16 = 111 and A15..A2 = 0, so its two registers sit at E70000 and E70002 (and at mirrors
    in E7, EF, F7, FF).
*/
assign keyboard_6850     = (high_byte[7:5] == 3'b111) && (high_byte[2:0] == 3'b111) && (cpu_address[15:2] == 14'd0);

endmodule
