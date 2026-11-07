/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The CPU's data input mux (FPGA only: on the real board each device drives the bus through its own buffers and the address decoder PALs enable one
    of them).

    The first row that matches wins. Nothing selected reads FFFF (a floating bus with pull-ups).
*/
module board_read_mux
(
    /* RAM and ROM */
    input         rom_region,
    input  [15:0] rom_data,
    input         ram_region,
    input         page_zero_area,               /* user read of page zero: reads 0 */
    input         address_in_installed_ram,
    input  [15:0] ram_data,

    /* the register blocks of 4x0000 */
    input         page_map_selected,
    input  [15:0] page_map_entry,
    input         general_status_selected,
    input  [15:0] gsr,
    input         video_ram_selected,
    input  [15:0] video_ram_data,
    input         bus_status_0_selected,
    input  [15:0] bsr0,
    input         bus_status_1_selected,
    input  [15:0] bsr1,
    input         telephony_selected,
    input  [15:0] telephony_data,
    input         printer_selected,
    input  [15:0] printer_data,

    /* devices with their own address decoding */
    input         disk_selected,
    input  [15:0] disk_data,
    input         ethernet_selected,
    input  [15:0] ethernet_data,

    /* E2xxxx..E7xxxx */
    input         modem_control_2_selected,
    input  [15:0] mcr2,
    input         rtc_selected,
    input  [15:0] rtc_data,
    input         serial_selected,
    input  [7:0]  serial_data,                  /* the 7201 is on D7..D0; it is mirrored on D15..D8 */
    input         modem_status_selected,
    input  [15:0] modem_status_data,
    input         keyboard_selected,
    input  [7:0]  keyboard_data,                /* the ACIA is on D15..D8 in the register map but mirrored on both bytes */

    output reg [15:0] cpu_data_in
);

always @* begin
    cpu_data_in = 16'hffff;

    if (rom_region)                     cpu_data_in = rom_data;
    else if (ram_region) begin
        if (page_zero_area)
            cpu_data_in = 16'h0000;
        else if (address_in_installed_ram)
            cpu_data_in = ram_data;
    end
    else if (page_map_selected)         cpu_data_in = page_map_entry;
    else if (general_status_selected)   cpu_data_in = gsr;
    else if (video_ram_selected)        cpu_data_in = video_ram_data;
    else if (bus_status_0_selected)     cpu_data_in = bsr0;
    else if (bus_status_1_selected)     cpu_data_in = bsr1;
    else if (telephony_selected)        cpu_data_in = telephony_data;
    else if (printer_selected)          cpu_data_in = printer_data;
    else if (disk_selected)             cpu_data_in = disk_data;
    else if (ethernet_selected)         cpu_data_in = ethernet_data;
    else if (modem_control_2_selected)  cpu_data_in = mcr2;
    else if (rtc_selected)              cpu_data_in = rtc_data;
    else if (serial_selected)           cpu_data_in = {serial_data, serial_data};
    else if (modem_status_selected)     cpu_data_in = modem_status_data;
    else if (keyboard_selected)         cpu_data_in = {keyboard_data, keyboard_data};
end

endmodule
