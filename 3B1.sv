/*
    Copyright (c) 2026 by Hrvoje Cavrak

    AT&T UNIX PC 7300 / 3B1 for MiSTer (FPGA only).

    The glue between the MiSTer framework (sys/) and the board in ../rtl: a single 20 MHz clock, a 68010 on fx68k.

      Memory   main RAM (up to 4 MB) in the SDRAM add-on through rtl/ram/sdram_bridge.sv and sdram.sv (sdram16); ROM, video RAM and page map RAM in
               block RAM; the disk images on the SD card through the HPS (rtl/disk/fpga/sd_image_bridge.sv)
      Video    720x348 monochrome bitmap, 20 MHz pixel clock = clk_sys (CE_PIXEL is always 1), CRT look, the monitor's frame and the printer paper
               on the 1280x720 canvas of video.sv
      Input    PS/2 keyboard and mouse from hps_io, translated by rtl/keyboard/kbd_3b1.sv
      Serial   7201 channel A (the RS-232 port) through the HPS UART pins and / or the user port pins; the built-in modem (channel B) reaches MiSTer's
               Hayes modem (MidiLink) on the same UART in the UART mode "Modem"; the Ethernet board has its own path (a mailbox in DDR3 that
               Main_MiSTer serves)
      Sound    telephone tones, call sounds and modem data sound through rtl/audio/telephone_audio.sv
*/
/*
    The core has no ROM in it: the boot ROM is loaded from files (boot0.rom and boot1.rom in the core's folder), the machine stays in reset until
    it has arrived and a notice on the screen says so.
*/

module emu
import board_ports_pkg::*;
(
    `include "sys/emu_ports.vh"
);

/* ---- outputs of the template that this core does not use */
assign ADC_BUS        = 'Z;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign VGA_SL         = 0;
assign VGA_F1         = 0;
assign VGA_SCALER     = 0;
assign VGA_DISABLE    = 0;
assign HDMI_FREEZE    = 0;
assign HDMI_BLACKOUT  = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S   = 1;                   /* signed samples; the audio is mono (telephone_audio below) */
assign AUDIO_MIX = 0;

assign LED_POWER = 0;
assign BUTTONS   = 0;

`include "build_id.v"

/*
    The bits of the OSD status word. CONF_STR below and this table have to stay in step; bump the "v," entry of CONF_STR when bits move. The decoding
    into settings is in osd_settings.sv.

      [0]        Reset (trigger)
      [2:1]      RAM size: 0 = 2 MB, 1 = 4 MB, 2 = 512 KB, 3 = 1 MB
      [3]        Tear off paper (trigger): clears the printout
      [5:4]      Screen colour: 0 = green (default), 1 = white, 2 = amber, 3 = mint (a pale green, RGB 108 235 194)
      [6]        Invert video
      [8:7]      Hard disk geometry: 0 = 17 sectors (136 per cylinder, default), 1 = 17 sectors (272 per cylinder), 2 = 8 heads x 16 sectors
      [9]        Floppy: 0 = 10 sectors per track, 1 = 8 sectors per track
      [11]       Phosphor persistence (4 bit per pixel, exponential fade, dark after 16 frames = 0.27 s): 0 = on (default), 1 = off
      [13]       Screen edges (brightness falloff towards the edges): 0 = on (default), 1 = off
      [12]       Round corners (the corners of the screen rounded by 28 pixels instead of 4): 0 = off (default), 1 = on
      [15:14]    Glow: 0 = mid (default), 1 = high, 2 = low, 3 = off
      [17:16]    Glass (dark level of the tube face, 45 / 60 / 68): 0 = low (default), 1 = mid, 2 = high, 3 = off
      [43]       Curvature (barrel distortion of the picture, fixed strength 0.031): 0 = on (default), 1 = off
      [44]       Monitor frame (the bezel on both sides of the picture): 0 = on (default), 1 = off
      [20:19]    Ethernet card and host interface (read by Main_MiSTer, support/3b1/3b1_eth.cpp): 0 = eth0 (default), 1 = no card, 2 = macvlan
                 child of eth0
      [21]       Serial loopback plug (diagnostic communication test): TxD to RxD, RTS to CTS, DTR to DCD and to the ring input (channel B DCD):
                 0 = off (default)
      [23:22]    Roll band (a bright band drifting down the picture, with a little frame flicker): 0 = low (default), 1 = mid, 2 = high, 3 = off
      [27:26]    Line wobble (single lines jumping sideways for a frame): 0 = mid (default), 1 = high, 2 = low, 3 = off
      [29:28]    Noise (random brightness variation of every pixel): 0 = mid (default), 1 = high, 2 = low, 3 = off
      [31:30]    Glow noise (random variation of the halo of every pixel): 0 = mid (default), 1 = high, 2 = low, 3 = off
      [33:32]    Sound volume (838A tones: beep, DTMF, ring): 0 = mid (default), 1 = high, 2 = low, 3 = off
      [34]       Call sounds (dial tone, ringback, modem handshake, busy; synthesised for the modem bridge): 0 = on (default), 1 = off
      [36:35]    Modem data sound (the line signal of the data bytes, like a modem speaker): 0 = handshake only (default: the call sounds, the connect
                 screech lasts 5 s and then the line is silent), 1 = data sound in both directions, 2 = receive only, 3 = transmit only
      [37]       (unused; the modem sound speed always follows the 882A: 300 / 1200 / 2400)
      [38]       Eject floppy (trigger): the drive reports not ready until a floppy image is mounted again
      [40:39]    Modem flow control (UART_RTS towards MidiLink in modem mode; the HPS side sees 0 = ready, measured with the pre-flow-control core):
                 0 = on (default), 1 = off (always ready = 0), 2 = opposite polarity
      [42:41]    Serial port (channel A): 0 = the MiSTer's UART /dev/ttyS1 (default), 1 = user port (3.3 V TX on USER_IO[1], RX on USER_IO[0],
                 115200 8N1 style pins as in the VT52 core; needs an external RS-232 level shifter), 2 = both (TX goes to both, RX from whichever
                 receives a character)
*/
localparam CONF_STR = {
    "3B1;UART9600:115200:4800:2400:1200:300;",
    "-;",
    "S0,VHD,Mount hard disk;",
    "S1,IMG,Mount floppy;",
    "-;",
    "O[2:1],RAM,2 MB,4 MB,512 KB,1 MB;",
    "O[8:7],Hard disk geometry,17 sect (136),17 sect (272),16 sect;",
    "O[9],Floppy,10 sect,8 sect;",
    "-;",
    "P1,CRT effects;",
    "P1O[5:4],Screen colour,Green,White,Amber,Mint;",
    "P1O[6],Invert video,No,Yes;",
    "P1O[11],Phosphor,On,Off;",
    "P1O[15:14],Glow,Mid,High,Low,Off;",
    "P1O[17:16],Glass,Low,Mid,High,Off;",
    "P1O[43],Curvature,On,Off;",
    "P1O[44],Monitor frame,On,Off;",
    "P1O[23:22],Roll band,Low,Mid,High,Off;",
    "P1O[27:26],Line wobble,Mid,High,Low,Off;",
    "P1O[29:28],Noise,Mid,High,Low,Off;",
    "P1O[31:30],Glow noise,Mid,High,Low,Off;",
    "P1O[13],Screen edges,On,Off;",
    "P1O[12],Round corners,Off,On;",
    "O[20:19],Ethernet,eth0,Off,macvlan;",
    "O[21],Serial loopback plug,Off,On;",
    "O[42:41],Serial port,MiSTer UART,User port,Both;",
    "O[33:32],Sound volume,Mid,High,Low,Off;",
    "O[34],Call sounds,On,Off;",
    "O[36:35],Modem sound,Handshake,Both,Receive,Transmit;",
    "O[40:39],Modem flow control,On,Off,Opposite;",
    "T[3],Tear off paper;",
    "T[38],Eject floppy;",
    "-;",
    "T[0],Reset;",
    "R[0],Reset and close OSD;",
    "v,4;",
    "V,v",`BUILD_DATE
};

/* ---- the framework: hps_io brings the OSD status, the PS/2 devices, the virtual drives, file downloads, the UART mode and the host time */
wire         forced_scandoubler;
wire   [1:0] buttons;
wire [127:0] status;
wire  [10:0] ps2_key;
wire  [24:0] ps2_mouse;
wire  [64:0] host_time;

wire   [7:0] uart_mode;
wire  [31:0] uart_speed;

wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire [15:0] ioctl_dout;

/* the virtual drives: 0 = hard disk, 1 = floppy (512 byte blocks, 16 bit buffer) */
wire [31:0] sd_lba [2];
wire  [5:0] sd_blk_cnt [2];
wire  [1:0] sd_rd;
wire  [1:0] sd_wr;
wire  [1:0] sd_ack;
wire [12:0] sd_buff_addr;
wire [15:0] sd_buff_dout;
wire [15:0] sd_buff_din [2];
wire        sd_buff_wr;
wire  [1:0] img_mounted;
wire        img_readonly;
wire [63:0] img_size;

hps_io #(.CONF_STR(CONF_STR), .WIDE(1), .VDNUM(2), .BLKSZ(2)) hps_io
(
    .clk_sys            (clk_sys),
    .HPS_BUS            (HPS_BUS),
    .EXT_BUS            (),
    .gamma_bus          (),

    .forced_scandoubler (forced_scandoubler),

    .buttons            (buttons),
    .status             (status),
    .status_menumask    (16'd0),

    .ps2_key            (ps2_key),
    .ps2_mouse          (ps2_mouse),

    .sd_lba             (sd_lba),
    .sd_blk_cnt         (sd_blk_cnt),
    .sd_rd              (sd_rd),
    .sd_wr              (sd_wr),
    .sd_ack             (sd_ack),
    .sd_buff_addr       (sd_buff_addr),
    .sd_buff_dout       (sd_buff_dout),
    .sd_buff_din        (sd_buff_din),
    .sd_buff_wr         (sd_buff_wr),
    .img_mounted        (img_mounted),
    .img_readonly       (img_readonly),
    .img_size           (img_size),

    .ioctl_download     (ioctl_download),
    .ioctl_index        (ioctl_index),
    .ioctl_wr           (ioctl_wr),
    .ioctl_addr         (ioctl_addr),
    .ioctl_dout         (ioctl_dout),
    .ioctl_wait         (1'b0),

    .RTC                (host_time),
    .uart_mode          (uart_mode),
    .uart_speed         (uart_speed),

    /* force feedback (FFB=0 here, so Main never asks for it): tied off; the analog triggers are not used by the core (left open) */
    .joystick_0_ffb     (56'd0),
    .joystick_1_ffb     (56'd0),
    .joystick_2_ffb     (56'd0),
    .joystick_3_ffb     (56'd0),
    .joystick_4_ffb     (56'd0),
    .joystick_5_ffb     (56'd0)
);

/* ---- the board's port bundles (board_ports_pkg.sv): the other modules connect straight to their fields */
cpu_bus_from_cpu_t cpu_bus_in;
cpu_bus_to_cpu_t cpu_bus_out;
keyboard_to_board_t keyboard_in;
keyboard_from_board_t keyboard_out;
board_configuration_t configuration;
sdram_request_t sdram_request;
sdram_response_t sdram_response;
disk_image_request_t disk_image_request;
disk_image_response_t disk_image_response;
serial_from_board_t serial_out;
serial_to_board_t serial_in;
printer_from_board_t printer_out;
printer_to_board_t printer_in;
telephone_to_board_t telephone_in;
telephone_from_board_t telephone_out;
boot_rom_write_t boot_rom_write;
ethernet_phy_to_board_t ethernet_phy_in;
ethernet_phy_from_board_t ethernet_phy_out;

/* ---- the OSD status word decoded into settings (hps_io, the status word, ps2_key and ps2_mouse all run on clk_sys: no clock crossing) */
wire        osd_reset_request;
wire        osd_tear_off_paper;
wire        osd_ethernet_present;
wire        osd_eject_floppy;
wire        osd_serial_to_hps_uart;
wire        osd_serial_to_user_port;
wire  [1:0] osd_modem_flow_control;
wire  [1:0] osd_sound_volume;
wire        osd_call_sounds_on;
wire  [1:0] osd_data_sound_select;
wire        osd_invert_video;
wire  [1:0] osd_screen_colour;
wire        osd_phosphor_on;
wire  [1:0] osd_glow_mode;
wire  [1:0] osd_glow_noise;
wire        osd_edges_on;
wire        osd_round_corners;
wire  [1:0] osd_roll_band;
wire  [1:0] osd_line_wobble;
wire  [1:0] osd_noise;
wire        osd_curvature_on;
wire        osd_bezel_on;
wire  [7:0] osd_glass_floor_level;

osd_settings osd
(
    .status               (status),
    .reset_request        (osd_reset_request),
    .ram_size             (configuration.ram_size),
    .hard_disk_geometry   (configuration.hard_disk_geometry),
    .floppy_eight_sectors (configuration.floppy_eight_sectors),
    .tear_off_paper       (osd_tear_off_paper),
    .serial_loopback      (serial_in.loopback),
    .ethernet_present     (osd_ethernet_present),
    .eject_floppy_request (osd_eject_floppy),
    .serial_to_hps_uart   (osd_serial_to_hps_uart),
    .serial_to_user_port  (osd_serial_to_user_port),
    .modem_flow_control   (osd_modem_flow_control),
    .sound_volume         (osd_sound_volume),
    .call_sounds_on       (osd_call_sounds_on),
    .data_sound_select    (osd_data_sound_select),
    .invert_video         (osd_invert_video),
    .screen_colour        (osd_screen_colour),
    .phosphor_on          (osd_phosphor_on),
    .glow_mode            (osd_glow_mode),
    .glow_noise           (osd_glow_noise),
    .edges_on             (osd_edges_on),
    .round_corners        (osd_round_corners),
    .roll_band            (osd_roll_band),
    .line_wobble          (osd_line_wobble),
    .noise                (osd_noise),
    .curvature_on         (osd_curvature_on),
    .bezel_on             (osd_bezel_on),
    .glass_floor_level    (osd_glass_floor_level)
);

/* the shape of the canvas, 1280x720 (16:9): the scaler makes it 1920x1080 without bars */
assign VIDEO_ARX = 12'd1280;
assign VIDEO_ARY = 12'd720;

/* ---- clocks */
wire clk_sys;               /* 20 MHz: the CPU, the board, the video */
wire clk_sd;                /* 100 MHz: the SDRAM controller */
wire clk_hd;                /* 75 MHz: the video canvas (video.sv) */
wire pll_locked;

pll pll
(
    .refclk   (CLK_50M),
    .rst      (0),
    .outclk_0 (clk_sys),
    .outclk_1 (clk_sd),
    .outclk_2 (clk_hd),
    .locked   (pll_locked)
);

/* ---- boot ROM from a file */
/*
    The 16 KB boot ROM is not in the core: it is the two 8 KB EPROMs of the board, 15C and 14C, each loaded from a file. The files boot0.rom (15C) and boot1.rom (14C) in the core's folder are loaded by Main_MiSTer at core start; they
    stay until the core is loaded again. The machine is held in reset until a file has arrived,
    during the transfer and for the usual reset time afterwards (rtl/rom/rom_loader.sv clears the part of the ROM the file did not cover). While there
    is no ROM, the picture is a notice that says so (boot_rom_notice.sv).
*/
/* ---- the boot ROM, from two files */
boot_rom_files boot_rom_files
(
    .clk_sys          (clk_sys),
    .ioctl_download   (ioctl_download),
    .ioctl_wr         (ioctl_wr),
    .ioctl_addr       (ioctl_addr),
    .ioctl_dout       (ioctl_dout),
    .ioctl_index      (ioctl_index),
    .rom_high_write   (boot_rom_write.write_enable_high),
    .rom_high_address (boot_rom_write.address_high),
    .rom_high_data    (boot_rom_write.data_high),
    .rom_low_write    (boot_rom_write.write_enable_low),
    .rom_low_address  (boot_rom_write.address_low),
    .rom_low_data     (boot_rom_write.data_low),
    .rom_loading      (rom_loading),
    .boot_rom_ready   (boot_rom_ready)
);

/* ---- reset */
wire rtc_ready;
wire reset_stretched;
wire reset;

wire floppy_swap_reset;
wire reset_request = RESET | osd_reset_request | buttons[1] | ~pll_locked | img_mounted[0] | rom_loading | ~boot_rom_ready | floppy_swap_reset;

reset_sequencer reset_control
(
    .clk             (clk_sys),
    .reset_request   (reset_request),
    .rtc_ready       (rtc_ready),
    .reset_stretched (reset_stretched),
    .reset           (reset)             /* held until the host time has been loaded into the RTC (rtl/rtc/rtc_conv.sv) */
);

/* ---- floppy: an image is mounted (the drive reports "not ready" otherwise), and the OSD can eject it */
wire [1:0] image_present;
wire [1:0] image_read_only;

floppy_eject floppy_ejector
(
    .clk                  (clk_sys),
    .eject_request        (osd_eject_floppy),
    .floppy_image_mounted (img_mounted[1]),
    .floppy_image_present (image_present[1]),
    .floppy_present       (configuration.floppy_present),
    .reset_after_swap     (floppy_swap_reset)
);

/* ---- the real time clock gets the host time */
wire        rtc_set_enable;
wire [51:0] rtc_set_value;

rtc_conv host_time_converter
(
    .clk     (clk_sys),
    .rst     (reset_stretched),
    .rtc     (host_time),
    .set_en  (rtc_set_enable),
    .set_val (rtc_set_value),
    .ready   (rtc_ready)
);

/* ---- the CPU */
wire [23:0] cpu_addr;
wire        cpu_reset_out_n;
wire        cpu_halted_n;

cpu_fx68k cpu
(
    .clk         (clk_sys),
    .reset       (reset),
    .din         (cpu_bus_out.data_in),
    .dout        (cpu_bus_in.data_out),
    .addr        (cpu_addr),
    .as_n        (cpu_bus_in.address_strobe_n),
    .uds_n       (cpu_bus_in.upper_data_strobe_n),
    .lds_n       (cpu_bus_in.lower_data_strobe_n),
    .rw_n        (cpu_bus_in.read_not_write),
    .fc          (cpu_bus_in.function_code),
    .dtack_n     (cpu_bus_out.data_acknowledge_n),
    .berr_n      (cpu_bus_out.bus_error_n),
    .vpa_n       (cpu_bus_out.valid_peripheral_address_n),
    .ipl_n       (cpu_bus_out.interrupt_priority_n),
    .br_n        (cpu_bus_out.bus_request_n),
    .bgack_n     (cpu_bus_out.bus_grant_acknowledge_n),
    .bg_n        (cpu_bus_in.bus_grant_n),
    .reset_out_n (cpu_reset_out_n),
    .halted_n    (cpu_halted_n)
);

/* ---- keyboard and mouse */

kbd_3b1 keyboard
(
    .clk        (clk_sys),
    .rst        (reset),
    .ps2_key    (ps2_key),
    .ps2_mouse  (ps2_mouse),
    .tx_valid   (keyboard_out.byte_valid),
    .tx_data    (keyboard_out.byte_data),
    .kbd_data   (keyboard_in.key_code),
    .kbd_strobe (keyboard_in.key_strobe)
);

/* ---- serial channels */

/* the parallel printer port of the board */

/* the telephony registers of the board, for the built-in modem and the sounds */
/* ---- serial channel A on the MiSTer UART / user port, and the built-in modem on channel B */
serial_ports serial_ports
(
    .clk_sys                  (clk_sys),
    .reset                    (reset),
    .uart_mode                (uart_mode),
    .uart_speed               (uart_speed),
    .osd_serial_to_hps_uart   (osd_serial_to_hps_uart),
    .osd_serial_to_user_port  (osd_serial_to_user_port),
    .osd_modem_flow_control   (osd_modem_flow_control),
    .ser_tx_valid             (serial_out.tx_valid),
    .ser_tx_a                 (serial_out.tx_data_a),
    .ser_tx_b                 (serial_out.tx_data_b),
    .ser_rx_ready             (serial_out.rx_ready),
    .ser_dtr                  (serial_out.data_terminal_ready),
    .telephone_hook_closed    (telephone_out.hook_line_1),
    .dialer_toggle            (telephone_out.dial_toggle),
    .dialer_word              (telephone_out.dial_word),
    .modem_chip_w0            (telephone_out.modem_register_0),
    .modem_chip_w1            (telephone_out.modem_register_1),
    .modem_chip_w8            (telephone_out.modem_register_8),
    .channel_a_rx_valid       (channel_a_rx_valid),
    .channel_a_rx_data        (serial_in.rx_data_a),
    .modem_channel_b_rx_valid (modem_channel_b_rx_valid),
    .modem_channel_b_rx_data  (serial_in.rx_data_b),
    .modem_mode               (modem_mode),
    .modem_call_state         (modem_call_state),
    .modem_digits_dialled     (modem_digits_dialled),
    .modem_carrier            (modem_carrier),
    .UART_RXD                 (UART_RXD),
    .UART_TXD                 (UART_TXD),
    .UART_RTS                 (UART_RTS),
    .UART_DTR                 (UART_DTR),
    .USER_IN                  (USER_IN),
    .USER_OUT                 (USER_OUT)
);

/* ---- sound */
wire signed [15:0] sound;

telephone_audio #(.CLK_HZ(20000000)) telephone_sound
(
    .clk                  (clk_sys),
    .reset                (reset),
    .master_volume        (osd_sound_volume),
    .call_sounds_enabled  (modem_mode & osd_call_sounds_on),
    .modem_mode           (modem_mode),
    .data_sound_select    (osd_data_sound_select),
    .dialer_word          (telephone_out.dial_word),
    .modem_call_state     (modem_call_state),
    .digits_dialled       (modem_digits_dialled),
    .carrier              (modem_carrier),
    .line_speed_1200      (telephone_out.modem_register_1[4]),
    .line_speed_2400      (telephone_out.modem_register_8[3]),
    .guest_transmit_valid (serial_out.tx_valid[1]),
    .guest_transmit_data  (serial_out.tx_data_b),
    .guest_receive_valid  (modem_channel_b_rx_valid),
    .guest_receive_data   (serial_in.rx_data_b),
    .audio_out            (sound)
);


assign AUDIO_L = sound;
assign AUDIO_R = sound;

/* ---- the board */
wire [13:0] video_ram_address;
wire [15:0] video_ram_data;

/* the main RAM in the SDRAM, through sdram_bridge (the toggle handshake) */

/* the 64 bit image port of the disk controllers, served by sd_image_bridge */
wire        disk_image_activity;

wire        dma_active;
wire  [3:0] front_panel_leds;                   /* 0 = lit */

/* Ethernet: frames go through a mailbox in DDR3 to Main_MiSTer (support/3b1/3b1_eth.cpp), which puts them on the network */
wire       eth_activity;

eth_ddr_mbox ethernet_mailbox
(
    .clk          (clk_sys),
    .rst          (reset),
    .tx_vld       (ethernet_phy_out.tx_valid),
    .tx_dat       (ethernet_phy_out.tx_data),
    .tx_end       (ethernet_phy_out.tx_end),
    .rx_vld       (ethernet_phy_in.rx_valid),
    .rx_dat       (ethernet_phy_in.rx_data),
    .rx_end       (ethernet_phy_in.rx_end),
    .rx_rdy       (ethernet_phy_out.rx_ready),
    .ddr_busy     (DDRAM_BUSY),
    .ddr_burstcnt (DDRAM_BURSTCNT),
    .ddr_addr     (DDRAM_ADDR),
    .ddr_dout     (DDRAM_DOUT),
    .ddr_dready   (DDRAM_DOUT_READY),
    .ddr_rd       (DDRAM_RD),
    .ddr_din      (DDRAM_DIN),
    .ddr_be       (DDRAM_BE),
    .ddr_we       (DDRAM_WE),
    .activity     (eth_activity)
);


assign DDRAM_CLK = clk_sys;

/* ---- the port bundles of the board: the fields that are not simply a wire of another module, or are tied */
assign cpu_bus_in.address                    = cpu_addr[23:1];
assign configuration.floppy_write_protected     = image_read_only[1];
assign serial_in.rx_valid                   = {modem_channel_b_rx_valid, channel_a_rx_valid};
assign serial_in.carrier_detect             = {modem_carrier, 1'b1};
assign serial_in.clear_to_send              = 2'b11;
assign telephone_in.ring_line_1                = 1'b0;
assign telephone_in.ring_line_2                = 1'b0;
assign telephone_in.external_off_hook          = 1'b0;

unixpc_board #(.ROM_BUILT_IN(0), .EXT_RAM(1), .PARITY(1)) board
(
    .clk                  (clk_sys),
    .rst                  (reset),
    .rtc_set_en           (rtc_set_enable),
    .rtc_set_val          (rtc_set_value),

    .cpu_bus_in            (cpu_bus_in),
    .cpu_bus_out           (cpu_bus_out),
    .keyboard_in           (keyboard_in),
    .keyboard_out          (keyboard_out),
    .configuration         (configuration),
    .sdram_request         (sdram_request),
    .sdram_response        (sdram_response),
    .disk_image_request    (disk_image_request),
    .disk_image_response   (disk_image_response),
    .serial_out            (serial_out),
    .serial_in             (serial_in),
    .printer_out           (printer_out),
    .printer_in            (printer_in),
    .telephone_in          (telephone_in),
    .telephone_out         (telephone_out),
    .boot_rom_write        (boot_rom_write),
    .ethernet_phy_in       (ethernet_phy_in),
    .ethernet_phy_out      (ethernet_phy_out),

    .video_scan_address   (video_ram_address),
    .video_scan_data      (video_ram_data),
    .eth_present          (osd_ethernet_present),
    .leds                 (front_panel_leds),
    .dma_active           (dma_active)
);

/* ---- main RAM in the SDRAM */
main_ram main_ram
(
    .clk_sys                   (clk_sys),
    .clk_sd                    (clk_sd),
    .reset                     (reset),
    .sdram_request_toggle      (sdram_request.request_toggle),
    .sdram_write               (sdram_request.write),
    .sdram_word_address        (sdram_request.word_address),
    .sdram_byte_enables        (sdram_request.byte_enables),
    .sdram_write_data          (sdram_request.write_data),
    .sdram_parity_flags_in_use (sdram_request.parity_flags_in_use),
    .sdram_store_bad_parity    (sdram_request.store_bad_parity),
    .sdram_read_data           (sdram_response.read_data),
    .sdram_bad_parity_flags    (sdram_response.bad_parity_flags),
    .sdram_acknowledge_toggle  (sdram_response.acknowledge_toggle),
    .sdram_fast_ok             (sdram_response.fast_ok),
    .SDRAM_DQ                  (SDRAM_DQ),
    .SDRAM_A                   (SDRAM_A),
    .SDRAM_DQML                (SDRAM_DQML),
    .SDRAM_DQMH                (SDRAM_DQMH),
    .SDRAM_BA                  (SDRAM_BA),
    .SDRAM_nCS                 (SDRAM_nCS),
    .SDRAM_nWE                 (SDRAM_nWE),
    .SDRAM_nRAS                (SDRAM_nRAS),
    .SDRAM_nCAS                (SDRAM_nCAS),
    .SDRAM_CKE                 (SDRAM_CKE),
    .SDRAM_CLK                 (SDRAM_CLK)
);

/*
    ---- disk images on the SD card (hps_io virtual drives) The disk controllers keep their 64 bit DDR style image port; the bridge serves it from a
    sector cache per drive that is filled from, and written back to, the image file through the HPS. The DDR3 interface is not used for this; DDRAM
    belongs to the Ethernet mailbox above.
*/
sd_image_bridge disk_image_store
(
    .clk          (clk_sys),
    .rst          (reset),
    .ddr_rd       (disk_image_request.read),
    .ddr_we       (disk_image_request.write),
    .ddr_addr     (disk_image_request.address),
    .ddr_din      (disk_image_request.write_data),
    .ddr_be       (disk_image_request.byte_enables),
    .ddr_dout     (disk_image_response.read_data),
    .ddr_ready    (disk_image_response.ready),
    .ddr_busy     (disk_image_response.busy),
    .sd_lba0      (sd_lba[0]),
    .sd_lba1      (sd_lba[1]),
    .sd_rd        (sd_rd),
    .sd_wr        (sd_wr),
    .sd_ack       (sd_ack),
    .sd_buff_addr (sd_buff_addr),
    .sd_buff_dout (sd_buff_dout),
    .sd_buff_wr   (sd_buff_wr),
    .sd_buff_din0 (sd_buff_din[0]),
    .sd_buff_din1 (sd_buff_din[1]),
    .img_mounted  (img_mounted),
    .img_size     (img_size),
    .img_readonly (img_readonly),
    .present      (image_present),
    .read_only    (image_read_only),
    .activity     (disk_image_activity)
);

assign sd_blk_cnt[0] = 6'd0;
assign sd_blk_cnt[1] = 6'd0;

/* ---- the picture: raster, notice, CRT effects, printer paper and the 1280x720 canvas */
video_output display
(
    .clk_sys               (clk_sys),
    .clk_hd                (clk_hd),
    .reset                 (reset),
    .boot_rom_ready        (boot_rom_ready),
    .ps2_key               (ps2_key),
    .video_ram_address     (video_ram_address),
    .video_ram_data        (video_ram_data),
    .lp_wr_data            (printer_out.data_written),
    .lp_data               (printer_out.data),
    .lp_wr_mcr             (printer_out.control_written),
    .lp_mcr13              (printer_out.strobe),
    .lp_busy               (printer_in.busy),
    .lp_select             (printer_in.select),
    .lp_nopaper            (printer_in.no_paper),
    .lp_int                (printer_in.acknowledge_interrupt),
    .front_panel_leds      (front_panel_leds),
    .osd_tear_off_paper    (osd_tear_off_paper),
    .osd_invert_video      (osd_invert_video),
    .osd_phosphor_on       (osd_phosphor_on),
    .osd_glow_mode         (osd_glow_mode),
    .osd_glow_noise        (osd_glow_noise),
    .osd_noise             (osd_noise),
    .osd_bezel_on          (osd_bezel_on),
    .osd_curvature_on      (osd_curvature_on),
    .osd_line_wobble       (osd_line_wobble),
    .osd_screen_colour     (osd_screen_colour),
    .osd_glass_floor_level (osd_glass_floor_level),
    .osd_edges_on          (osd_edges_on),
    .osd_round_corners     (osd_round_corners),
    .osd_roll_band         (osd_roll_band),
    .VGA_R                 (VGA_R),
    .VGA_G                 (VGA_G),
    .VGA_B                 (VGA_B),
    .VGA_HS                (VGA_HS),
    .VGA_VS                (VGA_VS),
    .VGA_DE                (VGA_DE),
    .CLK_VIDEO             (CLK_VIDEO),
    .CE_PIXEL              (CE_PIXEL)
);

/* ---- LEDs */
assign LED_USER = front_panel_leds[0];
assign LED_DISK = {1'b1, dma_active | disk_image_activity | eth_activity};

endmodule
