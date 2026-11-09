/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Decoding of the OSD status word into the settings the core uses (FPGA only). The status word comes from hps_io, which runs on clk_sys, so there is
    no clock crossing; everything here is combinational. The bit map of the status word and the OSD menu (CONF_STR) are in 3B1.sv; the two have to
    stay in step.

    Most menu entries list their choices in the order the user sees them, with the default first, so that a cleared status word gives the default
    setup. The tables below turn the position in the menu into the value the core wants (for example "Mid, Low, High, Off" into 2, 1, 3, 0).
*/
module osd_settings
(
    input  [127:0] status,

    /* machine */
    output         reset_request,               /* [0] */
    output   [1:0] ram_size,                    /* [2:1], in the board's encoding: 0 = 512 KB, 1 = 1 MB, 2 = 2 MB, 3 = 4 MB */
    output reg [1:0] hard_disk_geometry,        /* [8:7], disk code numbering: 0 = 16 sect, 1 = 17 sect (136), 2 = 17 sect (272) */
    output         floppy_eight_sectors,        /* [9]: 8 sectors per track instead of 10 */
    output         tear_off_paper,              /* [3] */
    output         serial_loopback,             /* [21] */
    output         ethernet_present,            /* [20:19]: anything but "no card" */
    output         eject_floppy_request,        /* [38] */

    /* serial channel A (OSD "Serial port") */
    output         serial_to_hps_uart,          /* channel A goes to the MiSTer's UART */
    output         serial_to_user_port,         /* channel A goes to the user port pins */
    output   [1:0] modem_flow_control,          /* [40:39]: 0 = ready follows the receive buffer, 1 = always ready, 2 = opposite polarity */

    /* sound */
    output   [1:0] sound_volume,                /* 0 off, 1 low, 2 mid, 3 high */
    output         call_sounds_on,              /* [34] */
    output   [1:0] data_sound_select,           /* [36:35]: 0 call sounds only, 1 both directions, 2 receive only, 3 transmit only */

    /* picture */
    output         invert_video,                /* [6] */
    output   [1:0] screen_colour,               /* 0 = white, 1 = green, 2 = amber, 3 = mint */
    output         phosphor_on,
    output   [1:0] glow_mode,                   /* 0 off, 1 low, 2 mid, 3 high */
    output   [1:0] glow_noise,                  /* 0 off, 1 low, 2 mid, 3 high */
    output         edges_on,
    output         round_corners,               /* [12]: the corners of the picture cut to a quarter circle: 0 = off (default), 1 = on */
    output   [1:0] roll_band,                   /* 0 off, 1 low, 2 mid, 3 high */
    output   [1:0] line_wobble,                 /* 0 off, 1 low, 2 mid, 3 high */
    output   [1:0] noise,                       /* 0 off, 1 low, 2 mid, 3 high */
    output         curvature_on,                /* [43] the barrel distortion of the picture (its strength is fixed in video.sv) */
    output         bezel_on,                    /* [44] the monitor's frame on both sides of the picture */
    output   [7:0] glass_floor_level            /* brightness code of "black" inside the screen */
);

assign reset_request        = status[0];
assign floppy_eight_sectors = status[9];

/*
    Hard disk geometry. The OSD lists 17 sect (136) first so that it is the default (MiSTer starts every option at 0); the disk code keeps its
    own numbering, so the menu entry is translated here:

        OSD [8:7]   menu entry        disk code
           0        17 sect (136)        1
           1        17 sect (272)        2
           2, 3     16 sect              0
*/
always @* begin
    case (status[8:7])
        2'd0:    hard_disk_geometry = 2'd1;
        2'd1:    hard_disk_geometry = 2'd2;
        default: hard_disk_geometry = 2'd0;
    endcase
end
assign tear_off_paper       = status[3];
assign serial_loopback      = status[21];
assign ethernet_present     = (status[20:19] != 2'd1);
assign eject_floppy_request = status[38];
assign invert_video         = status[6];
assign phosphor_on          = ~status[11];
assign edges_on             = ~status[13];
assign round_corners        = status[12];
assign curvature_on         = ~status[43];
assign bezel_on             = ~status[44];
assign call_sounds_on       = ~status[34];
assign data_sound_select    = status[36:35];
assign modem_flow_control   = status[40:39];

/* OSD "Serial port": 0 = the MiSTer UART, 1 = user port, 2 = both */
assign serial_to_hps_uart  = (status[42:41] != 2'd1);
assign serial_to_user_port = (status[42:41] != 2'd0);

/* a menu choice becomes the value the core uses: the first line of each table is the default */
reg [1:0] ram_size_value;
reg [1:0] screen_colour_value;
reg [1:0] glow_mode_value;
reg [1:0] glow_noise_value;
reg [1:0] roll_band_value;
reg [1:0] line_wobble_value;
reg [1:0] noise_value;
reg [1:0] sound_volume_value;
reg [7:0] glass_floor_value;

always @* begin
    case (status[2:1])                      /* RAM: 2 MB, 4 MB, 512 KB, 1 MB */
        2'd0:    ram_size_value = 2'd2;
        2'd1:    ram_size_value = 2'd3;
        2'd2:    ram_size_value = 2'd0;
        default: ram_size_value = 2'd1;
    endcase

    case (status[5:4])                      /* Screen colour: Green, White, Amber, Mint */
        2'd0:    screen_colour_value = 2'd1;
        2'd1:    screen_colour_value = 2'd0;
        default: screen_colour_value = status[5:4];
    endcase

    case (status[15:14])                    /* Glow: Mid, High, Low, Off */
        2'd0:    glow_mode_value = 2'd2;
        2'd1:    glow_mode_value = 2'd3;
        2'd2:    glow_mode_value = 2'd1;
        default: glow_mode_value = 2'd0;
    endcase

    case (status[31:30])                    /* Glow noise: Mid, High, Low, Off */
        2'd0:    glow_noise_value = 2'd2;
        2'd1:    glow_noise_value = 2'd3;
        2'd2:    glow_noise_value = 2'd1;
        default: glow_noise_value = 2'd0;
    endcase

    case (status[23:22])                    /* Roll band: Low, Mid, High, Off */
        2'd0:    roll_band_value = 2'd1;
        2'd1:    roll_band_value = 2'd2;
        2'd2:    roll_band_value = 2'd3;
        default: roll_band_value = 2'd0;
    endcase

    case (status[27:26])                    /* Line wobble: Mid, High, Low, Off */
        2'd0:    line_wobble_value = 2'd2;
        2'd1:    line_wobble_value = 2'd3;
        2'd2:    line_wobble_value = 2'd1;
        default: line_wobble_value = 2'd0;
    endcase

    case (status[29:28])                    /* Noise: Mid, High, Low, Off */
        2'd0:    noise_value = 2'd2;
        2'd1:    noise_value = 2'd3;
        2'd2:    noise_value = 2'd1;
        default: noise_value = 2'd0;
    endcase

    case (status[33:32])                    /* Sound volume: Mid, High, Low, Off */
        2'd0:    sound_volume_value = 2'd2;
        2'd1:    sound_volume_value = 2'd3;
        2'd2:    sound_volume_value = 2'd1;
        default: sound_volume_value = 2'd0;
    endcase

    case (status[17:16])                    /* Glass: Low, Mid, High, Off (brightness code of the black of the tube face) */
        2'd0:    glass_floor_value = 8'd45;
        2'd1:    glass_floor_value = 8'd60;
        2'd2:    glass_floor_value = 8'd68;
        default: glass_floor_value = 8'd0;
    endcase
end

assign ram_size          = ram_size_value;
assign screen_colour     = screen_colour_value;
assign glow_mode         = glow_mode_value;
assign glow_noise        = glow_noise_value;
assign roll_band         = roll_band_value;
assign line_wobble       = line_wobble_value;
assign noise             = noise_value;
assign sound_volume      = sound_volume_value;
assign glass_floor_level = glass_floor_value;


endmodule
