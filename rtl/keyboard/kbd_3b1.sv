/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Keyboard and mouse converter for the keyboard port (FPGA only: the real keyboard has a microcontroller that scans the keys and sends bytes to the
    ACIA at 9600 baud).

    Input is what MiSTer's hps_io gives: PS/2 set 2 key events (ps2_key) and mouse packets (ps2_mouse). Output is the byte stream the 3B1 kernel's
    keyboard driver expects, presented on kbd_data / kbd_strobe (the ACIA model takes them as received characters). The protocol, verified against the
    kernel (see tb/hps):

     - Every key report lists ALL keys that are down, by 3B1 key code in ascending order; all bytes but the last have bit 7 clear, the last has bit 7
       set and makes the kernel process the list. The byte 0x40 means "all keys up".
     - Printable keys: the key code is the lower case ASCII code (digits and punctuation unshifted); Esc 1B, BS 08, TAB 09, CR 0D, DLETE 7F; shift 48
       / 49 (left / right), ctrl 4C / 4D; F1..F8 15 16 17 18 19 1A 1C 1D. The other keys of the UNIX PC keyboard (arrows and keypad share keys, Help,
       CMD, ...) are in kbd_scancode_rom.sv. Alt selects the keys the PC keyboard lacks (Alt+Esc Exit, Alt+Backspace CANCL, Alt+Return ENTER,
       Alt+F3..F11 and Alt+1..6), like the freebee emulator.
     - Mouse (after the kernel sent 0xD0 to enable it): CE, flags, |dx|, 80 | |dy|. Flags: 10 dx > 0, 08 dy < 0 (down is positive on screen), buttons
       R=1 M=2 L=4. A "80 | |dy|" byte that equals DF is sent twice. Keyboard bytes that follow mouse data are prefixed with DF (BEGKBD).
     - Bytes leave 2 ms apart (kbd_byte_fifo).

    How it works: a key event is stored in a one slot register and looked up in kbd_scancode_rom. A RAM with one bit per key code remembers which keys
    are down. A report scans that RAM in ascending order and writes the codes of the keys found into the FIFO. A mouse packet is built from the live
    ps2_mouse inputs, which hold still for several clocks. Pause has no release event, so it is released by itself right after its report.
*/
module kbd_3b1
(
    input             clk,                /* 20 MHz */
    input             rst,
    input      [10:0] ps2_key,            /* [10] toggles per event, [9] pressed, [8] extended, [7:0] set 2 scancode */
    input      [24:0] ps2_mouse,          /* [24] toggles per packet, [7:0] status, [15:8] dx, [23:16] dy */
    input             tx_valid,           /* the CPU wrote a byte to the keyboard port (the mouse enable and disable commands) */
    input      [7:0]  tx_data,

    output     [7:0]  kbd_data,           /* byte for the keyboard port */
    output            kbd_strobe          /* high while kbd_data is valid */
);

localparam [7:0] MOUSE_ENABLE_COMMAND     = 8'hd0;
localparam [7:0] MOUSE_DISABLE_COMMAND_1  = 8'hd1;
localparam [7:0] MOUSE_DISABLE_COMMAND_2  = 8'h92;
localparam [7:0] PS2_SCANCODE_ALT         = 8'h11;
localparam [8:0] PS2_KEY_PAUSE            = 9'h177;       /* hps_io delivers Pause as a press only */
localparam [6:0] KEY_CODE_RESET_BREAK     = 7'h25;        /* the 3B1 key that Pause gives */
localparam [7:0] ALT_KEY_SWALLOWED        = 8'hff;
localparam [7:0] BYTE_ALL_KEYS_UP         = 8'h40;
localparam [7:0] BYTE_KEYBOARD_BEGIN      = 8'hdf;        /* BEGKBD: keyboard data follows mouse data */
localparam [7:0] BYTE_MOUSE_BEGIN         = 8'hce;
localparam [6:0] MOUSE_DY_ESCAPED_VALUE   = 7'h5f;        /* 0x80 | 0x5F = 0xDF, would read as BEGKBD, so it is sent twice */

/* What the report logic is doing. */
typedef enum logic [1:0] {
    STATE_IDLE              = 2'd0,
    STATE_SCAN_KEYS_DOWN    = 2'd1,   /* walk through the keys down RAM and write the codes found into the FIFO */
    STATE_FINISH_KEY_REPORT = 2'd2,   /* mark the last byte (bit 7), or write "all keys up" */
    STATE_MOUSE_PACKET      = 2'd3
} report_state_t;

report_state_t report_state                  = STATE_IDLE;
/* table position; also the sweep counter that clears the RAM. The RAM answers one clock late: position n handles key n - 1 */
reg [7:0] scan_position                 = 8'd0;
reg [3:0] report_byte_count             = 4'd0;           /* key report: bytes written so far; mouse packet: number of the byte */
reg [7:0] last_key_found                = 8'd0;           /* [7] = some key is down, [6:0] = the highest key code found by the scan */
reg       ps2_key_toggle_previous       = 1'b0;
reg       ps2_mouse_toggle_previous     = 1'b0;
reg       mouse_enabled                 = 1'b0;
reg       report_due                    = 1'b0;           /* a key changed: a report is to be sent */
reg       left_alt_down                 = 1'b0;
reg       right_alt_down                = 1'b0;
reg       pause_release_pending         = 1'b0;           /* Pause was pressed: release it after its report */
reg       pause_release_now             = 1'b0;
reg       clearing_keys_down_ram        = 1'b1;           /* sweep after reset */
reg       second_clear_valid            = 1'b0;           /* second clear of a key release (the key was down with and without Alt) */
reg [6:0] second_clear_key_code         = 7'd0;
reg       last_report_was_mouse         = 1'b0;

/* ---- one slot register for the newest key event: it waits until the report logic is idle. The ROM output is valid one clock after the load. */
reg [8:0] event_scancode                = 9'd0;           /* {extended, scancode} */
reg       event_pressed                 = 1'b0;
reg       event_waiting                 = 1'b0;
reg       event_just_loaded             = 1'b0;

wire [7:0] normal_key;
wire [7:0] alt_key;

kbd_scancode_rom scancode_rom
(
    .clk(clk),
    .ps2_scancode(event_scancode),
    .normal_key(normal_key),
    .alt_key(alt_key)
);

/* ---- the keys down RAM: one bit per key code, written by key events, read by the scan, cleared by a sweep after reset */
reg       keys_down [0:127];
reg       scan_key_is_down              = 1'b0;
reg       keys_down_write;
reg [6:0] keys_down_write_address;
reg       keys_down_write_value;

always @(posedge clk) begin
    if (keys_down_write)
        keys_down[keys_down_write_address] <= keys_down_write_value;

    scan_key_is_down <= keys_down[scan_position[6:0]];
end

/* ---- decoding of the stored event */
wire       report_logic_busy     = (report_state != STATE_IDLE) | second_clear_valid | clearing_keys_down_ram;
wire       event_processed_now   = event_waiting && !event_just_loaded && !report_logic_busy;
wire       event_is_alt_key      = (event_scancode[7:0] == PS2_SCANCODE_ALT);
wire       alt_held              = left_alt_down | right_alt_down;
wire       alt_key_defined       = (alt_key != 8'd0) && (alt_key != ALT_KEY_SWALLOWED);
wire [7:0] key_to_press          = (alt_held && alt_key != 8'd0) ? alt_key : normal_key;

wire       key_press_event       = event_processed_now && !event_is_alt_key &&  event_pressed && key_to_press != 8'd0 && key_to_press < 8'd128;
wire       key_release_event     = event_processed_now && !event_is_alt_key && !event_pressed && (normal_key != 8'd0 || alt_key_defined);
wire       pause_release_write   = (report_state == STATE_FINISH_KEY_REPORT) && pause_release_now;

/* ---- mouse packet. One shared unit makes sign / magnitude / clamp for dx (byte 2) and dy (byte 3). */
wire [8:0] mouse_motion          = (report_byte_count == 4'd2) ? {ps2_mouse[4], ps2_mouse[15:8]}
                                                               : {ps2_mouse[5], ps2_mouse[23:16]};
wire [8:0] mouse_magnitude       = mouse_motion[8] ? (9'd0 - mouse_motion) : mouse_motion;
wire [6:0] mouse_clamped         = (mouse_magnitude[8] | mouse_magnitude[7]) ? 7'h7f : mouse_magnitude[6:0];
wire       mouse_packet_arrived  = (ps2_mouse[24] != ps2_mouse_toggle_previous);
wire       mouse_moves_right     = ~ps2_mouse[4] && (ps2_mouse[15:8]  != 8'd0);       /* flag 0x10: dx > 0 */
wire       mouse_moves_down      = ~ps2_mouse[5] && (ps2_mouse[23:16] != 8'd0);       /* flag 0x08: PS/2 dy > 0 is up, so dy < 0 on screen */
wire       mouse_dy_needs_repeat = (report_byte_count == 4'd3) && (mouse_clamped == MOUSE_DY_ESCAPED_VALUE);

/* the last byte of the report is written in this clock */
wire       report_ends = (report_state == STATE_FINISH_KEY_REPORT)
                      || (report_state == STATE_MOUSE_PACKET && ((report_byte_count == 4'd3 && !mouse_dy_needs_repeat) || report_byte_count == 4'd4));

/* index of the byte written now (the finishing step rewrites the last key byte, with bit 7 set) */
wire [3:0] write_index           = (report_state == STATE_FINISH_KEY_REPORT && last_key_found[7]) ? report_byte_count - 4'd1 : report_byte_count;
wire [6:0] scanned_key_code      = scan_position[6:0] - 7'd1;

wire       fifo_room;

/* Keys down RAM write port: one write per clock; the first condition that holds decides. */
always @* begin
    keys_down_write         = 1'b1;
    keys_down_write_address = 7'd0;
    keys_down_write_value   = 1'b0;

    if (clearing_keys_down_ram) begin
        keys_down_write_address = scan_position[6:0];
    end else if (key_press_event) begin
        keys_down_write_address = key_to_press[6:0];
        keys_down_write_value   = 1'b1;
    end else if (key_release_event && normal_key != 8'd0) begin
        keys_down_write_address = normal_key[6:0];
    end else if (key_release_event) begin
        keys_down_write_address = alt_key[6:0];                  /* (normal_key is 0 here) */
    end else if (second_clear_valid) begin
        keys_down_write_address = second_clear_key_code;
    end else if (pause_release_write) begin
        keys_down_write_address = KEY_CODE_RESET_BREAK;
    end else begin
        keys_down_write = 1'b0;
    end
end

/* FIFO write port: what is written at position write_index of the report being built. */
reg       fifo_write;
reg [7:0] fifo_byte;

always @* begin
    fifo_write = 1'b0;
    fifo_byte  = 8'd0;

    case (report_state)
        STATE_IDLE: begin
            /* keyboard data after mouse data starts with BEGKBD (report_byte_count is 0 here) */
            if (report_due && !event_waiting && fifo_room && last_report_was_mouse) begin
                fifo_write = 1'b1;
                fifo_byte  = BYTE_KEYBOARD_BEGIN;
            end
        end

        STATE_SCAN_KEYS_DOWN: begin
            /* a key found by the scan, at most 9 bytes */
            if (scan_position != 8'd0 && scan_key_is_down && !(report_byte_count[3] & report_byte_count[0])) begin
                fifo_write = 1'b1;
                fifo_byte  = {1'b0, scanned_key_code};
            end
        end

        STATE_FINISH_KEY_REPORT: begin
            /* the last byte gets bit 7; with no key down the report is the single byte "all keys up" */
            fifo_write = 1'b1;
            fifo_byte  = last_key_found[7] ? {1'b1, last_key_found[6:0]} : BYTE_ALL_KEYS_UP;
        end

        STATE_MOUSE_PACKET: begin
            fifo_write = 1'b1;

            case (report_byte_count[2:0])
                3'd0:    fifo_byte = BYTE_MOUSE_BEGIN;
                /* buttons R=1 M=2 L=4 */
                3'd1:    fifo_byte = {3'b000, mouse_moves_right, mouse_moves_down, ps2_mouse[0], ps2_mouse[2], ps2_mouse[1]};
                3'd2:    fifo_byte = {1'b0, mouse_clamped};
                3'd3:    fifo_byte = {1'b1, mouse_clamped};
                /* repeat of an escaped dy */
                default: fifo_byte = BYTE_KEYBOARD_BEGIN;
            endcase
        end
    endcase
end

kbd_byte_fifo output_fifo
(
    .clk(clk),
    .rst(rst),
    .report_write(fifo_write),
    .report_index(write_index),
    .report_byte(fifo_byte),
    .report_commit(report_ends),
    .room_for_report(fifo_room),
    .kbd_data(kbd_data),
    .kbd_strobe(kbd_strobe)
);

always @(posedge clk) begin
    ps2_key_toggle_previous   <= ps2_key[10];
    ps2_mouse_toggle_previous <= ps2_mouse[24];

    if (rst) begin
        report_state          <= STATE_IDLE;
        report_byte_count     <= 4'd0;
        mouse_enabled         <= 1'b0;
        report_due            <= 1'b0;
        left_alt_down         <= 1'b0;
        right_alt_down        <= 1'b0;
        pause_release_pending <= 1'b0;
        pause_release_now     <= 1'b0;
        event_waiting         <= 1'b0;
        event_just_loaded     <= 1'b0;
        last_report_was_mouse <= 1'b0;
        second_clear_valid    <= 1'b0;
        clearing_keys_down_ram <= 1'b1;
        scan_position         <= 8'd0;
    end else begin
        if (clearing_keys_down_ram) begin
            scan_position <= scan_position + 8'd1;

            if (scan_position[6:0] == 7'd127)
                clearing_keys_down_ram <= 1'b0;
        end

        second_clear_valid <= 1'b0;

        /* mouse enable / disable commands from the CPU */
        if (tx_valid) begin
            if (tx_data == MOUSE_ENABLE_COMMAND)
                mouse_enabled <= 1'b1;
            else if (tx_data == MOUSE_DISABLE_COMMAND_1 || tx_data == MOUSE_DISABLE_COMMAND_2)
                mouse_enabled <= 1'b0;
        end

        /* the newest key event goes into the slot */
        event_just_loaded <= 1'b0;

        if (event_processed_now)
            event_waiting <= 1'b0;

        if (ps2_key[10] != ps2_key_toggle_previous) begin
            event_scancode    <= ps2_key[8:0];
            event_pressed     <= ps2_key[9];
            event_waiting     <= 1'b1;
            event_just_loaded <= 1'b1;
        end

        /* A processed event: Alt only changes the Alt state; a key changes the keys down RAM (see the port logic above) and calls for a report. */
        if (event_processed_now && event_is_alt_key) begin
            if (event_scancode[8])
                right_alt_down <= event_pressed;
            else
                left_alt_down  <= event_pressed;
        end else if (key_press_event) begin
            report_due <= 1'b1;

            if (event_scancode == PS2_KEY_PAUSE)
                pause_release_pending <= 1'b1;
        end else if (key_release_event) begin
            /* a release clears the key whether or not Alt was held when it went down */
            if (normal_key != 8'd0 && alt_key_defined) begin
                second_clear_valid    <= 1'b1;
                second_clear_key_code <= alt_key[6:0];
            end

            report_due <= 1'b1;
        end

        /* the report logic */
        case (report_state)
            STATE_IDLE: begin
                if (report_due && !event_waiting && fifo_room) begin
                    report_due        <= 1'b0;
                    report_state      <= STATE_SCAN_KEYS_DOWN;
                    scan_position     <= 8'd0;
                    last_key_found    <= 8'd0;
                    report_byte_count <= last_report_was_mouse ? 4'd1 : 4'd0;      /* index 0 holds BEGKBD */
                    last_report_was_mouse <= 1'b0;

                    if (pause_release_pending) begin
                        pause_release_now     <= 1'b1;
                        pause_release_pending <= 1'b0;
                    end
                end else if (!report_due && mouse_packet_arrived && mouse_enabled && fifo_room) begin
                    report_state <= STATE_MOUSE_PACKET;
                end
            end

            STATE_SCAN_KEYS_DOWN: begin
                /* ascending key codes into the FIFO, remembering the last one */
                if (scan_position != 8'd0 && scan_key_is_down && !(report_byte_count[3] & report_byte_count[0])) begin
                    report_byte_count <= report_byte_count + 4'd1;
                    last_key_found    <= {1'b1, scanned_key_code};
                end

                scan_position <= scan_position + 8'd1;

                if (scan_position[7])
                    report_state <= STATE_FINISH_KEY_REPORT;
            end

            STATE_FINISH_KEY_REPORT: begin
                if (pause_release_now) begin
                    report_due        <= 1'b1;
                    pause_release_now <= 1'b0;
                end
            end

            STATE_MOUSE_PACKET: begin
                report_byte_count <= report_byte_count + 4'd1;
            end
        endcase

        if (report_ends) begin
            report_byte_count     <= 4'd0;
            report_state          <= STATE_IDLE;
            last_report_was_mouse <= (report_state == STATE_MOUSE_PACKET);
        end
    end
end

endmodule
