/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Scancode table of the keyboard converter (FPGA only, there is no such chip on the real machine, where the keyboard has its own microcontroller).

    The PS/2 key events that hps_io delivers carry a set 2 scancode with an "extended" flag (the 0xE0 prefix). The 3B1 keyboard reports a key by one
    byte, and these two tables give that byte: one for the key on its own, one for the key pressed while Alt is held down (the 3B1 has keys the PC
    keyboard lacks, they sit on Alt + another key). A table entry of 0 means "no such key"; in the Alt table FF means "swallowed" (the combination
    belongs to the core, not to the guest).

    The tables are written as functions and copied into a block RAM at start-up: 512 entries of 16 bits, {Alt key, normal key}, addressed by
    {extended, scancode}. Reading takes one clock: the entry for the address of clock n is on the outputs in clock n+1.
*/
module kbd_scancode_rom
(
    input             clk,
    input      [8:0]  ps2_scancode,       /* {extended flag, set 2 scancode} */

    output     [7:0]  normal_key,         /* 3B1 key byte, 0 = unmapped */
    output     [7:0]  alt_key             /* 3B1 key byte with Alt held, 0 = same as normal, FF = swallowed */
);

/* The 3B1 key for a PC key on its own: printable keys give their lower case ASCII code, the others the codes of the kernel's key table (kbd.h). */
function [7:0] scancode_to_key(input extended, input [7:0] scancode);
    case ({extended, scancode})
        9'h045: scancode_to_key = "0";
        9'h016: scancode_to_key = "1";
        9'h01e: scancode_to_key = "2";
        9'h026: scancode_to_key = "3";
        9'h025: scancode_to_key = "4";
        9'h02e: scancode_to_key = "5";
        9'h036: scancode_to_key = "6";
        9'h03d: scancode_to_key = "7";
        9'h03e: scancode_to_key = "8";
        9'h046: scancode_to_key = "9";
        9'h01c: scancode_to_key = "a";
        9'h032: scancode_to_key = "b";
        9'h021: scancode_to_key = "c";
        9'h023: scancode_to_key = "d";
        9'h024: scancode_to_key = "e";
        9'h02b: scancode_to_key = "f";
        9'h034: scancode_to_key = "g";
        9'h033: scancode_to_key = "h";
        9'h043: scancode_to_key = "i";
        9'h03b: scancode_to_key = "j";
        9'h042: scancode_to_key = "k";
        9'h04b: scancode_to_key = "l";
        9'h03a: scancode_to_key = "m";
        9'h031: scancode_to_key = "n";
        9'h044: scancode_to_key = "o";
        9'h04d: scancode_to_key = "p";
        9'h015: scancode_to_key = "q";
        9'h02d: scancode_to_key = "r";
        9'h01b: scancode_to_key = "s";
        9'h02c: scancode_to_key = "t";
        9'h03c: scancode_to_key = "u";
        9'h02a: scancode_to_key = "v";
        9'h01d: scancode_to_key = "w";
        9'h022: scancode_to_key = "x";
        9'h035: scancode_to_key = "y";
        9'h01a: scancode_to_key = "z";
        9'h05a: scancode_to_key = 8'h0d; /* Return -> RETURN */
        9'h15a: scancode_to_key = 8'h0a; /* keypad Enter -> ENTER */
        9'h076: scancode_to_key = 8'h1b; /* Esc -> ESC */
        9'h066: scancode_to_key = 8'h08; /* Backspace -> BACKSPACE */
        9'h00d: scancode_to_key = 8'h09; /* Tab -> TAB */
        9'h171: scancode_to_key = 8'h7f; /* Delete -> DLETE */
        9'h029: scancode_to_key = " "; /* Space */
        9'h052: scancode_to_key = "'"; /* ' */
        9'h00e: scancode_to_key = "`"; /* ` */
        9'h041: scancode_to_key = ","; /* , */
        9'h04e: scancode_to_key = "-"; /* - */
        9'h049: scancode_to_key = "."; /* . */
        9'h04a: scancode_to_key = "/"; /* / */
        9'h04c: scancode_to_key = ";"; /* ; */
        9'h055: scancode_to_key = "="; /* = */
        9'h05d: scancode_to_key = 8'h5c; /* backslash */
        9'h054: scancode_to_key = "["; /* [ */
        9'h05b: scancode_to_key = "]"; /* ] */
        9'h005: scancode_to_key = 8'h15; /* F1 -> F1 */
        9'h006: scancode_to_key = 8'h16; /* F2 -> F2 */
        9'h004: scancode_to_key = 8'h17; /* F3 -> F3 */
        9'h00c: scancode_to_key = 8'h18; /* F4 -> F4 */
        9'h003: scancode_to_key = 8'h19; /* F5 -> F5 */
        9'h00b: scancode_to_key = 8'h1a; /* F6 -> F6 */
        9'h083: scancode_to_key = 8'h1c; /* F7 -> F7 */
        9'h00a: scancode_to_key = 8'h1d; /* F8 -> F8 */
        9'h012: scancode_to_key = 8'h48; /* left Shift -> left SHIFT */
        9'h059: scancode_to_key = 8'h49; /* right Shift -> right SHIFT */
        9'h014: scancode_to_key = 8'h4c; /* left Ctrl -> left CTRL */
        9'h114: scancode_to_key = 8'h4d; /* right Ctrl -> right CTRL */
        9'h058: scancode_to_key = 8'h45; /* Caps Lock -> CAPS LOCK */
        9'h061: scancode_to_key = 8'h5c; /* key left of Z (102 key keyboards) -> backslash */
        9'h001: scancode_to_key = 8'h1e; /* F9 -> Suspd */
        9'h009: scancode_to_key = 8'h1f; /* F10 -> Rsume */
        9'h078: scancode_to_key = 8'h42; /* F11 -> CLOSE/OPEN */
        9'h070: scancode_to_key = 8'h0b; /* keypad 0 -> ROLL/DN (Down, keypad 0) */
        9'h069: scancode_to_key = 8'h05; /* keypad 1 -> PREV (keypad 1) */
        9'h072: scancode_to_key = 8'h01; /* keypad 2 -> ROLL/UP (Up, keypad 2) */
        9'h07a: scancode_to_key = 8'h0c; /* keypad 3 -> NEXT (keypad 3) */
        9'h06b: scancode_to_key = 8'h47; /* keypad 4 -> BEG (keypad 4) */
        9'h073: scancode_to_key = 8'h4a; /* keypad 5 -> HOME (keypad 5) */
        9'h074: scancode_to_key = 8'h4b; /* keypad 6 -> END (keypad 6) */
        9'h06c: scancode_to_key = 8'h43; /* keypad 7 -> PRINT (keypad 7) */
        9'h075: scancode_to_key = 8'h44; /* keypad 8 -> CLEAR/RFRSH (keypad 8) */
        9'h07d: scancode_to_key = 8'h46; /* keypad 9 -> PAGE (keypad 9, Page Down) */
        9'h071: scancode_to_key = 8'h0f; /* keypad . -> right arrow (keypad .) */
        9'h07b: scancode_to_key = 8'h0e; /* keypad - -> left arrow (keypad -) */
        9'h175: scancode_to_key = 8'h01; /* Up -> ROLL/UP (Up, keypad 2) */
        9'h172: scancode_to_key = 8'h0b; /* Down -> ROLL/DN (Down, keypad 0) */
        9'h16b: scancode_to_key = 8'h0e; /* Left -> left arrow (keypad -) */
        9'h174: scancode_to_key = 8'h0f; /* Right -> right arrow (keypad .) */
        9'h16c: scancode_to_key = 8'h4a; /* Home -> HOME (keypad 5) */
        9'h169: scancode_to_key = 8'h4b; /* End -> END (keypad 6) */
        9'h17d: scancode_to_key = 8'h2a; /* Page Up -> Help */
        9'h17a: scancode_to_key = 8'h46; /* Page Down -> PAGE (keypad 9, Page Down) */
        9'h170: scancode_to_key = 8'h41; /* Insert -> CMD */
        9'h14a: scancode_to_key = 8'h7e; /* keypad / -> NUMLOCK */
        9'h177: scancode_to_key = 8'h25; /* Pause -> RESET/BREAK */
        default: scancode_to_key = 8'h00;
    endcase
endfunction

/* The same keys with Alt held: the 3B1 keys that have no PC counterpart. Alt+F1 / Alt+F2 belong to the core (video: screen / printer paper). */
function [7:0] scancode_to_alt_key(input extended, input [7:0] scancode);
    case ({extended, scancode})
        9'h076: scancode_to_alt_key = 8'h04; /* Alt+Esc       Exit */
        9'h066: scancode_to_alt_key = 8'h07; /* Alt+Backspace CANCL */
        9'h05a: scancode_to_alt_key = 8'h0a; /* Alt+Return    ENTER */
        9'h005: scancode_to_alt_key = 8'hff; /* Alt+F1   core: screen */
        9'h006: scancode_to_alt_key = 8'hff; /* Alt+F2   core: printer paper */
        9'h004: scancode_to_alt_key = 8'h02; /* Alt+F3   ClearLine */
        9'h00c: scancode_to_alt_key = 8'h03; /* Alt+F4   Rstrt/Ref */
        9'h003: scancode_to_alt_key = 8'h06; /* Alt+F5   Msg */
        9'h00b: scancode_to_alt_key = 8'h10; /* Alt+F6   Creat */
        9'h083: scancode_to_alt_key = 8'h11; /* Alt+F7   Save */
        9'h00a: scancode_to_alt_key = 8'h12; /* Alt+F8   MOVE */
        9'h001: scancode_to_alt_key = 8'h13; /* Alt+F9   Ops */
        9'h009: scancode_to_alt_key = 8'h14; /* Alt+F10  COPY */
        9'h078: scancode_to_alt_key = 8'h21; /* Alt+F11  Undo      (Alt+F12 is Main_MiSTer's OSD key) */
        9'h016: scancode_to_alt_key = 8'h24; /* Alt+1    RPLAC */
        9'h01e: scancode_to_alt_key = 8'h26; /* Alt+2    DLETECHAR */
        9'h026: scancode_to_alt_key = 8'h28; /* Alt+3    SLECT/MARK */
        9'h025: scancode_to_alt_key = 8'h29; /* Alt+4    INPUT/MODE */
        9'h02e: scancode_to_alt_key = 8'h22; /* Alt+5    Redo */
        9'h036: scancode_to_alt_key = 8'h23; /* Alt+6    FIND */
        default: scancode_to_alt_key = 8'h00;
    endcase
endfunction

reg [15:0] key_table [0:511];
integer    table_index;

initial begin
    for (table_index = 0; table_index < 512; table_index = table_index + 1)
        key_table[table_index] = {scancode_to_alt_key(table_index[8], table_index[7:0]),
                                  scancode_to_key(table_index[8], table_index[7:0])};
end

reg [15:0] key_table_output = 16'd0;

always @(posedge clk)
    key_table_output <= key_table[ps2_scancode];

assign normal_key = key_table_output[7:0];
assign alt_key    = key_table_output[15:8];

endmodule
