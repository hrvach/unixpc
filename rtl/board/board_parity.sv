/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Bad parity flags: one flag per RAM byte lane (the real RAM has a ninth, parity bit per byte).

    The real board checks parity on every RAM read and raises PERR* on a mismatch. Here no parity is stored; instead a flag per byte remembers that
    the byte was written while the diagnostic "force bad parity" bit (GCR bit 2, BP+) was set. A CPU read of RAM that hits a flagged byte makes
    bad_parity_hit true; board_status turns that into PERR*. (The caller decides when a hit counts: not during DMA and not during a page fault.)

      EXT_RAM = 0 : two 1-bit block RAMs (one per byte lane) for the lower 2 MB; RAM above that never reports parity EXT_RAM = 1 : the flags live next
      to the data in the SDRAM (rtl/ram/sdram_bridge.sv); this module only tells the bridge when they are in use
      PARITY  = 0 : no flags at all, bad_parity_hit is constant 0 (saves the block RAM; the ROM diagnostic's parity test then fails)
*/
module board_parity
#(
    parameter PARITY  = 1,
    parameter EXT_RAM = 0
)
(
    input         clk,
    input         rst,
    input         force_bad_parity,                 /* GCR bit 2 (BP+): bytes written now get a bad parity flag */
    input  [21:1] ram_word_address,                 /* mapped RAM word address */

    /* EXT_RAM = 0: the write strobe of the internal RAM and its byte enables */
    input         ram_write,
    input  [1:0]  ram_write_byte_enables,

    /* EXT_RAM = 1: the transaction toggle and write flag of the SDRAM bridge, and the flags of the bytes it just read */
    input         sdram_request_toggle,
    input         sdram_write,
    input  [1:0]  sdram_flags_read,
    output        sdram_flags_in_use,               /* the SDRAM keeps flags (from the first write with bad parity forced) */

    output        bad_parity_hit                    /* the addressed word has a flagged byte */
);

generate
    if (PARITY && EXT_RAM == 0) begin : internal_flags
        reg         flag_upper_byte [0:(1<<20)-1];
        reg         flag_lower_byte [0:(1<<20)-1];
        reg         flag_upper_byte_read = 1'b0;
        reg         flag_lower_byte_read = 1'b0;
        wire [19:0] flag_address = ram_word_address[20:1];

        always @(posedge clk) begin
            if (ram_write & ram_write_byte_enables[1] & ~ram_word_address[21])
                flag_upper_byte[flag_address] <= force_bad_parity;

            flag_upper_byte_read <= flag_upper_byte[flag_address];
        end

        always @(posedge clk) begin
            if (ram_write & ram_write_byte_enables[0] & ~ram_word_address[21])
                flag_lower_byte[flag_address] <= force_bad_parity;

            flag_lower_byte_read <= flag_lower_byte[flag_address];
        end

        assign bad_parity_hit     = (flag_upper_byte_read | flag_lower_byte_read) & ~ram_word_address[21];
        assign sdram_flags_in_use = 1'b0;
    end else if (PARITY) begin : sdram_flags
        /* The SDRAM flags stay untouched until the first write with bad parity forced, so only the diagnostic pays for them. */
        reg flags_dirty                 = 1'b0;
        reg sdram_request_toggle_before = 1'b0;

        always @(posedge clk) begin
            sdram_request_toggle_before <= sdram_request_toggle;

            if (rst)
                flags_dirty <= 1'b0;
            else if ((sdram_request_toggle ^ sdram_request_toggle_before) & sdram_write & force_bad_parity)
                flags_dirty <= 1'b1;
        end

        assign bad_parity_hit     = |sdram_flags_read & ~ram_word_address[21];
        assign sdram_flags_in_use = flags_dirty | (sdram_write & force_bad_parity);
    end else begin : no_flags
        assign bad_parity_hit     = 1'b0;
        assign sdram_flags_in_use = 1'b0;
    end
endgenerate

endmodule
