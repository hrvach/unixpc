/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Definitions shared by the Ethernet parts of the core.
*/
package ethernet_pkg;

    /*
        The Ethernet CRC-32 after one more byte of the frame: reflected, polynomial 0x04C11DB7 (0xEDB88320 as shifted right), no final inversion.
        Start with all ones; the receiver appends the result as the frame check sequence, least significant byte first.
    */
    function automatic [31:0] crc_after_byte(input [31:0] crc, input [7:0] data_byte);
        reg [31:0] shifted;
        integer    bit_number;

        begin
            shifted = crc ^ {24'd0, data_byte};

            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                if (shifted[0])
                    shifted = (shifted >> 1) ^ 32'hEDB88320;
                else
                    shifted = shifted >> 1;
            end

            crc_after_byte = shifted;
        end
    endfunction

endpackage
