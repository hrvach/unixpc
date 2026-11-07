/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Shared constant of the audio synthesiser (FPGA only): how a frequency in Hz turns into a phase increment.

    The tone generator (tone_gen.sv) keeps one 32 bit phase per voice and adds an increment to it once per audio sample. A full turn of the phase
    is 2^32, so a tone of f Hz needs

        increment = f * SAMPLE_DIVIDER / CLOCK_HZ * 2^32

    because one audio sample is SAMPLE_DIVIDER clocks long (417 clocks of 20 MHz; every third sample is a clock shorter, so the true rate is 48 kHz and
    every tone is 0.08 % sharp, 1.4 cents, which nobody hears). All of the sound modules call this function with constant
    frequencies, so Quartus folds every call into a constant and the design has no divider.
*/
package tone_pkg;

    function automatic [31:0] phase_increment
    (
        input [15:0] frequency_hz,
        input integer clock_hz,
        input integer sample_divider
    );
        reg [63:0] scaled;
        begin
            scaled          = {48'd0, frequency_hz} * sample_divider;
            scaled          = (scaled << 32) / clock_hz;
            phase_increment = scaled[31:0];
        end
    endfunction

endpackage
