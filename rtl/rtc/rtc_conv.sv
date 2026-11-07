/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Host clock -> TC8250 time load. FPGA only: the real board has a battery backed clock chip and no host.

    MiSTer's hps_io delivers the host time as a 65 bit word in the layout of the MSM6242B clock chip (BCD digits, one nibble each):
        [3:0] seconds ones   [7:4] seconds tens   [11:8] minutes ones   [15:12] minutes tens   [19:16] hours ones   [23:20] hours tens
        [27:24] day ones     [31:28] day tens     [35:32] month ones    [39:36] month tens     [43:40] year ones    [47:44] year tens
        [51:48] weekday      [64] toggles every time Main_MiSTer sends the time (at core start, then once a minute)
    The TC8250 (rtl/rtc/tc8250.sv) takes the same digits as its registers 0..12 (set_val nibble i = register i), with two differences: register 9
    holds the month tens digit in D0 and "year mod 4" (the leap year selector L1 L0) in D3:D2, where the host word has the month tens digit alone; the
    chip has a two digit year and the UNIX PC software expects 19xx. The calendar repeats every 28 years, weekday included, so the host year is folded
        into 1972..1999: that gives the right weekday and the right leap years.

    The load is done once, on the first update after the core was loaded: the chip keeps counting by itself afterwards and the OS has read the time at
    boot. `ready` tells the board it may leave reset. It goes high when the first update has been seen, or after 2^WAIT_BITS clocks (about 1.7 s at 20
    MHz) for a host that has no clock, so the OS never starts with the chip's default date unless there is no host time at all.
*/
module rtc_conv
#(
    parameter WAIT_BITS = 25                        /* length of the wait for the first host update, in clocks: 2^WAIT_BITS */
)
(
    input             clk,
    input             rst,                          /* not used (the state is only ever set once after the core is loaded) */
    input      [64:0] rtc,                          /* host time, layout above */
    output reg        set_en,                       /* one clock pulse: load set_val into the TC8250 */
    output reg [51:0] set_val,                      /* the TC8250 registers 0..12, nibble i = register i */
    output            ready                         /* the board may leave reset */
);

/* -------------------------------------------------------------------------------------------------------------------------------- year folding */
localparam CALENDAR_CYCLE_YEARS = 28;               /* the calendar (leap years and weekdays) repeats every 28 years */
localparam FOLDED_YEAR_FIRST    = 72;               /* the folded year runs from 1972 ... */

wire [3:0] host_year_ones = rtc[43:40];
wire [3:0] host_year_tens = rtc[47:44];
wire [6:0] host_year      = host_year_tens * 7'd10 + host_year_ones;            /* 0..99 */

/* year mod 28: host_year is below 100, so at most three subtractions of 28 are needed */
reg [6:0] year_in_cycle;
always @* begin
    if      (host_year >= 3 * CALENDAR_CYCLE_YEARS) year_in_cycle = host_year - 3 * CALENDAR_CYCLE_YEARS;
    else if (host_year >= 2 * CALENDAR_CYCLE_YEARS) year_in_cycle = host_year - 2 * CALENDAR_CYCLE_YEARS;
    else if (host_year >= 1 * CALENDAR_CYCLE_YEARS) year_in_cycle = host_year - 1 * CALENDAR_CYCLE_YEARS;
    else                                            year_in_cycle = host_year;
end

wire [6:0] folded_year      = FOLDED_YEAR_FIRST + year_in_cycle;                /* 72 .. 99 */
wire [3:0] folded_year_tens = folded_year / 7'd10;
wire [3:0] folded_year_ones = folded_year % 7'd10;
wire [1:0] folded_year_mod_4 = folded_year[1:0];                                 /* the leap year selector L1 L0 of register 9 */

/* -------------------------------------------------------------------------------------------------------------------------------- the load */
wire [35:0] host_seconds_to_month_ones = rtc[35:0];                               /* registers 0..8 are the host digits unchanged */
wire        host_month_tens            = rtc[36];
wire [3:0]  host_weekday               = rtc[51:48];

wire [3:0]  register_9 = {folded_year_mod_4, 1'b0, host_month_tens};             /* L1 L0, the stored bit (0), month tens */

reg        previous_update_toggle = 1'b0;
reg        loaded = 1'b0;                                                         /* the first host time has been loaded */
reg [WAIT_BITS:0] wait_clocks = 0;

wire wait_over = wait_clocks[WAIT_BITS];
wire host_sent_time = (rtc[64] != previous_update_toggle);

assign ready = loaded | wait_over;

always @(posedge clk) begin
    previous_update_toggle <= rtc[64];
    set_en                 <= 1'b0;

    if (!wait_over)
        wait_clocks <= wait_clocks + 1'b1;

    if (!loaded && host_sent_time) begin
        loaded  <= 1'b1;
        set_en  <= 1'b1;
        set_val <= {host_weekday, folded_year_tens, folded_year_ones, register_9, host_seconds_to_month_ones};
    end
end

endmodule
