/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Toshiba TC8250 real time clock (battery backed). The CPU talks to it through two ports and a few bits, one nibble at a time.

      Write port 480000   D15 RTCCE   chip enable
                          D14 RTCALE  address latch enable
                          D13 RTCR/W  1 = write the addressed register
                          D11..D8     the 4 bit register address, or the 4 bit data to write
      Read port  E30000   D3..D0 and D11..D8 = the addressed register

    How the diagnostics and the kernel drive it:
      select a register : put the address on D11..D8, raise ALE (CE is low), drop ALE
      write it          : put the data on D11..D8, raise CE with R/W = 1
      read it           : raise CE with R/W = 0, then read the read port

    The sixteen 4 bit registers:
      0 seconds ones   1 seconds tens   2 minutes ones   3 minutes tens   4 hours ones   5 hours tens
      6 day ones       7 day tens        8 month ones     9 see below      A year ones    B year tens
      C weekday (0..6) D control (stored, not used)   E protection key   F status / seconds reset

      9  month tens in D0, L1 L0 in D3:D2 (the value of "year mod 4" that counts as a leap year, as written), D1 stored but cleared by the next count.
         Reading it gives D1 = LY, which is 1 while the current year is a leap year.
      E  the protection key. Every other register can only be written while E holds 5. Reset locks it again.
      F  read:  D1 = BUSY (around each count of the seconds), D0 = XBUSY (a count happened since F was last read)
         write: D1 = 1 resets the seconds to 00; if the seconds were 30 or more, the minutes count once (a rounding to the next minute)

    24 hour mode only. The chip counts from its 1 second clock; here that is the 20 MHz board clock divided by CLK_HZ.

    Not modelled: the seconds reset does not restart the 1 second divider, the real chip does (so a seconds reset is up to one second early or late
    here).
*/
module tc8250
#(
    parameter CLK_HZ = 20000000                 /* board clock; the divider produces one count per CLK_HZ clocks */
)
(
    input         clk,
    input         rst,

    input         wr_go,                        /* CPU writes the write port (one clock) */
    input         rd_go,                        /* CPU reads the read port (one clock) */
    input  [15:0] wdata,                        /* data of the CPU write */
    output [15:0] rdata,                        /* data of the read port */
    input         set_en,                       /* load the time from the host clock (simulation / MiSTer): set_val nibble i = register i */
    input  [51:0] set_val
);

/*
    ---- register numbers
*/
typedef enum logic [3:0] {
    REG_SECONDS_ONES = 4'd0,
    REG_SECONDS_TENS = 4'd1,
    REG_MINUTES_ONES = 4'd2,
    REG_MINUTES_TENS = 4'd3,
    REG_HOURS_ONES   = 4'd4,
    REG_HOURS_TENS   = 4'd5,
    REG_DAY_ONES     = 4'd6,
    REG_DAY_TENS     = 4'd7,
    REG_MONTH_ONES   = 4'd8,
    REG_MONTH_TENS   = 4'd9,
    REG_YEAR_ONES    = 4'd10,
    REG_YEAR_TENS    = 4'd11,
    REG_WEEKDAY      = 4'd12,
    REG_CONTROL      = 4'd13,
    REG_KEY          = 4'd14,
    REG_STATUS       = 4'd15
} rtc_register_t;

localparam KEY_UNLOCK = 4'd5;                   /* value of register E that allows writing the other registers */

/*
    ---- the 1 second count
*/
reg [24:0] divider = 25'd0;
reg        carry_request = 1'b0;                /* a seconds reset at 30 s or more asks for one extra count with the seconds forced to 00 */

wire divider_done = (divider == CLK_HZ - 1);
wire count_now    = divider_done | carry_request;               /* one count of the whole counter chain in this clock */

always @(posedge clk) begin
    if (divider_done)
        divider <= 25'd0;
    else
        divider <= divider + 25'd1;
end

/* -------------------------------------------------------------------------------------------------------------------------------- bus interface */
wire       rtc_ce   = wdata[15];                /* RTCCE */
wire       rtc_ale  = wdata[14];                /* RTCALE */
wire       rtc_rw   = wdata[13];                /* RTCR/W */
wire [3:0] rtc_data = wdata[11:8];              /* address or data nibble */

reg  [3:0] register_address = 4'd0;             /* latched by ALE */
reg  [3:0] key              = 4'd0;             /* register E */

wire latch_address  = wr_go & rtc_ale;
wire register_write = wr_go & ~rtc_ale & rtc_ce & rtc_rw;       /* CE and R/W high, ALE low: write the nibble to the addressed register */

wire unlocked = (key == KEY_UNLOCK);

always @(posedge clk) begin
    if (rst)
        key <= 4'd0;                                            /* the chip is locked again by reset */
    else if (register_write && register_address == REG_KEY)
        key <= rtc_data;                                        /* the key register itself is always writable */

    if (latch_address)
        register_address <= rtc_data;
end

/*
    A write to a time register is accepted only while unlocked (registers E and F are handled on their own), and not in the very clock in which the
    counters count: the count wins, the write is lost (the chip is busy then).
*/
wire write_time_register = register_write & unlocked & ~count_now & (register_address != REG_KEY) & (register_address != REG_STATUS);
wire host_load           = set_en & ~count_now;                    /* the same for a load of the host clock */

/* the register the CPU writes (one-hot, by register number); the control register is not part of the count and also takes a write in a counting clock */
wire [15:0] write_register = write_time_register ? (16'd1 << register_address) : 16'd0;
wire        write_control  = register_write & unlocked & (register_address == REG_CONTROL);

/* Writing 1 to D1 of register F resets the seconds. */
wire seconds_reset = register_write & unlocked & (register_address == REG_STATUS) & rtc_data[1];

/*
    ---- the counter chain
    Six BCD fields, seconds, minutes, hours, day, month and year, in the registers 2n (ones) and 2n+1 (tens). The count of one field is the wrap of the one
    below it, and a field wraps when it holds its last value (">=" so that a digit written out of range still wraps at the next count).
*/
function integer tens_width(input integer field);                      /* the tens digit has fewer bits where the chip leaves the top bits unused */
    case (field)
        0, 1:    tens_width = 3;
        2, 3:    tens_width = 2;
        4:       tens_width = 1;
        default: tens_width = 4;
    endcase
endfunction

function integer first_ones(input integer field);                      /* the day and the month wrap to 01, the others to 00 */
    first_ones = (field == 3 || field == 4) ? 1 : 0;
endfunction

function integer last_value(input integer field);                      /* the last value of a field (the day is decided by the month below) */
    case (field)
        0, 1:    last_value = 59;
        2:       last_value = 23;
        4:       last_value = 12;
        default: last_value = 99;
    endcase
endfunction

function [6:0] bcd_value(input [3:0] tens, input [3:0] ones);          /* a BCD pair as a plain number */
    bcd_value = tens * 7'd10 + ones;
endfunction

wire [23:0] ones_of_field;                                              /* the digits of the field n in the nibble n, the tens zero extended */
wire [23:0] tens_of_field;
wire [5:0]  count_field;
wire [5:0]  last_of_field;                                              /* the field holds its last value */
wire        day_at_last;

assign count_field[0] = count_now;

genvar field;
generate
    for (field = 0; field < 6; field = field + 1) begin : counter
        reg [3:0]                    ones = (field == 5) ? 4'd6 : first_ones(field);     /* the clock starts at 1 January 1986 */
        reg [tens_width(field)-1:0]  tens = (field == 5) ? 4'd8 : 4'd0;

        /* from the lowest to the highest priority, as in the chip: host clock load, CPU write of a digit, CPU clear, count (a count wins over a write) */
        always @(posedge clk) begin
            if (host_load) begin
                ones <= set_val[8*field +: 4];
                tens <= set_val[8*field + 4 +: tens_width(field)];
            end

            if (write_register[2*field])
                ones <= rtc_data;

            if (write_register[2*field + 1])
                tens <= rtc_data[tens_width(field)-1:0];

            if (field == 0 && seconds_reset) begin                      /* only the seconds can be cleared by the CPU */
                ones <= 4'd0;
                tens <= 4'd0;
            end

            if (count_field[field]) begin
                if (last_of_field[field]) begin                         /* wrap to the first value */
                    ones <= first_ones(field);
                    tens <= 4'd0;
                end
                else if (ones == 4'd9) begin
                    ones <= 4'd0;
                    tens <= tens + 1'b1;
                end
                else begin
                    ones <= ones + 4'd1;
                    tens <= tens;                                       /* the count also overrides a write or clear of the digit that does not change */
                end
            end
        end

        assign ones_of_field[4*field +: 4] = ones;
        assign tens_of_field[4*field +: 4] = tens;

        if (field == 3)
            assign last_of_field[field] = day_at_last;
        else if (field == 0)
            assign last_of_field[field] = carry_request | (bcd_value(tens_of_field[3:0], ones_of_field[3:0]) >= last_value(field));   /* or forced by a seconds reset */
        else
            assign last_of_field[field] = bcd_value(tens_of_field[4*field +: 4], ones_of_field[4*field +: 4]) >= last_value(field);

        if (field > 0)
            assign count_field[field] = count_field[field - 1] & last_of_field[field - 1];
    end
endgenerate

wire [3:0] seconds_ones = ones_of_field[3:0];
wire [2:0] seconds_tens = tens_of_field[2:0];
wire [3:0] minutes_ones = ones_of_field[7:4];
wire [2:0] minutes_tens = tens_of_field[6:4];
wire [3:0] hours_ones   = ones_of_field[11:8];
wire [1:0] hours_tens   = tens_of_field[9:8];
wire [3:0] day_ones     = ones_of_field[15:12];
wire [1:0] day_tens     = tens_of_field[13:12];
wire [3:0] month_ones   = ones_of_field[19:16];
wire       month_tens   = tens_of_field[16];
wire [3:0] year_ones    = ones_of_field[23:20];
wire [3:0] year_tens    = tens_of_field[23:20];

/*
    ---- leap year and the length of the month
    The chip keeps "year mod 4" in L1 L0 (register 9, written by software) and compares it with the year: a match means leap year.
*/
wire [1:0] year_mod_4 = year_ones[1:0] + {year_tens[0], 1'b0};          /* the year is BCD: year = 10 * tens + ones, and 10 mod 4 = 2 */
wire [1:0] leap_year_selector;                                          /* L1 L0, register 9 D3:D2 */
wire       leap_year = (year_mod_4 == leap_year_selector);

reg [6:0] days_in_month;
always @* begin
    case (bcd_value(month_tens, month_ones))
        7'd2:                         days_in_month = leap_year ? 7'd29 : 7'd28;
        7'd4, 7'd6, 7'd9, 7'd11:      days_in_month = 7'd30;
        default:                      days_in_month = 7'd31;
    endcase
end

assign day_at_last = (bcd_value(day_tens, day_ones) >= days_in_month);  /* ">=" also moves a day past the end of a short month to the 1st */

/* ---- register 9: L1 L0 and the bit between them and the month tens */
reg [2:0] register_9_upper = 3'd0;                                      /* D3:D2 = L1 L0, D1 = stored, cleared by every count */
assign leap_year_selector = register_9_upper[2:1];

always @(posedge clk) begin
    if (host_load)
        register_9_upper <= set_val[39:37];

    if (write_register[REG_MONTH_TENS])
        register_9_upper <= rtc_data[3:1];

    if (count_now)
        register_9_upper[0] <= 1'b0;                          /* cleared by every count */
end

/* ---- weekday (0..6) and the control register */
reg [2:0] weekday = 3'd3;
reg [3:0] control = 4'd0;

wire weekday_at_last = (weekday == 3'd6);

always @(posedge clk) begin
    if (host_load)
        weekday <= set_val[50:48];

    if (write_register[REG_WEEKDAY])
        weekday <= rtc_data[2:0];

    if (count_field[3])
        weekday <= weekday_at_last ? 3'd0 : weekday + 3'd1;

    if (write_control)
        control <= rtc_data;
end

/* ---- the extra count after a seconds reset (at 30 s or more) and the BUSY / XBUSY flags */
localparam BUSY_CLOCKS = 3640;                  /* BUSY stays high for 3640 clocks (182 us at 20 MHz) after each count of the seconds */

reg        busy  = 1'b0;
reg        xbusy = 1'b0;
reg [11:0] busy_clocks_left = 12'd0;

wire status_read    = rd_go & (register_address == REG_STATUS);
wire start_busy     = count_now | seconds_reset;                        /* a count of the seconds, or a seconds reset, makes the chip busy */

always @(posedge clk) begin
    if (busy_clocks_left != 12'd0) begin
        busy_clocks_left <= busy_clocks_left - 12'd1;
        if (busy_clocks_left == 12'd1)
            busy <= 1'b0;
    end

    if (status_read)
        xbusy <= 1'b0;

    if (start_busy) begin
        busy             <= 1'b1;
        xbusy            <= 1'b1;
        busy_clocks_left <= BUSY_CLOCKS;
    end

    /*
        A seconds reset at 30 s or more asks for one extra count (with the seconds forced to 00, which carries into the minutes). The request is
        served by the next clock; a request made in the clock that serves the previous one is dropped.
    */
    if (seconds_reset)
        carry_request <= (seconds_tens >= 3'd3);
    if (carry_request)
        carry_request <= 1'b0;
end

/* -------------------------------------------------------------------------------------------------------------------------------- read port */
reg [3:0] register_value;
always @* begin
    case (register_address)
        REG_MONTH_TENS: register_value = {register_9_upper[2:1], leap_year, month_tens};   /* D1 reads back as LY, not as the stored bit */
        REG_WEEKDAY:    register_value = {1'b0, weekday};
        REG_CONTROL:    register_value = control;
        REG_KEY:        register_value = key;
        REG_STATUS:     register_value = {2'b00, busy, xbusy};
        default:        register_value = register_address[0] ? tens_of_field[4*register_address[3:1] +: 4] : ones_of_field[4*register_address[3:1] +: 4];
    endcase
end

assign rdata = {4'h0, register_value, 4'h0, register_value};          /* the nibble appears on D3..D0 and on D11..D8 */

endmodule
