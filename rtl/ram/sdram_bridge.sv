/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Clock domain bridge between the UNIX PC board (a single 20 MHz clock, toggle handshake RAM port, see ram_port.sv with EXT_RAM = 1) and the 16 bit
    SDRAM controller sdram16 (sdram.sv) running at about 100 MHz. FPGA only.

    One transaction at a time. The board flips request_toggle and holds the address, write flag, byte enables and data stable until acknowledge_toggle
    equals request_toggle. The two clocks are phase locked (same PLL, 1:5), so the toggles are used directly, without synchronisers (see below); the
    payload is registered on the board side at the launch edge and stable for the whole transaction. The read data stays in read_data until the next
    transaction.

    Bad parity flags (the diagnostic parity test): one flag per byte of the lower 2 MB, kept in the SDRAM itself above the RAM (byte address 16 MB,
    256 KB, 16 flags per word: flag word = word address [19:3], flag bits {word address [2:0], lane}). The flags are only touched while
    parity_flags_in_use is set (the board sets it at the first write with bad parity forced, so normal operation costs nothing): a write then also
    updates the flags of its byte lanes (read, modify, write; store_bad_parity is the value written), a read also returns the flags of its two bytes
    in bad_parity_flags. The whole flag area is cleared after every reset that follows a flag use (or power-up).
*/
module sdram_bridge
(
    input             clk,                      /* board clock (20 MHz) */
    input             clk_sd,                   /* SDRAM controller clock (100 MHz, from the same PLL) */
    input             rst,                      /* board reset (any clock, synchronised here) */

    /* the board's RAM port */
    input             request_toggle,
    input             write,
    input      [21:1] word_address,
    input      [1:0]  byte_enables,             /* byte enables of a write, bit 1 = high (even) byte */
    input      [15:0] write_data,
    input             parity_flags_in_use,      /* bad parity flags in use (see above) */
    input             store_bad_parity,         /* value stored into the flags of the bytes written */
    output reg [15:0] read_data,
    output reg [1:0]  bad_parity_flags,         /* flags of the bytes just read (bit 1 = high byte), valid with acknowledge_toggle */
    output            acknowledge_toggle,

    /* the sdram16 controller */
    output            sd_init,
    output reg        sd_rd,
    output reg        sd_we,
    output reg [24:0] sd_addr,
    output reg [1:0]  sd_wtbt,
    output reg [15:0] sd_din,
    input      [15:0] sd_dout,
    input             sd_ready,
    input             sd_idle_ok,               /* the controller is idle and no refresh is near (sdram.sv) */
    output reg        fast_ok                   /* a read launched now is answered in the shortest possible time: the board may release DTACK* early */
);

/* ---- the reset and the request toggle come over from the board clock, the acknowledge goes back */
reg [2:0] reset_synchroniser = 3'b111;

always @(posedge clk_sd)
    reset_synchroniser <= {reset_synchroniser[1:0], rst};

assign sd_init = reset_synchroniser[2];

/*
    The two toggles are NOT synchronised. clk (20 MHz) and clk_sd (100 MHz) come from the same PLL with an exact 1:5 ratio and aligned edges, so
    a toggle launched by one clock is sampled by the other no later than one clk_sd period (10 ns) after the launching edge. The path is an
    ordinary single flop to flop path in static timing (3B1.sdc relates the two clocks) and needs to meet that 10 ns. Two flip flops in each
    direction used to cost about 120 ns per RAM access, a third of the 68010's 400 ns bus cycle.
*/
wire request_pending_toggle = request_toggle;

reg  acknowledge_in_sd_domain = 1'b0;

assign acknowledge_toggle = acknowledge_in_sd_domain;

/*
    States. Every request to the controller is a rd / we pulse (set in an ISSUE state, visible in the following WAIT_START state) followed by a wait
    for 'ready': WAIT_START lets the controller drop 'ready' for the new request, WAIT_READY waits for it to come back.
*/
typedef enum logic [3:0] {
    IDLE                     = 4'd0,
    ISSUE_DATA_ACCESS        = 4'd1,   /* the data word access */
    DATA_WAIT_START          = 4'd2,
    DATA_WAIT_READY          = 4'd3,
    ISSUE_FLAG_READ          = 4'd4,   /* the flag word read */
    FLAG_READ_WAIT_START     = 4'd5,
    FLAG_READ_WAIT_READY     = 4'd6,
    ISSUE_FLAG_WRITE         = 4'd7,   /* the flag word write */
    FLAG_WRITE_WAIT_START    = 4'd8,
    FLAG_WRITE_WAIT_READY    = 4'd9,
    ISSUE_CLEAR_WRITE        = 4'd10,   /* the flag area clear */
    CLEAR_WAIT_START         = 4'd11,
    CLEAR_WAIT_READY         = 4'd12,
    START_CLEAR              = 4'd13
} bridge_state_t;

bridge_state_t state = START_CLEAR;
reg       transaction_is_write = 1'b0;

/*
    'flag_area_dirty': the flag area may hold flags (or, after power-up, undefined SDRAM content), so the next reset must clear it. Set at power-up
    and by every flag write, cleared only when a complete clear has finished; reset itself does not touch it, so a reset without flag use in between
    skips the clear of about 8 ms.
*/
reg flag_area_dirty = 1'b1;

reg [15:0] flag_word = 16'd0;                           /* the flag word as read */

wire [16:0] flag_word_index = word_address[20:4];
wire [3:0]  low_lane_flag_bit = {word_address[3:1], 1'b0};      /* bit of the low byte lane of this word; the high lane is the next one */
wire [15:0] written_lanes_flag_mask = ({15'd0, byte_enables[0]} << low_lane_flag_bit) |
                                      ({15'd0, byte_enables[1]} << (low_lane_flag_bit + 4'd1));
wire        flags_needed = parity_flags_in_use & ~word_address[21];

/*
    The controller address is a register (no logic between it and sdram16). It is loaded one state before the rd / we pulse is issued and then holds:
    the data address in IDLE, the flag address when the flag access is decided (DATA_WAIT_READY) or the clear starts (START_CLEAR); during the clear
    it is also the counter (flag word index = sd_addr[17:1]).
*/
always @(posedge clk_sd) begin
    sd_rd <= 1'b0;
    sd_we <= 1'b0;

    /*
        The shortest read: this bridge idle with nothing pending, the controller idle with no refresh near, and the parity flags not in use (a read then
        also reads the flag word, a second access). The board samples it when it launches a read; nothing can make this untrue before the request arrives
        (the bridge and the controller leave IDLE only for a request, which only the board makes, one at a time).
    */
    fast_ok <= (state == IDLE) & (request_pending_toggle == acknowledge_in_sd_domain) & sd_idle_ok & ~parity_flags_in_use & ~sd_init;

    case (state)
        IDLE: begin
            if (request_pending_toggle != acknowledge_in_sd_domain) begin
                transaction_is_write <= write;
                sd_wtbt              <= byte_enables;
                sd_din               <= write_data;
                sd_addr              <= {3'b000, word_address, 1'b0};

                if (write && byte_enables == 2'b00)
                    acknowledge_in_sd_domain <= request_pending_toggle;         /* nothing to write */
                else
                    state <= ISSUE_DATA_ACCESS;
            end
        end

        ISSUE_DATA_ACCESS: begin                        /* a pulse of one clock, the controller detects the rising edge */
            sd_we <= transaction_is_write;
            sd_rd <= ~transaction_is_write;
            state <= DATA_WAIT_START;
        end

        DATA_WAIT_START: begin
            state <= DATA_WAIT_READY;
        end

        DATA_WAIT_READY: begin
            if (sd_ready) begin
                read_data        <= sd_dout;
                bad_parity_flags <= 2'b00;

                if (flags_needed) begin
                    sd_addr <= {1'b1, 6'd0, flag_word_index, 1'b0};
                    state   <= ISSUE_FLAG_READ;
                end
                else begin
                    acknowledge_in_sd_domain <= request_pending_toggle;
                    state                    <= IDLE;
                end
            end
        end

        ISSUE_FLAG_READ: begin
            sd_rd <= 1'b1;
            state <= FLAG_READ_WAIT_START;
        end

        FLAG_READ_WAIT_START: begin
            state <= FLAG_READ_WAIT_READY;
        end

        FLAG_READ_WAIT_READY: begin
            if (sd_ready) begin
                flag_word <= sd_dout;

                if (transaction_is_write) begin
                    state <= ISSUE_FLAG_WRITE;                  /* set or clear the flags of the lanes that are written */
                end
                else begin
                    bad_parity_flags         <= {sd_dout[low_lane_flag_bit + 4'd1], sd_dout[low_lane_flag_bit]};
                    acknowledge_in_sd_domain <= request_pending_toggle;
                    state                    <= IDLE;
                end
            end
        end

        ISSUE_FLAG_WRITE: begin
            sd_wtbt         <= 2'b11;
            sd_din          <= (flag_word & ~written_lanes_flag_mask) | (store_bad_parity ? written_lanes_flag_mask : 16'd0);
            sd_we           <= 1'b1;
            flag_area_dirty <= 1'b1;
            state           <= FLAG_WRITE_WAIT_START;
        end

        FLAG_WRITE_WAIT_START: begin
            state <= FLAG_WRITE_WAIT_READY;
        end

        FLAG_WRITE_WAIT_READY: begin
            if (sd_ready) begin
                acknowledge_in_sd_domain <= request_pending_toggle;
                state                    <= IDLE;
            end
        end

        START_CLEAR: begin
            sd_addr <= {1'b1, 24'd0};
            sd_wtbt <= 2'b11;
            sd_din  <= 16'd0;
            state   <= ISSUE_CLEAR_WRITE;
        end

        ISSUE_CLEAR_WRITE: begin
            sd_we <= 1'b1;
            state <= CLEAR_WAIT_START;
        end

        CLEAR_WAIT_START: begin
            state <= CLEAR_WAIT_READY;
        end

        CLEAR_WAIT_READY: begin
            if (sd_ready) begin
                if (&sd_addr[17:1]) begin                       /* the last flag word has been written */
                    flag_area_dirty <= 1'b0;
                    state           <= IDLE;
                end
                else begin
                    sd_addr <= sd_addr + 25'd2;
                    state   <= ISSUE_CLEAR_WRITE;
                end
            end
        end

        default: begin
            state <= IDLE;
        end
    endcase

    /* a reset starts the clear if the flag area was used since the last one */
    if (sd_init) begin
        state                    <= flag_area_dirty ? START_CLEAR : IDLE;
        acknowledge_in_sd_domain <= 1'b0;
    end
end

endmodule
