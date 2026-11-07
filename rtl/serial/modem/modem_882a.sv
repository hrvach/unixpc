/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The 882A modem chip on the 3B1 telephone board (analog side not modelled): its write registers and the three status registers the kernel reads.

    Writes (address bits A15:A12 of the E6xxxx block choose the register): W0, W1, W3, W4, W5, W6, W8, W9. They are only stored, the harness decodes
    the bits it needs (dialling, speed, loopback) from the outputs. Reads: R2, R3 and R10 are built from the far end state (carrier) and from DTR in
    W0.

    What the kernel uses (machdep.c chipinit, io/phsub.c):
      W0 bit 2   DTR: data terminal ready; without it the chip reports nothing
      W1 bit 0   self test (diagnostic), W1 bit 2 local analog loopback, W1 bit 4 1200 baud
      R2  bit 5 data mode, bit 4 DSR (follows DTR), bits 3:2 speed (01 = 1200 baud), bit 1 handshake done, bit 0 CTS; idle with DTR on = 0x10
      R3  bit 3 energy on the line (answer tone / carrier)
      R10 bit 5 data mode, bit 3 answer tone detected (the kernel only tests 0x20, 0x08 and 0x1e)
*/
module modem_882a
(
    input         clk,
    input         rst,

    input         write,            /* the CPU writes a register of this chip (E6xxxx) */
    input  [3:0]  register_select,  /* address bits A15:A12 */
    input  [15:0] wdata,

    input         carrier,          /* the far end modem is connected (harness): data carrier on the line */

    output reg [15:0] w0 = 16'd0,   /* the stored write registers */
    output reg [15:0] w1 = 16'd0,
    output reg [15:0] w3 = 16'd0,
    output reg [15:0] w4 = 16'd0,
    output reg [15:0] w5 = 16'd0,
    output reg [15:0] w6 = 16'd0,
    output reg [15:0] w8 = 16'd0,
    output reg [15:0] w9 = 16'd0,

    output [15:0] read_data,        /* R2, R3 or R10 chosen by register_select (all other reads give 0) */
    output        loopback          /* local analog loopback / self test active: data from the modem transmitter returns to its receiver */
);

/* ---- registers: select codes of the E6xxxx block (address bits A15:A12) */
localparam SEL_W0  = 4'h0;
localparam SEL_W1  = 4'h1;
localparam SEL_R2  = 4'h2;
localparam SEL_R3  = 4'h3;
localparam SEL_W3  = 4'h3;      /* reads and writes of the same select code reach different registers */
localparam SEL_W4  = 4'h4;
localparam SEL_W5  = 4'h5;
localparam SEL_W6  = 4'h6;
localparam SEL_W8  = 4'h8;
localparam SEL_W9  = 4'h9;
localparam SEL_R10 = 4'hA;

always @(posedge clk) begin
    if (rst) begin
        w0 <= 16'd0;
        w1 <= 16'd0;
        w3 <= 16'd0;
        w4 <= 16'd0;
        w5 <= 16'd0;
        w6 <= 16'd0;
        w8 <= 16'd0;
        w9 <= 16'd0;
    end
    else if (write) begin
        case (register_select)
            SEL_W0:  w0 <= wdata;
            SEL_W1:  w1 <= wdata;
            SEL_W3:  w3 <= wdata;
            SEL_W4:  w4 <= wdata;
            SEL_W5:  w5 <= wdata;
            SEL_W6:  w6 <= wdata;
            SEL_W8:  w8 <= wdata;
            SEL_W9:  w9 <= wdata;
            default: ;
        endcase
    end
end

/* ---- what the chip reports */
wire dtr_on          = w0[2];
wire connected       = carrier & dtr_on;            /* data mode: the far end is there and the terminal is ready */
wire self_test       = w1[0] & dtr_on;
wire speed_is_1200   = w1[4];

assign loopback = dtr_on & (w1[2] | w1[0]);

/* Self test reports test mode, handshake done and CTS (0x43, +0x04 at 1200 baud). */
wire [7:0] r2_self_test = {
    1'b0,
    1'b1,                                           /* bit 6 test mode */
    1'b0,
    dtr_on,                                         /* bit 4 DSR */
    speed_is_1200 ? 2'b01 : 2'b00,                  /* bits 3:2 speed */
    1'b1,                                           /* bit 1 handshake done */
    1'b1                                            /* bit 0 CTS */
};

wire [7:0] r2_normal = {
    2'b00,
    connected,                                      /* bit 5 data mode */
    dtr_on,                                         /* bit 4 DSR */
    connected ? 2'b01 : 2'b00,                      /* bits 3:2 speed (the harness only offers 1200 here) */
    connected,                                      /* bit 1 handshake done */
    connected                                       /* bit 0 CTS */
};

wire [7:0] r2  = self_test ? r2_self_test : r2_normal;
wire [7:0] r3  = {4'b0000, carrier, 3'b000};        /* bit 3 energy on the line */
wire [7:0] r10 = {
    1'b0,
    connected,                                      /* bit 6 mirrors bit 5 */
    connected,                                      /* bit 5 data mode */
    1'b0,
    carrier,                                        /* bit 3 answer tone detected */
    3'b000
};

reg [15:0] selected_register;
always @* begin
    case (register_select)
        SEL_R2:  selected_register = {8'h00, r2};
        SEL_R3:  selected_register = {8'h00, r3};
        SEL_R10: selected_register = {8'h00, r10};
        default: selected_register = 16'h0000;
    endcase
end

assign read_data = selected_register;

endmodule
