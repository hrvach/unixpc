/*
    Copyright (c) 2026 by Hrvoje Cavrak

    The colours of the printer paper. The paper view (lp_paper_view) draws with them; the video output keeps only the number of the colour for every
    pixel of the paper (three bits instead of 24) and turns it back into the colour when it draws the pixel.
*/
package paper_palette_pkg;

typedef enum logic [2:0]
{
    PAPER_BLANKING    = 3'd0,
    PAPER_WHITE       = 3'd1,       /* white paper, also the sprocket strips */
    PAPER_GREEN_BAR   = 3'd2,
    PAPER_INK         = 3'd3,
    PAPER_HOLE        = 3'd4,       /* punched sprocket holes */
    PAPER_PERFORATION = 3'd5        /* the perforation between a strip and the sheet */
} paper_colour_t;

localparam [23:0] COLOR_BLANKING    = 24'h000000;
localparam [23:0] COLOR_PAPER       = 24'hf0f0ea;
localparam [23:0] COLOR_GREEN_BAR   = 24'h8fd0aa;
localparam [23:0] COLOR_INK         = 24'h181c20;
localparam [23:0] COLOR_HOLE        = 24'h16181c;
localparam [23:0] COLOR_PERFORATION = 24'hb4b4ac;

/* the number of a colour of the paper (the paper view draws only these six; anything else counts as blanking) */
function paper_colour_t paper_colour_number(input [23:0] colour);
    case (colour)
        COLOR_PAPER:       paper_colour_number = PAPER_WHITE;
        COLOR_GREEN_BAR:   paper_colour_number = PAPER_GREEN_BAR;
        COLOR_INK:         paper_colour_number = PAPER_INK;
        COLOR_HOLE:        paper_colour_number = PAPER_HOLE;
        COLOR_PERFORATION: paper_colour_number = PAPER_PERFORATION;
        default:           paper_colour_number = PAPER_BLANKING;
    endcase
endfunction

/* the colour of a number */
function [23:0] paper_colour_of(input paper_colour_t number);
    case (number)
        PAPER_WHITE:       paper_colour_of = COLOR_PAPER;
        PAPER_GREEN_BAR:   paper_colour_of = COLOR_GREEN_BAR;
        PAPER_INK:         paper_colour_of = COLOR_INK;
        PAPER_HOLE:        paper_colour_of = COLOR_HOLE;
        PAPER_PERFORATION: paper_colour_of = COLOR_PERFORATION;
        default:           paper_colour_of = COLOR_BLANKING;
    endcase
endfunction

endpackage
