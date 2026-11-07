/*
    Copyright (c) 2026 by Hrvoje Cavrak

    What the modem bridge (modem_bridge.sv) is doing in a call, as a number that other parts of the core look at: line_sound.sv plays the call sounds
    from it, and the HPS side (telephony and the debug LEDs) shows its progress.
*/
package modem_pkg;

    typedef enum logic [2:0] {
        CALL_ON_HOOK             = 3'd0,
        CALL_OFF_HOOK            = 3'd1,       /* hook closed, digits are being dialled */
        CALL_SENDING_COMMAND     = 3'd2,       /* the dial or the hang up command goes to the UART */
        CALL_WAITING_FOR_CONNECT = 3'd3,
        CALL_DATA_MODE           = 3'd4
    } modem_call_state_t;

endpackage
