/*
    Copyright (c) 2026 by Hrvoje Cavrak

    Register numbers and command codes of the two disk controller chips on the logic board, from the data sheets (names as printed there). The models
    wd1010_controller.sv (hard disk) and wd2797_controller.sv (floppy) import them; the numbers are what the CPU writes to the chip, so the
    names appear in the kernel's and the diagnostic's source as well.
*/
package disk_controllers_pkg;

    /*
        WD2797 floppy disk controller. The CPU reaches four registers with address bits A2..A1 (E1000x). A read of the first gives the status, a
        write is a command.
    */
    typedef enum logic [1:0] {
        WD2797_REGISTER_STATUS_COMMAND = 2'd0,
        WD2797_REGISTER_TRACK          = 2'd1,
        WD2797_REGISTER_SECTOR         = 2'd2,
        WD2797_REGISTER_DATA           = 2'd3
    } wd2797_register_t;

    /*
        WD2797 commands: the upper four bits of the command byte. The lower four bits are options (head load, verify, step rate, multiple
        sectors, update track register) that this model mostly ignores; the pairs STEP / STEP_UPDATE, READ_SECTOR / READ_SECTOR_MULTIPLE and so on
        differ in one option bit.
    */
    typedef enum logic [3:0] {
        WD2797_COMMAND_RESTORE               = 4'd0,
        WD2797_COMMAND_SEEK                  = 4'd1,
        WD2797_COMMAND_STEP                  = 4'd2,
        WD2797_COMMAND_STEP_UPDATE           = 4'd3,
        WD2797_COMMAND_STEP_IN               = 4'd4,
        WD2797_COMMAND_STEP_IN_UPDATE        = 4'd5,
        WD2797_COMMAND_STEP_OUT              = 4'd6,
        WD2797_COMMAND_STEP_OUT_UPDATE       = 4'd7,
        WD2797_COMMAND_READ_SECTOR           = 4'd8,
        WD2797_COMMAND_READ_SECTOR_MULTIPLE  = 4'd9,
        WD2797_COMMAND_WRITE_SECTOR          = 4'd10,
        WD2797_COMMAND_WRITE_SECTOR_MULTIPLE = 4'd11,
        WD2797_COMMAND_READ_ADDRESS          = 4'd12,
        WD2797_COMMAND_FORCE_INTERRUPT       = 4'd13,
        WD2797_COMMAND_READ_TRACK            = 4'd14,
        WD2797_COMMAND_WRITE_TRACK           = 4'd15     /* "format" */
    } wd2797_command_t;


    /*
        WD1010 hard disk controller: eight registers, chosen by address bits A3..A1 (E0000x).
    */
    typedef enum logic [2:0] {
        WD1010_REGISTER_DATA           = 3'd0,
        WD1010_REGISTER_ERROR          = 3'd1,       /* read: error register, write: write precompensation (not modelled) */
        WD1010_REGISTER_SECTOR_COUNT   = 3'd2,
        WD1010_REGISTER_SECTOR_NUMBER  = 3'd3,
        WD1010_REGISTER_CYLINDER_LOW   = 3'd4,
        WD1010_REGISTER_CYLINDER_HIGH  = 3'd5,
        WD1010_REGISTER_SDH            = 3'd6,       /* sector size, drive, head */
        WD1010_REGISTER_STATUS_COMMAND = 3'd7        /* read: status, write: command */
    } wd1010_register_t;

    /*
        WD1010 commands: the upper four bits of the command byte. Code 0 is the extended command group (compute correction), which this model does
        not implement; 6 is unused.
    */
    typedef enum logic [3:0] {
        WD1010_COMMAND_EXTENDED     = 4'd0,
        WD1010_COMMAND_RESTORE      = 4'd1,
        WD1010_COMMAND_READ_SECTOR  = 4'd2,
        WD1010_COMMAND_WRITE_SECTOR = 4'd3,
        WD1010_COMMAND_SCAN_ID      = 4'd4,
        WD1010_COMMAND_WRITE_FORMAT = 4'd5,
        WD1010_COMMAND_SEEK         = 4'd7
    } wd1010_command_t;

endpackage
