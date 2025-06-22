

//TODO delete this later
package flash_ctr_pkg;

    import croc_pkg::*;

    // Address map data type
    typedef struct packed {
        logic [31:0] idx;
        logic [31:0] start_addr;
        logic [31:0] end_addr;
    } addr_map_rule_t;



    /////////////////////////////
    // Peripheral address map ///
    /////////////////////////////

    localparam bit [31:0] FlashAddrOffset           = 32'h0000_0000;
    localparam bit [31:0] FlashAddrRange            = 32'h0100_0000; // 24bit

    typedef enum int {
        UserPeriphError = 0,
        UserPeriphFlash = 1
    } user_periph_outputs_e;


    localparam addr_map_user_rule_t [1:0] user_periph_addr_map = '{
    '{ idx: UserPeriphFlash,
        start_addr: UserBaseAddr + FlashAddrOffset,
        end_addr:   UserBaseAddr + FlashAddrRange }
    };
    
endpackage
