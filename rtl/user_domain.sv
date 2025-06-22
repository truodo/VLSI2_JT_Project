// Copyright 2024 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Philippe Sauter <phsauter@iis.ee.ethz.ch>

module user_domain import user_pkg::*; import croc_pkg::*; #(
  parameter int unsigned GpioCount = 16
) (
  input  logic      clk_i,
  input  logic      ref_clk_i,
  input  logic      rst_ni,
  input  logic      testmode_i,

  // QSPI - Flash
  output  logic           flash_sck_o,
  output  logic           flash_ce_n_o,
  input   logic [3:0]     flash_din_i,
  output  logic [3:0]     flash_dout_o,
  output  logic [3:0]     flash_douten_o,
  
  input  sbr_obi_req_t user_sbr_obi_req_i, // User Sbr (rsp_o), Croc Mgr (req_i)
  output sbr_obi_rsp_t user_sbr_obi_rsp_o,

  output mgr_obi_req_t user_mgr_obi_req_o, // User Mgr (req_o), Croc Sbr (rsp_i)
  input  mgr_obi_rsp_t user_mgr_obi_rsp_i,

  input  logic [      GpioCount-1:0] gpio_in_sync_i, // synchronized GPIO inputs
  output logic [NumExternalIrqs-1:0] interrupts_o // interrupts to core
);

  // -----------------
  // Control Signals
  // -----------------
  logic                flash_HSEL;
  logic [31:0]         flash_HADDR;
  logic [1:0]          flash_HTRANS;
  logic                flash_HWRITE;
  logic                flash_HREADY;
  logic                flash_HREADYOUT;
  logic [31:0]         flash_HRDATA;



  assign interrupts_o = '0;  


  //////////////////////
  // User Manager MUX //
  /////////////////////

  // No manager so we don't need a obi_mux module and just terminate the request properly
  assign user_mgr_obi_req_o = '0;


  ////////////////////////////
  // User Subordinate DEMUX //
  ////////////////////////////

  // ----------------------------------------------------------------------------------------------
  // User Subordinate Buses
  // ----------------------------------------------------------------------------------------------
  
  // collection of signals from the demultiplexer
  sbr_obi_req_t [NumDemuxSbr-1:0] all_user_sbr_obi_req;
  sbr_obi_rsp_t [NumDemuxSbr-1:0] all_user_sbr_obi_rsp;

  // Flash Subordinate Bus
  sbr_obi_req_t user_flash_obi_req;
  sbr_obi_rsp_t user_flash_obi_rsp;
  
  // Error Subordinate Bus
  sbr_obi_req_t user_error_obi_req;
  sbr_obi_rsp_t user_error_obi_rsp;

  // Fanout into more readable signals
  assign user_error_obi_req              = all_user_sbr_obi_req[UserError];
  assign all_user_sbr_obi_rsp[UserError] = user_error_obi_rsp;

  assign user_flash_obi_req              = all_user_sbr_obi_req[UserFlash];
  assign all_user_sbr_obi_rsp[UserFlash] = user_flash_obi_rsp;

  //-----------------------------------------------------------------------------------------------
  // Demultiplex to User Subordinates according to address map
  //-----------------------------------------------------------------------------------------------

  logic [cf_math_pkg::idx_width(NumDemuxSbr)-1:0] user_idx;

  addr_decode #(
    .NoIndices ( NumDemuxSbr                    ),
    .NoRules   ( NumDemuxSbrRules               ),
    .addr_t    ( logic[SbrObiCfg.DataWidth-1:0] ),
    .rule_t    ( addr_map_rule_t                ),
    .Napot     ( 1'b0                           )
  ) i_addr_decode_periphs (
    .addr_i           ( user_sbr_obi_req_i.a.addr ),
    .addr_map_i       ( user_addr_map             ),
    .idx_o            ( user_idx                  ),
    .dec_valid_o      (),
    .dec_error_o      (),
    .en_default_idx_i ( 1'b1 ),
    .default_idx_i    ( '0   )
  );

  obi_demux #(
    .ObiCfg      ( SbrObiCfg     ),
    .obi_req_t   ( sbr_obi_req_t ),
    .obi_rsp_t   ( sbr_obi_rsp_t ),
    .NumMgrPorts ( NumDemuxSbr   ),
    .NumMaxTrans ( 2             )
  ) i_obi_demux (
    .clk_i,
    .rst_ni,

    .sbr_port_select_i ( user_idx             ),
    .sbr_port_req_i    ( user_sbr_obi_req_i   ),
    .sbr_port_rsp_o    ( user_sbr_obi_rsp_o   ),

    .mgr_ports_req_o   ( all_user_sbr_obi_req ),
    .mgr_ports_rsp_i   ( all_user_sbr_obi_rsp )
  );


//-------------------------------------------------------------------------------------------------
// User Subordinates
//-------------------------------------------------------------------------------------------------

  // Error Subordinate
  obi_err_sbr #(
    .ObiCfg      ( SbrObiCfg     ),
    .obi_req_t   ( sbr_obi_req_t ),
    .obi_rsp_t   ( sbr_obi_rsp_t ),
    .NumMaxTrans ( 1             ),
    .RspData     ( 32'hBADCAB1E  )
  ) i_user_err (
    .clk_i,
    .rst_ni,
    .testmode_i ( testmode_i      ),
    .obi_req_i  ( user_error_obi_req ),
    .obi_rsp_o  ( user_error_obi_rsp )
  );

  `ifdef PnR
  obi2ahbm_adapter i_obi2ahbm_adapter_flash (
       // Clock and reset
       .hclk_i      ( clk_i ),                // (I) AHB clock
       .hresetn_i   ( rst_ni ),               // (I) AHB reset, active LOW
       
       // AHB master interface
       .haddr_o       ( flash_HADDR ),        // (O) 32-bit AHB system address bus
       .hburst_o      (  ),                   // (O) Burst type 
       .hmastlock_o   (  ),                   // (O) Sequence lock
       .hprot_o       (  ),                   // (O) Protection control
       .hsize_o       (  ),                   // (O) Transfer size
       .htrans_o      ( flash_HTRANS ),       // (O) Transfer type
       .hwdata_o      (  ),                   // (O) 32-bit AHB write data bus
       .hwrite_o      ( flash_HWRITE ),       // (O) Transfer direction
       .hrdata_i      ( flash_HRDATA ),       // (I) 32-bit AHB read data bus
       .hready_i      ( flash_HREADYOUT ),    // (I) Status of transfer
       .hresp_i       ( 1'b0 ), // No error   // (I) Transfer response
       
       // Data interface from core
       .data_req_i    ( user_flash_obi_req.req ),     // (I) Request ready
       .data_gnt_o    ( user_flash_obi_rsp.gnt ),     // (O) The other side accepted the request
       .data_rvalid_o ( user_flash_obi_rsp.rvalid ),  // (O) Read data valid when high
       .data_we_i     ( user_flash_obi_req.a.we ),    // (I) Write enable (active HIGH)
       .data_be_i     ( user_flash_obi_req.a.be ),    // (I) Byte enable
       .data_addr_i   ( user_flash_obi_req.a.addr ),  // (I) Address
       .data_wdata_i  ( user_flash_obi_req.a.wdata ), // (I) Write data
       .data_rdata_o  ( user_flash_obi_rsp.r.rdata ), // (O) Read data
       .data_err_o    ( user_flash_obi_rsp.r.err ),   // (O) Error
       .pending_dbus_xfer_i   ( 1'b0 ), // (I) Asserted if data bus is busy from other transactions

       // Miscellaneous
       .priv_mode_i   ( 1'b1 )       // (I) Privilege mode (from core. 1=machine mode, 0=user mode)
    );

    assign flash_HSEL = periph_idx == PeriphFlash;
    assign flash_HREADY = flash_HREADYOUT;

    assign user_flash_obi_rsp.r.rid = flash_obi_req.a.aid;
    assign user_flash_obi_rsp.r.r_optional = 1'b0;

    EF_QSPI_XIP_CTRL_AHBL 
    #(
        .NUM_LINES      ( 8 ), 
        .LINE_SIZE      ( 32 ), 
        .RESET_CYCLES   ( 999 ) 
    )
    i_EF_QSPI_XIP_CTRL_AHBL
    (
        // AHB-Lite Slave Interface
        .HCLK        ( clk_i  ),
        .HRESETn     ( rst_ni ),

        .HSEL        ( flash_HSEL      ),
        .HADDR       ( flash_HADDR     ),
        .HTRANS      ( flash_HTRANS    ),
        .HWRITE      ( flash_HWRITE    ),
        .HREADY      ( flash_HREADY    ),
        .HREADYOUT   ( flash_HREADYOUT ),
        .HRDATA      ( flash_HRDATA    ),

        // External Interface to Quad I/O
        .sck     ( flash_sck    ),
        .ce_n    ( flash_ce_n   ),
        .din     ( flash_din    ),
        .dout    ( flash_dout   ),
        .douten  ( flash_douten )
    );

    `else

    // Use a ROM for FPGA
    
    localparam RomAddrWidth = 12; // in words, in total 16kByte memory

    logic rom_req, rom_we, rom_gnt, rom_single_err;
    logic [SbrObiCfg.AddrWidth-1:0] rom_byte_addr;
    logic [RomAddrWidth-1:0] rom_word_addr;
    logic [SbrObiCfg.DataWidth-1:0] rom_wdata, rom_rdata;
    logic [SbrObiCfg.DataWidth/8-1:0] rom_be;

    obi_sram_shim #(
      .ObiCfg    ( SbrObiCfg     ),
      .obi_req_t ( sbr_obi_req_t ),
      .obi_rsp_t ( sbr_obi_rsp_t )
    ) i_rom_shim (
      .clk_i,
      .rst_ni,

      .obi_req_i ( user_flash_obi_req ),
      .obi_rsp_o ( user_flash_obi_rsp ),

      .req_o   ( rom_req       ),
      .we_o    ( rom_we        ),
      .addr_o  ( rom_byte_addr ),
      .wdata_o ( rom_wdata     ),
      .be_o    ( rom_be        ),

      .gnt_i   ( rom_gnt   ),
      .rdata_i ( rom_rdata )
    );

    assign rom_word_addr = rom_byte_addr[SbrObiCfg.AddrWidth-1:2];

    // 4kByte memory
    logic [31:0] rom [2**RomAddrWidth];
    
    // 	initial begin
		// $readmemh("firmware/hello_world/hello_world.hex", rom);
	// end
    
    always @(posedge clk_i) begin
        if (rom_req) begin
            if (!rom_we) begin
                rom_rdata <= rom[rom_word_addr];
            end
        end
    end

    assign rom_gnt = 1'b1;
    
    `endif
  

endmodule
