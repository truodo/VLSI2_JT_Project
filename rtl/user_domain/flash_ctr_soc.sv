module flash_ctr_soc import flash_ctr_pkg::*, croc_pkg::*;
(
    // QSPI - Flash
    output  logic           flash_sck,
    output  logic           flash_ce_n,
    input   logic [3:0]     flash_din,
    output  logic [3:0]     flash_dout,
    output  logic [3:0]     flash_douten,
);

    // TODO move all this into user_domain.sv and obi2ahbm_adapter.sv should be moved into a newly created flash folder 
    // Flash bus
    sbr_obi_req_t flash_obi_req;
    sbr_obi_rsp_t flash_obi_rsp;

    // QSPI XIP

    `ifdef PnR

    logic                flash_HSEL;
    logic [31:0]         flash_HADDR;
    logic [1:0]          flash_HTRANS;
    logic                flash_HWRITE;
    logic                flash_HREADY;
    logic                flash_HREADYOUT;
    logic [31:0]         flash_HRDATA;

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
       .data_req_i    ( flash_obi_req.req ),     // (I) Request ready
       .data_gnt_o    ( flash_obi_rsp.gnt ),     // (O) The other side accepted the request
       .data_rvalid_o ( flash_obi_rsp.rvalid ),  // (O) Read data valid when high
       .data_we_i     ( flash_obi_req.a.we ),    // (I) Write enable (active HIGH)
       .data_be_i     ( flash_obi_req.a.be ),    // (I) Byte enable
       .data_addr_i   ( flash_obi_req.a.addr ),  // (I) Address
       .data_wdata_i  ( flash_obi_req.a.wdata ), // (I) Write data
       .data_rdata_o  ( flash_obi_rsp.r.rdata ), // (O) Read data
       .data_err_o    ( flash_obi_rsp.r.err ),   // (O) Error
       .pending_dbus_xfer_i   ( 1'b0 ), // (I) Asserted if data bus is busy from other transactions

       // Miscellaneous
       .priv_mode_i   ( 1'b1 )       // (I) Privilege mode (from core. 1=machine mode, 0=user mode)
    );

    assign flash_HSEL = periph_idx == PeriphFlash;
    assign flash_HREADY = flash_HREADYOUT;

    assign flash_obi_rsp.r.rid = flash_obi_req.a.aid;
    assign flash_obi_rsp.r.r_optional = 1'b0;
    
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

      .obi_req_i ( flash_obi_req ),
      .obi_rsp_o ( flash_obi_rsp ),

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