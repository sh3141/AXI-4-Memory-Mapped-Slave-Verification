interface axi4_if#(
		parameter int DATA_WIDTH       = 32,
		parameter int MEM_ADDR_WIDTH   = 10,
		parameter int SLAVE_ADDR_WIDTH = 16
	)(
		input logic ACLK
	);
	
	/// GLOBAL SIGNALS ///
	logic                        ARESETn;
	
	/// WRITE ADDRESS SIGNALS ///
	logic [SLAVE_ADDR_WIDTH-1:0] AWADDR; 
	logic [7:0]                  AWLEN; 
	logic [2:0]                  AWSIZE;
	logic                        AWVALID;
	logic                        AWREADY;
	
	/// WRITE DATA SIGNALS ///
	logic [DATA_WIDTH-1:0]       WDATA;
	logic 					     WLAST;
	logic 					     WVALID;
	logic 					     WREADY;
	
	/// WRITE RESPONSE SIGNALS ///
	logic [1:0]                  BRESP;
	logic 					     BVALID;
	logic 					     BREADY;
	
	/// READ ADDRESS SIGNALS ///
	logic [SLAVE_ADDR_WIDTH-1:0] ARADDR; 
	logic [7:0]                  ARLEN; 
	logic [2:0]                  ARSIZE;
	logic                        ARVALID;
	logic                        ARREADY;
	
	/// READ DATA SIGNALS ///
	logic [DATA_WIDTH-1:0]       RDATA;
	logic 					     RLAST;
	logic 					     RVALID;
	logic 					     RREADY;
	
	/// READ RESPONSE SIGNALS ///
	logic [1:0]                  RRESP;
	
	modport master(
		output ARESETn, 
		output AWADDR, AWLEN, AWSIZE, AWVALID, WDATA, WLAST, WVALID, 
		output BREADY,
		output ARADDR, ARLEN, ARSIZE, ARVALID, RREADY,
		input  ACLK, 
		input  AWREADY, WREADY, BVALID, BRESP,
		input  ARREADY, RDATA,  RLAST,  RVALID, RRESP
	);
	
	modport slave(
		input  ARESETn, 
		input  AWADDR, AWLEN, AWSIZE, AWVALID, WDATA, WLAST, WVALID, 
		input  BREADY,
		input  ARADDR, ARLEN, ARSIZE, ARVALID, RREADY,
		input  ACLK, 
		output AWREADY, WREADY, BVALID, BRESP,
		output ARREADY, RDATA,  RLAST,  RVALID, RRESP
	);
	
	modport assertions(
		input ARESETn, 
		input AWADDR, AWLEN, AWSIZE, AWVALID, WDATA, WLAST, WVALID, 
		input BREADY,
		input ARADDR, ARLEN, ARSIZE, ARVALID, RREADY,
		input ACLK, 
		input AWREADY, WREADY, BVALID, BRESP,
		input ARREADY, RDATA,  RLAST,  RVALID, RRESP
	);
	
endinterface 