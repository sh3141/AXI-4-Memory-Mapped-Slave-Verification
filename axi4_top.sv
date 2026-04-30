`timescale 1ns/1ps
module axi4_top();

	//------------------------ PARAMTERS ------------------------//
	int            CLOCK_PERIOD   = 10ns;
	localparam int ADDR_WIDTH_M   = 10;
	localparam int ADDR_WIDTH_S   = 16;
	localparam int DATA_WIDTH     = 32;
	localparam int MEMORY_DEPTH   = (1<<ADDR_WIDTH_M);
	
	
	//-------------------- CLOCK GENERATION ---------------------//
	logic clk = 0;
	always begin
		#(CLOCK_PERIOD/2) clk = ~clk;
	end
	
	//------------------------ INTERFACE ------------------------//
	axi4_if#(.DATA_WIDTH(DATA_WIDTH),
			 .MEM_ADDR_WIDTH(ADDR_WIDTH_M),
			 .SLAVE_ADDR_WIDTH(ADDR_WIDTH_S)
		) axi4if(.ACLK(clk));
	
	//-------------------- DUT INSTANTIATION --------------------//
	axi4 #(.DATA_WIDTH(DATA_WIDTH),
		   .ADDR_WIDTH(ADDR_WIDTH_S),
		   .MEMORY_DEPTH(MEMORY_DEPTH)
	) dut(.ACLK(axi4if.ACLK),
		  .ARESETn(axi4if.ARESETn),

		 // Write address channel
		  .AWADDR(axi4if.AWADDR),
		  .AWLEN(axi4if.AWLEN),
		  .AWSIZE(axi4if.AWSIZE),
		  .AWVALID(axi4if.AWVALID),
          .AWREADY(axi4if.AWREADY),

		// Write data channel
		  .WDATA(axi4if.WDATA),
		  .WVALID(axi4if.WVALID),
	      .WLAST(axi4if.WLAST),
		  .WREADY(axi4if.WREADY),

		// Write response channel
		  .BRESP(axi4if.BRESP),
		  .BVALID(axi4if.BVALID),
		  .BREADY(axi4if.BREADY),

		// Read address channel
		  .ARADDR(axi4if.ARADDR),
		  .ARLEN(axi4if.ARLEN),
		  .ARSIZE(axi4if.ARSIZE),
		  .ARVALID(axi4if.ARVALID),
		  .ARREADY(axi4if.ARREADY),

		// Read data channel
		  .RDATA(axi4if.RDATA),
		  .RRESP(axi4if.RRESP),
		  .RVALID(axi4if.RVALID),
		  .RLAST(axi4if.RLAST),
		  .RREADY(axi4if.RREADY)
		);
	
	//----------------- TESTBENCH INSTANTIATION -----------------//
	axi4_tb tb(axi4if.master);
	
	//----------------------- ASSERTIONS ------------------------//
	bind axi4 axi4_sva axi4_sva_bind(
		  .ACLK(axi4.ACLK),
		  .ARESETn(axi4.ARESETn),

		 // Write address channel
		  .AWADDR(axi4.AWADDR),
		  .AWLEN(axi4.AWLEN),
		  .AWSIZE(axi4.AWSIZE),
		  .AWVALID(axi4.AWVALID),
          .AWREADY(axi4.AWREADY),

		// Write data channel
		  .WDATA(axi4.WDATA),
		  .WVALID(axi4.WVALID),
	      .WLAST(axi4.WLAST),
		  .WREADY(axi4.WREADY),

		// Write response channel
		  .BRESP(axi4.BRESP),
		  .BVALID(axi4.BVALID),
		  .BREADY(axi4.BREADY),

		// Read address channel
		  .ARADDR(axi4.ARADDR),
		  .ARLEN(axi4.ARLEN),
		  .ARSIZE(axi4.ARSIZE),
		  .ARVALID(axi4.ARVALID),
		  .ARREADY(axi4.ARREADY),

		// Read data channel
		  .RDATA(axi4.RDATA),
		  .RRESP(axi4.RRESP),
		  .RVALID(axi4.RVALID),
		  .RLAST(axi4.RLAST),
		  .RREADY(axi4.RREADY)
	);
				
endmodule