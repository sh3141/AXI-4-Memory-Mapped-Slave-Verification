`timescale 1ns/1ps
`define OKAY   2'b00
`define SLVERR 2'b10

module axi4_sva#(
		parameter int DATA_WIDTH       = 32,
		parameter int MEM_ADDR_WIDTH   = 10,
		parameter int SLAVE_ADDR_WIDTH = 16
	)(
		input logic                        ACLK,
		input logic                        ARESETn,

		// Write address channel
		input logic [SLAVE_ADDR_WIDTH-1:0] AWADDR,
		input logic [7:0]                  AWLEN,
		input logic [2:0]                  AWSIZE,
		input logic                        AWVALID,
		input logic                        AWREADY,

		// Write data channel
		input logic [DATA_WIDTH-1:0]       WDATA,
		input logic                        WVALID,
		input logic                        WLAST,
		input logic                        WREADY,

		// Write response channel
		input logic [1:0]                  BRESP,
		input logic                        BVALID,
		input logic                        BREADY,

		// Read address channel
		input logic [SLAVE_ADDR_WIDTH-1:0] ARADDR,
		input logic [7:0]                  ARLEN,
		input logic [2:0]                  ARSIZE,
		input logic                        ARVALID,
		input logic                        ARREADY,

		// Read data channel
		input logic [DATA_WIDTH-1:0]       RDATA,
		input logic [1:0]                  RRESP,
		input logic                        RVALID,
		input logic                        RLAST,
		input logic                        RREADY
	);
	
	//////              PARAMETERS            //////
	int MEM_BOUNDARY = (1<<(MEM_ADDR_WIDTH+2)) - 1;
	int MEM_DEPTH    = (1<<(MEM_ADDR_WIDTH));
	
	///////////////////////////////////////////////
	/////      CALCULATIONS OF SINGALS        /////
	//////////////////////////////////////////////
	
	///// ------------ Beat number tracking -------------- /////
	logic [31:0] w_beats_no;          // write number of beats
	logic [8:0]  w_expected_beats_no; // write expected number of beats
	logic [31:0] r_beats_no;          // read number of beats
	logic [8:0]  r_expected_beats_no; // read expected number of beats
	logic        wlast_accepted;     //wlast is accepted in write xact
	
	///// ---------- Address boundary crossing checks ----------- /////
	logic        w_exceed_page_bounds;
	logic        r_exceed_page_bounds;
	logic        w_addr_in_bounds;
	logic        r_addr_in_bounds;
	logic [1:0]  expected_bresp;
	logic [1:0]  expected_rresp;
	
	always_ff @(posedge ACLK or negedge ARESETn) begin
		if(!ARESETn) begin
			w_beats_no           <= 0;
			w_expected_beats_no  <= 0;
			r_beats_no           <= 0;
			r_expected_beats_no  <= 0;
			wlast_accepted       <= 0;
			
			expected_bresp       <= `OKAY;
			expected_rresp       <= `OKAY;	
		end
		else begin
			if(AWVALID && AWREADY) begin
				//handshake to start write xact
				w_expected_beats_no  <= AWLEN + 1;
				w_beats_no           <= 0;
				wlast_accepted       <= 0;
				expected_bresp       <= (w_addr_in_bounds && !w_exceed_page_bounds)? `OKAY : `SLVERR;
			end
			else if(BVALID && BREADY) begin
				//write response accepted ending the write xact
				w_expected_beats_no <= 0;
				wlast_accepted      <= 0;
			end
			if(WVALID && WREADY) begin
				//write beat accepted
				w_beats_no          <= ((AWVALID && AWREADY)? 1: w_beats_no + 1);
				if(WLAST) begin
					wlast_accepted <= 1;
				end
			end
			if(ARVALID && ARREADY) begin
				//handshake to start read xact
				r_expected_beats_no  <= ARLEN + 1;
				r_beats_no           <= 0;
				expected_rresp       <= (r_addr_in_bounds && !r_exceed_page_bounds)? `OKAY : `SLVERR;
			end
			if(RVALID && RREADY) begin
				//read beat accepted
				r_beats_no           <= (ARVALID && ARREADY) ? 1: r_beats_no + 1;
				if(!(ARVALID && ARREADY)) begin
					if(RLAST) begin
						r_expected_beats_no <= 0;
					end
				end
			end
		end
	end
	
	always_comb begin
		w_exceed_page_bounds = ((AWADDR & MEM_BOUNDARY) + ((AWLEN)<<AWSIZE)) > MEM_BOUNDARY;
		w_addr_in_bounds     = ((AWADDR>>2) < MEM_DEPTH);
		r_exceed_page_bounds = ((ARADDR & MEM_BOUNDARY) + ((ARLEN)<<ARSIZE)) > MEM_BOUNDARY;
		r_addr_in_bounds     = ((ARADDR>>2) < MEM_DEPTH);
	end
	
	///////////////////////////////////////////////
	/////              ASSERTIONS             /////
	//////////////////////////////////////////////
	
	//----------------- Reset -----------------//
	always_comb begin
		if(!ARESETn) begin
			assert_reset: assert final(BVALID == 0 && RVALID == 0) else $error("[RESETn Assertion Failed], VALID signals didn't fall to LOW");
			
			//assert_reset: assert final(AWREADY == 1 && WREADY == 1 && BVALID == 0 && BRESP == 0 && ARREADY == 1 RDATA == 0 && RLAST == 0 && RVALID == 0 && RRESP == 0);
		end
	end
	
	//----------- Handshakes ----------------//
	property valid_before_ready_p(logic VALID, logic READY);
		@(posedge ACLK) disable iff (!ARESETn) (VALID && !READY) |=> VALID;
	endproperty
	
	assert property (valid_before_ready_p(BVALID, BREADY)) else $error("[ASSERTION FAILED] BVALID was not held until BREADY is asserted");
	assert property (valid_before_ready_p(RVALID, RREADY)) else $error("[ASSERTION FAILED] RVALID was not held until RREADY is asserted");
	
	valid_before_ready_w_a: assert property (valid_before_ready_p(WVALID, WREADY)) else $error("[ASSERTION FAILED] WVALID was not held until WREADY is asserted");
	assert property (valid_before_ready_p(AWVALID, AWREADY)) else $error("[ASSERTION FAILED] AWVALID was not held until AWREADY is asserted");
	assert property (valid_before_ready_p(ARVALID, ARREADY)) else $error("[ASSERTION FAILED] ARVALID was not held until ARREADY is asserted");
	
	
	property data_stable_before_ready_p(logic VALID, logic READY, logic [DATA_WIDTH-1:0] DATA, logic LAST);
		@(posedge ACLK) disable iff (!ARESETn) (VALID && !READY) |=> ($stable(DATA) && $stable(LAST));
	endproperty
	
	assert property (data_stable_before_ready_p(WVALID, WREADY, WDATA, WLAST)) else $error("[ASSERTION FAILED] Write data channel is not held stable before WREADY");
	assert property (data_stable_before_ready_p(RVALID, RREADY, RDATA, RLAST)) else $error("[ASSERTION FAILED] Read data channel is not held stable before RREADY");
	
	property addr_control_stable_before_ready_p(logic VALID, logic READY, logic [SLAVE_ADDR_WIDTH-1:0] ADDR, logic [7:0] LEN, logic [2:0] SIZE);
		@(posedge ACLK) disable iff (!ARESETn) (VALID && !READY) |=> ($stable(ADDR) && $stable(LEN) && $stable(SIZE));
	endproperty
	
	assert property (addr_control_stable_before_ready_p(AWVALID, AWREADY, AWADDR, AWLEN, AWSIZE)) 
		else $error("[ASSERTION FAILED] Write control channel signals are not held stable before AWREADY");
	assert property (addr_control_stable_before_ready_p(ARVALID, ARREADY, ARADDR, ARLEN, ARSIZE)) 
		else $error("[ASSERTION FAILED] Read control channel signals are not held stable before ARREADY");
	
	property resp_stable_before_ready_p(logic VALID, logic READY, logic [1:0] RESP);
		@(posedge ACLK) disable iff (!ARESETn) (VALID && !READY) |=> $stable(RESP);
	endproperty
	
	assert property (resp_stable_before_ready_p(BVALID, BREADY, BRESP)) else $error("[ASSERTION FAILED] Write response channel (BRESP) is not held stable before BREADY");
	
	//------------- Channel dependencies -------------//
	
	property rvalid_dependency_p;
		@(posedge ACLK) disable iff (!ARESETn) 
		(r_expected_beats_no == 0 || (r_expected_beats_no <= r_beats_no)) |-> (!RVALID);
	endproperty
	
	err_rvalid_early_a: assert property (rvalid_dependency_p) 
		else $error("[ASSERTION FAILED] RVALID asserted before Read Address handshake");
		
	property bvalid_dependency_p;
		@(posedge ACLK) disable iff (!ARESETn) 
		(w_expected_beats_no == 0 || ( (!wlast_accepted) && (w_beats_no < w_expected_beats_no) )) |-> (!BVALID);
	endproperty
	
	err_bvalid_early_a: assert property (bvalid_dependency_p) 
		else $error("[ASSERTION FAILED] BVALID asserted before Write Address handshake or completion of write data transfer");
		
	//-------- Correct Last signal assertion --------- //
	property correct_beat_len_p(logic VALID, logic READY, logic LAST, logic [31:0] beats_no, logic [8:0] exp_beats_no);
		@(posedge ACLK) disable iff (!ARESETn) (VALID && READY && LAST) |-> (beats_no == (exp_beats_no - 1));
	endproperty
	
	correct_beat_no_w_a: assert property (correct_beat_len_p(WVALID, WREADY, WLAST, w_beats_no, w_expected_beats_no)) else $error("[ASSERTION FAILED] Incorrect number of beats for write xact");
	assert property (correct_beat_len_p(RVALID, RREADY, RLAST, r_beats_no, r_expected_beats_no)) else $error("[ASSERTION FAILED] Incorrect number of beats for read xact");
	
	property last_remains_low_p(logic VALID, logic READY, logic LAST, logic [31:0] beats_no, logic [8:0] exp_beats_no);
		@(posedge ACLK) disable iff (!ARESETn) (VALID && READY && (beats_no < (exp_beats_no - 1))) |-> (!LAST);
	endproperty

	last_remains_low_w_a: assert property (last_remains_low_p(WVALID, WREADY, WLAST, w_beats_no, w_expected_beats_no)) 
		else $error("[ASSERTION FAILED] LAST arrives early at beat no %0d, instead of beat no %0d for write burst",w_beats_no, w_expected_beats_no);
	last_remains_low_r_a: assert property (last_remains_low_p(RVALID, RREADY, RLAST, r_beats_no, r_expected_beats_no)) 
		else $error("[ASSERTION FAILED] LAST arrives early at beat no %0d, instead of beat no %0d for read burst",r_beats_no, r_expected_beats_no);
	
	property last_at_end_of_burst_p(logic VALID, logic READY, logic LAST, logic [31:0] beats_no, logic [8:0] exp_beats_no);
		@(posedge ACLK) disable iff (!ARESETn) (VALID && READY && (beats_no == (exp_beats_no - 1))) |-> (LAST);
	endproperty
	
	last_at_end_of_burst_w_a: assert property (last_at_end_of_burst_p(WVALID, WREADY, WLAST, w_beats_no, w_expected_beats_no)) 
		else $error("[ASSERTION FAILED] LAST is not asserted at end of write burst");
	last_at_end_of_burst_r_a: assert property (last_at_end_of_burst_p(RVALID, RREADY, RLAST, r_beats_no, r_expected_beats_no)) 
		else $error("[ASSERTION FAILED] LAST is not asserted at end of read burst");
		
	//--------------- Error response ------------------//
	property valid_resp_p(logic VALID, logic [1:0] resp_value, logic [1:0] exp_resp, logic [1:0] RESP);
		@(posedge ACLK) disable iff (!ARESETn) 
		(VALID && (exp_resp == resp_value) )|-> (RESP == resp_value);
	endproperty
	
	read_valid_okay_resp_a: assert property (valid_resp_p(RVALID,`OKAY, expected_rresp, RRESP)) 
		else $error("[ASSERTION FAILED] RRESP does not match expected when expected is OKAY response. Expected RRESP: %2b, Obtained RRESP: %2b",expected_rresp, RRESP);
		
	read_valid_slverr_resp_a: assert property (valid_resp_p(RVALID,`SLVERR, expected_rresp, RRESP)) 
		else $error("[ASSERTION FAILED] RRESP does not match expected when expected is SLVERR response. Expected RRESP: %2b, Obtained RRESP: %2b",expected_rresp, RRESP);
		
	
	write_valid_okay_resp_a: assert property (valid_resp_p(BVALID,`OKAY, expected_bresp, BRESP)) 
		else $error("[ASSERTION FAILED] BRESP does not match expected when expected is OKAY response. Expected BRESP: %2b, Obtained BRESP: %2b",expected_bresp, BRESP);
		
	write_valid_slverr_resp_a: assert property (valid_resp_p(BVALID,`SLVERR, expected_bresp, BRESP)) 
		else $error("[ASSERTION FAILED] BRESP does not match expected when expected is SLVERR response. Expected BRESP: %2b, Obtained BRESP: %2b",expected_bresp, BRESP);
		
	
	///////////////////////////////////////////////
	/////               COVERAGE             /////
	//////////////////////////////////////////////
	
	// ------ complete xact ----------//
	sequence write_xact_complete_s;
		(AWVALID && AWREADY) ##[1:$] (WVALID && WREADY && WLAST) ##[1:$] (BVALID && BREADY);
	endsequence 
	
	property write_xact_complete_p;
		@(posedge ACLK) disable iff(!ARESETn) write_xact_complete_s;
	endproperty 
	
	cover_complete_write_xact: cover property (write_xact_complete_p);
	
	sequence read_xact_complete_s;
		(ARVALID && ARREADY) ##[1:$] (RVALID && RREADY && RLAST);
	endsequence 
	
	property read_xact_complete_p;
		@(posedge ACLK) disable iff(!ARESETn) read_xact_complete_s;
	endproperty 
	
	cover_complete_read_xact: cover property (read_xact_complete_p);
	
	// ------ valid then ready handshake ----------//
	sequence valid_then_ready_s(logic VALID, logic READY);
		(VALID && !READY) ##[1:$] (VALID && READY);
	endsequence 
	
	property valid_then_ready_p(logic VALID, logic READY);
		@(posedge ACLK) disable iff(!ARESETn) valid_then_ready_s(VALID,READY);
	endproperty
	
	cover_BVALID_then_BREADY: cover property (valid_then_ready_p(BVALID, BREADY));
	cover_RVALID_then_RREADY: cover property (valid_then_ready_p(RVALID, RREADY));
	
	// ------ ready then valid handshake ----------//
	sequence ready_then_valid_s(logic VALID, logic READY);
		(!VALID && READY) ##[1:$] (VALID && READY);
	endsequence 
	
	property ready_then_valid_p(logic VALID, logic READY);
		@(posedge ACLK) disable iff(!ARESETn) ready_then_valid_s(VALID,READY);
	endproperty
	
	cover_BREADY_then_BVALID: cover property (ready_then_valid_p(BVALID, BREADY));
	cover_RREADY_then_RVALID: cover property (ready_then_valid_p(RVALID, RREADY));
	
	// ------ handshakes for all 5 channels -------//
	cover_w_addr_channel_handshake: cover property (@(posedge ACLK) (AWVALID && AWREADY));
	cover_w_data_channel_handshake: cover property (@(posedge ACLK) (WVALID && WREADY));
	cover_w_resp_channel_handshake: cover property (@(posedge ACLK) (BVALID && BREADY));
	cover_r_addr_channel_handshake: cover property (@(posedge ACLK) (ARVALID && ARREADY));
	cover_r_data_channel_handshake: cover property (@(posedge ACLK) (RVALID && RREADY));
	
	// ------ single and multi-beat transfers -------//
	cover_w_single_beat_xact: cover property (@(posedge ACLK) (AWVALID && AWREADY && AWLEN == 0));
	cover_w_multi_beat_xact:  cover property (@(posedge ACLK) (AWVALID && AWREADY && AWLEN > 0));
	cover_r_single_beat_xact: cover property (@(posedge ACLK) (ARVALID && ARREADY && ARLEN == 0));
	cover_r_multi_beat_xact:  cover property (@(posedge ACLK) (ARVALID && ARREADY && ARLEN > 0));
		
	//------- hit LAST of transaction -------//
	property hit_end_of_xact_p(logic VALID, logic READY, logic LAST);
		@(posedge ACLK) (VALID && READY && LAST);
	endproperty
	
	cover_w_end_of_xact: cover property (hit_end_of_xact_p(WVALID,WREADY,WLAST));
	cover_r_end_of_xact: cover property (hit_end_of_xact_p(RVALID,RREADY,RLAST));
	
	// ------ inbound and out of bound transfers -------//
	cover_w_page_out_of_bound_cross_addr_in_range : cover property (@(posedge ACLK) (AWVALID && AWREADY && w_addr_in_bounds && w_exceed_page_bounds));
	cover_okay_w_xact                             : cover property (@(posedge ACLK) (AWVALID && AWREADY && w_addr_in_bounds && !w_exceed_page_bounds));
	cover_w_page_inbound_addr_out_range           : cover property (@(posedge ACLK) (AWVALID && AWREADY && !w_addr_in_bounds && !w_exceed_page_bounds));
	cover_w_page_out_of_bound_cross_addr_out_range: cover property (@(posedge ACLK) (AWVALID && AWREADY && !w_addr_in_bounds && w_exceed_page_bounds));
	
	cover_r_page_out_of_bound_cross_addr_in_range : cover property (@(posedge ACLK) (ARVALID && ARREADY && r_addr_in_bounds && r_exceed_page_bounds));
	cover_okay_r_xact                             : cover property (@(posedge ACLK) (ARVALID && ARREADY && r_addr_in_bounds && !r_exceed_page_bounds));
	cover_r_page_inbound_addr_out_range           : cover property (@(posedge ACLK) (ARVALID && ARREADY && !r_addr_in_bounds && !r_exceed_page_bounds));
	cover_r_page_out_of_bound_cross_addr_out_range: cover property (@(posedge ACLK) (ARVALID && ARREADY && !r_addr_in_bounds && r_exceed_page_bounds));
	
	// ------ OKAY and SLVERR responses -------//
	property okay_resp_p(logic VALID, logic READY, logic [1:0] RESP);
		@(posedge ACLK) (VALID && READY && (RESP == `OKAY));
	endproperty 
	
	cover_w_okay_resp: cover property (okay_resp_p(BVALID,BREADY,BRESP));
	cover_r_okay_resp: cover property (okay_resp_p(RVALID,RREADY,RRESP));
	
	property err_resp_p(logic VALID, logic READY, logic [1:0] RESP);
		@(posedge ACLK) (VALID && READY && (RESP == `SLVERR));
	endproperty 
	
	cover_w_err_resp: cover property (err_resp_p(BVALID,BREADY,BRESP));
	cover_r_err_resp: cover property (err_resp_p(RVALID,RREADY,RRESP));
	
endmodule