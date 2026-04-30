`define OKAY   2'b0
`define SLVERR 2'b10
`timescale 1ns/1ps
`include "Axi4_packet.sv"

module axi4_tb(axi4_if.master axi4_if);
	//------------------------ PARAMTERS ------------------------//
	int MEM_DEPTH                = (1<<(axi4_if.MEM_ADDR_WIDTH));
	int MAX_AW_DELAY             = 3;
	int MAX_WDATA_DELAY          = 3;
	int MAX_BREADY_DELAY         = 5;
	
	int MAX_AR_DELAY             = 3;
	int MAX_RREADY_DELAY         = 5;
	
	int VALID_BEFORE_READY_RATIO = 3;
	int MAX_WLAST_DELAY          = 8;
	int MAX_BYTE_ADDRESS         = (1<<(axi4_if.MEM_ADDR_WIDTH+2))-1;
	
	int ADDITIONAL_ADDR_CHECK_R  = 20;
	int ADDITIONAL_ADDR_CHECK_W  = 50;
	
	int BVALID_TIMEOUT           = 1000;
	int WREADY_TIMEOUT           = 1000;
	int DRIVER_POST_RESET_MARGIN = 20; // number of cycles given to driver after reset is asserted. 
	
	//------------------ STIMULUS DECLARATIONS ------------------//
	int                                   beats_count;
	logic [axi4_if.SLAVE_ADDR_WIDTH- 1:0] start_addr;
	logic [axi4_if.SLAVE_ADDR_WIDTH- 1:0] last_addr;
	Axi4_packet#(.MEM_ADDR_WIDTH(axi4_if.MEM_ADDR_WIDTH),
				 .DATA_WIDTH(axi4_if.DATA_WIDTH),
				 .SLAVE_ADDR_WIDTH(axi4_if.SLAVE_ADDR_WIDTH)
				) pkt;
				
	Axi4_packet#(.MEM_ADDR_WIDTH(axi4_if.MEM_ADDR_WIDTH),
				 .DATA_WIDTH(axi4_if.DATA_WIDTH),
				 .SLAVE_ADDR_WIDTH(axi4_if.SLAVE_ADDR_WIDTH)
				) pkt2;
	//---------------- OUTPUT DATA DECLARATIONS -----------------//
	typedef struct packed{
		logic [axi4_if.DATA_WIDTH - 1:0] data; 
		logic [1:0]                      resp;               
	} read_data_channel_t;
	
	read_data_channel_t              actual_data_queue[$];
	logic [axi4_if.DATA_WIDTH - 1:0] mem_data;
	logic [1:0]                      sampled_resp;
	
	//--------------------- GOLDEN MODEL ------------------------//
	logic [axi4_if.DATA_WIDTH - 1:0]     expected_mem[(1<<axi4_if.MEM_ADDR_WIDTH) - 1:0];
	logic [1:0]                          exp_RESP;
	logic [axi4_if.DATA_WIDTH - 1:0]     exp_data_queue[$];
	int                                  extra_addr_queue[$];
	logic [axi4_if.DATA_WIDTH - 1:0]     mem_snapshot[logic [axi4_if.MEM_ADDR_WIDTH - 1:0]]; //check for no memory corruption
	
	//------------------------ TEST INFO ------------------------//
	int config_no        = 0;
	int tests_passed     = 0;
	int tests_failed     = 0;
	int tests_invalid    = 0;
	int total_tests      = 100;
	
	int tests_passed_r   = 0;
	int tests_failed_r   = 0;
	int tests_invalid_r  = 0;
	int total_tests_r    = 0;
	
	int tests_passed_w   = 0;
	int tests_failed_w   = 0;
	int tests_invalid_w  = 0;
	int total_tests_w    = 0;
	
	bit xact_passed;
	int read_delays;
	int write_delays;
	bit same_pkt;
	int reset_beat;
	int accepted_data_beats;
	
	//------ WLAST diagnostic info ----
	int accepted_on_early_WLAST = 0;
	int accepted_on_late_WLAST  = 0;
	int accepted_on_early_WLEN  = 0;
	int accepted_on_late_WLEN   = 0;
	int accepted_on_other       = 0;
	int failed_to_terminate     = 0;
	
	int valid_writes            = 0;
	int write_on_early_WLAST    = 0;
	int write_on_late_WLAST     = 0;
	int write_on_early_WLEN     = 0;
	int write_on_late_WLEN      = 0;
	int write_on_other          = 0;
	wlast_err_e wlast_err;
	
	//-------- R/W Race info --------
	int write_first             = 0;
	int read_first              = 0;
	int no_rw_order_rule        = 0;
	int rw_race_tests           = 0;
	int corrupt_read            = 0;
	
	//--------------------- RUNNING TESTCASES -------------------//
	string verb;   //configure details printed from test bench
	string tb_state;
	int show_per_test;
	bit pkt_state;
	
	initial begin
		if(! $value$plusargs("verbosity=%s", verb)) begin
			verb = "detailed";
		end
		show_per_test = verb_value(verb,tb_state);
		pkt = new();
		$display("-----------------------------------------------------------------------------------");
		$display("--------------------- AXI4 MEMORY-MAPPED SLAVE TESTBENCH (%s) ---------------------",tb_state);
		$display("-----------------------------------------------------------------------------------");	
		
		// clearing channels and asserting Reset
		init();
		reset();
		@(negedge axi4_if.ACLK);
		
		// direct memory data scramble 
		direct_mem_write();
	
		$display("-----------------------Running simple preliminary tests ------------------------");
		
		// ------------------------------------ Single write test ------------------------------------//
		
		$display("----------------------- Single write pretest------------------------");
		total_tests = 1;
		pkt.write_en = WRITE;
		pkt.addr = 16'h0100;		
		pkt.len  = 0; 
		pkt.size = 3'd2;
		pkt.burst_bounds = WITHIN_BOUNDS;
	
		pkt.burst_data = new[1];
		pkt.burst_data[0] = 32'hAAAA_AAAA;
		pkt.data_pattern = DATA_CHECKERBOARD;
		
		run_tests(pkt,config_no);
		print_summary(config_no);
		clear_test_info();
		// ------------------------------------ Single read test ------------------------------------//
		
		$display("----------------------- Single read pretest ------------------------");
		total_tests = 1;
		pkt = new();
		pkt.write_en = READ;
		pkt.addr = 16'h0100;		
		pkt.len  = 0; 
		pkt.size = 3'd2;
		pkt.burst_bounds = WITHIN_BOUNDS;
	
		pkt.burst_data = new[1];
		pkt.burst_data[0] = 32'h5555_5555;
		pkt.data_pattern = DATA_CHECKERBOARD;
		
		run_tests(pkt,config_no);
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple within bound single-beat write test ------------------------------------//
		
		$display("----------------------- Multiple within bound single-beat write tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.inbound_burst_c.constraint_mode(1);
		pkt.single_beat_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ On-boundary single-beat write test ------------------------------------//
		
		$display("----------------------- On-boundary single-beat write tests ------------------------");
		total_tests = 1;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.on_boundary_burst_c.constraint_mode(1);
		pkt.single_beat_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple out of bound single-beat write test ------------------------------------//
		
		$display("----------------------- Multiple out of bound single-beat write tests ------------------------");
		total_tests = 25;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.single_beat_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		pkt.out_of_bound_addr_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple within bound single-beat read test ------------------------------------//
		
		$display("----------------------- Multiple within bound single-beat read tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.inbound_burst_c.constraint_mode(1);
		pkt.single_beat_c.constraint_mode(1);
		pkt.read_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ On-boundary single-beat read test ------------------------------------//
		
		$display("----------------------- On-boundary single-beat read tests ------------------------");
		total_tests = 1;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.on_boundary_burst_c.constraint_mode(1);
		pkt.single_beat_c.constraint_mode(1);
		pkt.read_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple out of bound single-beat read test ------------------------------------//
		
		$display("----------------------- Multiple out of bound single-beat read tests ------------------------");
		total_tests = 25;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.single_beat_c.constraint_mode(1);
		pkt.read_xact_c.constraint_mode(1);
		pkt.out_of_bound_addr_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		
		// ------------------------------------ Multiple within bound and address multi-beat write test ------------------------------------//
		
		$display("----------------------- Multiple within bound and address multi-beat write tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.inbound_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple within bound and address violation multi-beat write test ------------------------------------//
		
		$display("----------------------- Multiple within bound but starting out of bound address multi-beat write tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.inbound_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		pkt.out_of_bound_addr_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple within bound write test ------------------------------------//
		
		$display("----------------------- Multiple within bound write tests ------------------------");
		total_tests = 100;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.inbound_burst_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple on boundary burst and address within range multi-beat write test ------------------------------------//
		
		$display("----------------------- Multiple on boundary and address within range multi-beat write tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.on_boundary_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple on boundary and address violation multi-beat write test ------------------------------------//
		
		$display("----------------------- Multiple on boundary but starting out of bound address multi-beat write tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.on_boundary_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		pkt.out_of_bound_addr_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple on-boundary burst write test ------------------------------------//
		
		$display("----------------------- Multiple on-boundary burst write tests ------------------------");
		total_tests = 100;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.on_boundary_burst_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple out of bound burst burst and address within range multi-beat write test ------------------------------------//
		
		$display("----------------------- Multiple out of bound burst and address within range multi-beat write tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.out_of_bound_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple out of bound burst and address violation multi-beat write test ------------------------------------//
		
		$display("----------------------- Multiple out of bound burst and starting out of bound address multi-beat write tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.out_of_bound_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		pkt.out_of_bound_addr_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple out of bound burst write test ------------------------------------//
		
		$display("----------------------- Multiple out of bound burst write tests ------------------------");
		total_tests = 100;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.out_of_bound_burst_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple burst write test ------------------------------------//
		
		$display("----------------------- Multiple burst write tests ------------------------");
		total_tests = 250;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.write_xact_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple within bound and address multi-beat read test ------------------------------------//
		
		$display("----------------------- Multiple within bound and address multi-beat read tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.inbound_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.read_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple within bound and address violation multi-beat read test ------------------------------------//
		
		$display("----------------------- Multiple within bound but starting out of bound address multi-beat read tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.inbound_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.read_xact_c.constraint_mode(1);
		pkt.out_of_bound_addr_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple within bound read test ------------------------------------//
		
		$display("----------------------- Multiple within bound read tests ------------------------");
		total_tests = 100;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.inbound_burst_c.constraint_mode(1);
		pkt.read_xact_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple on boundary burst and address within range multi-beat read test ------------------------------------//
		
		$display("----------------------- Multiple on boundary and address within range multi-beat read tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.on_boundary_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.read_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple on boundary and address violation multi-beat read test ------------------------------------//
		
		$display("----------------------- Multiple on boundary but starting out of bound address multi-beat read tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.on_boundary_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.read_xact_c.constraint_mode(1);
		pkt.out_of_bound_addr_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple on-boundary burst read test ------------------------------------//
		
		$display("----------------------- Multiple on-boundary burst read tests ------------------------");
		total_tests = 100;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.on_boundary_burst_c.constraint_mode(1);
		pkt.read_xact_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple out of bound burst burst and address within range multi-beat read test ------------------------------------//
		
		$display("----------------------- Multiple out of bound burst and address within range multi-beat read tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.out_of_bound_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.read_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple out of bound burst and address violation multi-beat read test ------------------------------------//
		
		$display("----------------------- Multiple out of bound burst and starting out of bound address multi-beat read tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.out_of_bound_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.read_xact_c.constraint_mode(1);
		pkt.out_of_bound_addr_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple out of bound burst read test ------------------------------------//
		
		$display("----------------------- Multiple out of bound burst read tests ------------------------");
		total_tests = 100;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.out_of_bound_burst_c.constraint_mode(1);
		pkt.read_xact_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Multiple burst read test ------------------------------------//
		
		$display("----------------------- Multiple burst read tests ------------------------");
		total_tests = 250;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.read_xact_c.constraint_mode(1);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		
		// ------------------------------------ Random burst tests ------------------------------------//
		
		$display("----------------------- Random burst tests ------------------------");
		total_tests = 500;
		pkt = new();
		disable_conf_constraints(pkt);
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			run_tests(pkt,config_no);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Back to Back write xacts ------------------------------------//
		
		$display("----------------------- Back to back write xacts ------------------------");
		total_tests = 200;
		same_pkt = 0;
		repeat(total_tests/2) begin
			exp_data_queue.delete();
			actual_data_queue.delete();
			back_to_back_write_xact(same_pkt);
		end
		print_summary(config_no);
		clear_test_info();
		// ------------------------------------ Back to Back write xacts with overwriting ------------------------------------//
		
		$display("----------------------- Back to back write xacts with over-writing ------------------------");
		total_tests = 50;
		same_pkt = 1;
		repeat(total_tests/2) begin
			exp_data_queue.delete();
			actual_data_queue.delete();
			back_to_back_write_xact(same_pkt);
		end
		
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ Back to Back read xacts ------------------------------------//
		
		$display("----------------------- Back to back read xacts ------------------------");
		total_tests = 250;

		repeat(total_tests/2) begin
			exp_data_queue.delete();
			actual_data_queue.delete();
			back_to_back_read_xact();
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ W_ADDR -> W_IDLE tests ------------------------------------//
		
		$display("----------------------- Mid-flight reset tests in write Address phase ------------------------");
		total_tests = 10;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.write_xact_c.constraint_mode(1);
		
		repeat(total_tests) begin
			exp_data_queue.delete();
			actual_data_queue.delete();
			generate_stimulus(pkt);
			reset_midst_address(pkt);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ R_ADDR -> R_IDLE tests ------------------------------------//
		
		$display("----------------------- Mid-flight reset tests in read Address phase ------------------------");
		total_tests = 10;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.read_xact_c.constraint_mode(1);
		
		repeat(total_tests) begin
			exp_data_queue.delete();
			actual_data_queue.delete();
			generate_stimulus(pkt);
			reset_midst_address(pkt);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ W_DATA -> W_IDLE tests ------------------------------------//
		MAX_WDATA_DELAY = 0;
		$display("----------------------- Mid-flight reset tests in write data phase ------------------------");
		total_tests = 100;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.write_xact_c.constraint_mode(1);
		
		repeat(total_tests/2) begin
			exp_data_queue.delete();
			actual_data_queue.delete();
			generate_stimulus(pkt);
			reset_beat = $urandom_range(0,pkt.len);
			reset_midst_data(pkt,reset_beat);
		end
		print_summary(config_no);
		clear_test_info();
		
		
		// ------------------------------------ W_DATA -> W_IDLE tests ------------------------------------//
		
		$display("----------------------- Mid-flight reset tests in write data or response phase ------------------------");
		total_tests = 100;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.write_xact_c.constraint_mode(1);
		
		repeat(total_tests/2) begin
			exp_data_queue.delete();
			actual_data_queue.delete();
			generate_stimulus(pkt);
			reset_beat = $urandom_range(0,pkt.len + 1);
			reset_midst_data(pkt,reset_beat);
		end
		print_summary(config_no);
		clear_test_info();
		
		// ------------------------------------ W_DATA -> W_IDLE on last beat stress-tests ------------------------------------//
		
		$display("----------------------- Reset assertion in last beat in write data phase ------------------------");
		total_tests = 40;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.write_xact_c.constraint_mode(1);
		
		repeat(total_tests/2) begin
			exp_data_queue.delete();
			actual_data_queue.delete();
			generate_stimulus(pkt);
			reset_beat = pkt.len + 1;
			reset_midst_data(pkt,reset_beat);
		end
		print_summary(config_no);
		clear_test_info();
		
		MAX_WDATA_DELAY = 3;
		// ------------------------------------ R_DATA -> R_IDLE tests ------------------------------------//
		VALID_BEFORE_READY_RATIO = 0;
		MAX_RREADY_DELAY         = 0;
		$display("----------------------- Mid-flight reset tests in read data phase ------------------------");
		total_tests = 100;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.read_xact_c.constraint_mode(1);
		
		repeat(total_tests/2) begin
			exp_data_queue.delete();
			actual_data_queue.delete();
			generate_stimulus(pkt);
			reset_beat = $urandom_range(0,pkt.len);
			reset_midst_data(pkt,reset_beat);
		end
		print_summary(config_no);
		clear_test_info();
		
		VALID_BEFORE_READY_RATIO = 3;
		MAX_RREADY_DELAY         = 5;
		
		$display("-------------------------------------------------------------------------------------");
		$display("------------------------ INITIATING NEGATIVE TESTING SUITE --------------------------");
		$display("------------------ Disabling WLAST protocol assertions to prevent log spam ----------");
		$display("-------------------------------------------------------------------------------------");
		
		$assertoff(0,axi4_top.dut.axi4_sva_bind.correct_beat_no_w_a);
		$assertoff(0,axi4_top.dut.axi4_sva_bind.last_remains_low_w_a);
		$assertoff(0,axi4_top.dut.axi4_sva_bind.last_at_end_of_burst_w_a);
		$assertoff(0,axi4_top.dut.axi4_sva_bind.valid_before_ready_w_a);
		
		
		// ------------------------------------ Early WLAST tests ------------------------------------//
		
		$display("----------------------- Early WLAST multi-beat within bounds write tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.inbound_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		wlast_err = EARLY_WLAST;
		extra_addr_queue.delete();
		mem_snapshot.delete();
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			pkt.addr = $urandom_range(0, 'd200);
			wlast_assertion_violation(pkt,wlast_err);
		end
		print_wlast_tests_results(config_no);
		clear_test_info();
		
		// ------------------------------------ Late WLAST tests ------------------------------------//
		
		$display("----------------------- Late WLAST multi-beat within bounds write tests ------------------------");
		total_tests = 50;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.inbound_burst_c.constraint_mode(1);
		pkt.multi_beat_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		wlast_err = DELAYED_WLAST;
		
		repeat(total_tests) begin
			generate_stimulus(pkt);
			pkt.addr = $urandom_range(0, 'd200);
			wlast_assertion_violation(pkt,wlast_err);
		end
		print_wlast_tests_results(config_no);
		clear_test_info();
		
		
		$display("-------------------------------------------------------------------------------------");
		$display("------------------------ ENDING NEGATIVE TESTING SUITE ------------------------------");
		$display("------------------ Enabling WLAST protocol assertions back again --------------------");
		$display("-------------------------------------------------------------------------------------");
		
		$asserton(0,axi4_top.dut.axi4_sva_bind.correct_beat_no_w_a);
		$asserton(0,axi4_top.dut.axi4_sva_bind.last_remains_low_w_a);
		$asserton(0,axi4_top.dut.axi4_sva_bind.last_at_end_of_burst_w_a);
		$asserton(0,axi4_top.dut.axi4_sva_bind.valid_before_ready_w_a);
		
		// ------------------------------------ Simultaneous R/W xacts ------------------------------------//
		MAX_AW_DELAY             = 0;
		MAX_WDATA_DELAY          = 0;
		MAX_BREADY_DELAY         = 0;
	
		MAX_AR_DELAY             = 0;
		MAX_RREADY_DELAY         = 0;
	
		VALID_BEFORE_READY_RATIO = 0;
		$display("-------------------------------- Simultaneous R/W xacts --------------------------------");
		total_tests = 5;
		pkt = new();
		disable_conf_constraints(pkt);
		pkt.write_xact_c.constraint_mode(1);
		
		pkt2 = new();
		disable_conf_constraints(pkt2);
		pkt2.read_xact_c.constraint_mode(1);
		
		extra_addr_queue.delete();
		mem_snapshot.delete();
		
		repeat(total_tests) begin
			actual_data_queue.delete();
			exp_data_queue.delete();
			generate_stimulus(pkt);
			generate_stimulus(pkt2);
			parallel_read_write_xact(pkt2,pkt);
		end
		print_summary(config_no);
		print_rw_race_tests_results(config_no);
		clear_test_info();
		
		// ------------------------------------ Simultaneous R/W xacts at the same address ------------------------------------//
		
		$display("----------------------- Simultaneous R/W xacts at the same address ------------------------");
		total_tests = 10;
		pkt = new();
		pkt2 = new();
		
		disable_conf_constraints(pkt);
		pkt.inbound_burst_c.constraint_mode(1);
		pkt.write_xact_c.constraint_mode(1);
		pkt.addr_inbound_c.constraint_mode(1);
		
		disable_conf_constraints(pkt2);
		pkt2.inbound_burst_c.constraint_mode(1);
		pkt2.read_xact_c.constraint_mode(1);
		pkt2.addr_inbound_c.constraint_mode(1);
		
		repeat(total_tests) begin
			actual_data_queue.delete();
			exp_data_queue.delete();
			generate_stimulus(pkt);
			generate_stimulus(pkt2);
			pkt.addr = $urandom_range(0, 'd200);
			pkt2.addr = pkt.addr;
			parallel_read_write_xact(pkt2,pkt);
		end
		print_summary(config_no);
		print_rw_race_tests_results(config_no);
		clear_test_info();
		
		MAX_AW_DELAY             = 3;
		MAX_WDATA_DELAY          = 3;
		MAX_BREADY_DELAY         = 5;
	
		MAX_AR_DELAY             = 3;
		MAX_RREADY_DELAY         = 5;
	
		VALID_BEFORE_READY_RATIO = 3;
		
		// ------------------------------------ End of test bench ------------------------------------//
		#150ns;
		$stop();
	end
	
	
	
	//-----------------------------------------------------------//
	//-------------------- TASKS AND FUNCTIONS ------------------//
	//-----------------------------------------------------------//
	

	//---------------- Generating random stimulus ---------------//
	task automatic generate_stimulus(ref Axi4_packet pkt);
		assert(pkt.randomize()) else $fatal("Stimulus randomization failed");
		if(show_per_test == 0 || show_per_test ==2) begin
			pkt.display();
		end
	endtask
	
	//----------------------- Driving dut -----------------------//
	task automatic drive_stim(ref Axi4_packet pkt, ref logic[1:0] sampled_resp);	
		
		int i;
		bit read_terminated;
		read_data_channel_t rd_data;
		read_terminated = 0;
		
		if(pkt.write_en == WRITE) begin
			//------------- WRITE ADDRESS CHANNEL --------------
			axi4_if.AWVALID = 1'b0; //master stall 
			write_delays = $urandom_range(0, MAX_AW_DELAY);
			repeat (write_delays) @(negedge axi4_if.ACLK);
			axi4_if.AWVALID = 1'b1;
			axi4_if.AWADDR  = pkt.addr;
			axi4_if.AWLEN   = pkt.len;
			axi4_if.AWSIZE  = pkt.size;
			
			do @(posedge axi4_if.ACLK); while (!axi4_if.AWREADY); //wait for ready to complete handshake
			if(show_per_test != 1) begin
				$display("[WRITE ADDRESS CHANNEL] Handshake done at time: %0t",$time());
			end
			@(negedge axi4_if.ACLK);
			axi4_if.AWVALID = 1'b0;
			
			//------------ WRITE DATA -------------------------
			if(show_per_test != 1) begin
				$display("Start sending burst write data at time: %0t",$time());
			end
			for(i = 0;i<(pkt.len+1);i++) begin
				axi4_if.WVALID = 1'b0;
				repeat ($urandom_range(0, MAX_WDATA_DELAY))  @(negedge axi4_if.ACLK);
				axi4_if.WVALID = 1'b1;
				axi4_if.WDATA  = pkt.burst_data[i];
				axi4_if.WLAST  = (i == (pkt.len));
				do @(posedge axi4_if.ACLK); while (!axi4_if.WREADY);
				if(show_per_test != 1) begin
					$display("[WRITE DATA CHANNEL] Handshake done for beat number: %0d WDATA: %h, WLAST: %b, @ time: %0t",i, axi4_if.WDATA, axi4_if.WLAST,$time());
				end
			end
			@(negedge axi4_if.ACLK);
			axi4_if.WVALID = 1'b0;
			axi4_if.WLAST  = 1'b0;
			
			// ---------------WRITE RESPONSE CHANNEL ------------
			axi4_if.BREADY = 1'b0; //stall assertion of BREADY to ensure slave holds BVALID and BRESP until BREADY is asserted
			if($urandom_range(0,VALID_BEFORE_READY_RATIO) == VALID_BEFORE_READY_RATIO) begin
				repeat ($urandom_range(0, MAX_BREADY_DELAY))  @(negedge axi4_if.ACLK);
				axi4_if.BREADY = 1'b1;
				if(show_per_test != 1) begin
					$display("BREADY asserted at time: %0t",$time());		
				end
				do @(posedge axi4_if.ACLK); while (!axi4_if.BVALID);
			end
			else begin
				do @(posedge axi4_if.ACLK); while (!axi4_if.BVALID);
				repeat ($urandom_range(1, MAX_BREADY_DELAY))  @(negedge axi4_if.ACLK);
				axi4_if.BREADY = 1'b1;
				@(posedge axi4_if.ACLK);
				if(show_per_test != 1) begin
					$display("BREADY asserted at time: %0t",$time());		
				end

				if(show_per_test != 1 && !axi4_if.BVALID) begin
					$display("[HANDSHAKE VIOLATION] BVALID got asserted and deasserted while BREADY was low");
					do @(posedge axi4_if.ACLK); while (!axi4_if.BVALID);
				end
			end
			
			if(show_per_test != 1) begin
				$display("[WRITE RESPONSE CHANNEL] Handshake done at time: %0t",$time());
			end
			sampled_resp = axi4_if.BRESP;
			if(show_per_test != 1) begin
				if(sampled_resp == `OKAY) begin
					$display("WRITE RESPONSE = %2b, (OKAY) at time: %0t",sampled_resp,$time());
				end
				else if(sampled_resp == `SLVERR) begin
					$display("WRITE RESPONSE = %2b, (SLVERR) at time: %0t",sampled_resp,$time());
				end
				else begin
					$display("WRITE RESPONSE = %2b, at time: %0t",sampled_resp,$time());
				end
			end
			@(negedge axi4_if.ACLK);
			axi4_if.BREADY = 1'b0;
			if(show_per_test != 1) begin
				$display("[WRITE XACT COMPLETE] @ time: %0t, Address: %h, Burst Length: %0d, Response: %2b",$time(),pkt.addr, pkt.len + 1,axi4_if.BRESP);
				$display("---------------------------------------------------------------------------------");
			end
			pkt.burst_sample();
		end
		else begin
			//------------ READ ADDRESS CHANNEL -----------------
			axi4_if.ARVALID = 1'b0; //master stall 
			read_delays = $urandom_range(0, MAX_AR_DELAY);
			repeat (read_delays) @(negedge axi4_if.ACLK);
			axi4_if.ARVALID = 1'b1;
			axi4_if.ARADDR  = pkt.addr;
			axi4_if.ARLEN   = pkt.len;
			axi4_if.ARSIZE  = pkt.size;
			
			do @(posedge axi4_if.ACLK); while (!axi4_if.ARREADY); //wait for ready to complete handshake
			if(show_per_test != 1) begin
				$display("[READ ADDRESS CHANNEL] Handshake done at time: %0t",$time());
			end
			@(negedge axi4_if.ACLK);
			axi4_if.ARVALID = 1'b0;
		
			// ----------- READ DATA -------------------------
			for(i = 0;i<(pkt.len + 1);i++) begin
				axi4_if.RREADY = 1'b0;
				if($urandom_range(0,VALID_BEFORE_READY_RATIO) == VALID_BEFORE_READY_RATIO) begin
					// do READY first then VALID handshake
					repeat ($urandom_range(0, MAX_RREADY_DELAY)) @(negedge axi4_if.ACLK);
					axi4_if.RREADY = 1'b1;
					if(show_per_test != 1) begin
						$display("RREADY asserted at time: %0t",$time());		
					end
					do @(posedge axi4_if.ACLK); while (!axi4_if.RVALID);
				end
				else begin
					// do VALID first then READY handshake
					do @(posedge axi4_if.ACLK); while (!axi4_if.RVALID);
					repeat ($urandom_range(1, MAX_RREADY_DELAY)) @(negedge axi4_if.ACLK);
					axi4_if.RREADY = 1'b1;
					@(posedge axi4_if.ACLK);
					if(show_per_test != 1) begin
						$display("RREADY asserted at time: %0t",$time());		
					end
					if(show_per_test != 1 && !axi4_if.RVALID) begin
						$display("[HANDSHAKE VIOLATION] RVALID got asserted and deasserted while RREADY was low");
						do @(posedge axi4_if.ACLK); while (!axi4_if.RVALID);
					end
				end
				if(show_per_test != 1) begin
					$display("[READ DATA CHANNEL] Handshake done at time: %0t",$time());
				end
				rd_data = '{data: axi4_if.RDATA,
							resp: axi4_if.RRESP
							};
				actual_data_queue.push_back(rd_data);
				if(show_per_test != 1) begin
					$display("[READ DATA] @ time: %0t, Address: %h, Beat number: %0d, Data: %h, RLAST: %b",$time(), pkt.addr,i, axi4_if.RDATA, axi4_if.RLAST);
				end
					
				if (axi4_if.RLAST) begin
					if(show_per_test != 1) begin
						read_terminated = 1;
						$display("[READ XACT TERMINATED] @ time: %0t, Address: %h, Burst Length: %0d",$time(),pkt.addr, pkt.len + 1);
						$display("---------------------------------------------------------------------------------");
					end
					break;
				end
			end
			@(negedge axi4_if.ACLK);
            axi4_if.RREADY = 0;
			if(show_per_test != 1) begin
				foreach(actual_data_queue[i]) begin
					$display("RDATA[%0d] = %h", i, actual_data_queue[i].data);
				end
			end
			
			if(show_per_test != 1 && !read_terminated) begin
				$display("[READ XACT COMPLETE] @ time: %0t, Address: %h, Burst Length: %0d",$time(),pkt.addr, pkt.len + 1);
				$display("---------------------------------------------------------------------------------");
			end
		end
		pkt.axi4_cov.sample();
	endtask

	//---------------------- Golden model -----------------------//
	function automatic void golden_model(ref Axi4_packet pkt, ref logic [1:0] exp_RESP);
		int burst_len;
		int base_addr;
		int last_burst_addr; 
		int mem_word_addr;
		int extra_addr;
		int addr_lb; //adder lower bound
		int addr_up; //adder upper bound
		int start_word_addr;
		int end_word_addr;
		int temp_addrs[];
		
		burst_len       = pkt.len + 1;
		base_addr       = (pkt.addr&MAX_BYTE_ADDRESS);
		last_burst_addr = base_addr + ((pkt.len)<<pkt.size);
		
		exp_RESP = `OKAY;
		
		//---- Generate random addresses out of operations bounds to ensure they are not affected by xact ----
		
		//random addresses
		extra_addr = (pkt.write_en == WRITE)? ADDITIONAL_ADDR_CHECK_W:ADDITIONAL_ADDR_CHECK_R;
		temp_addrs = new[extra_addr];
		
		start_word_addr = ((base_addr>>2) - 1)&(MEM_DEPTH - 1);
		end_word_addr   = ((base_addr>>2) + burst_len)&(MEM_DEPTH - 1);
		
		if (!std::randomize(temp_addrs) with {
			unique {temp_addrs}; 
			
			foreach(temp_addrs[i]) {
				temp_addrs[i] inside {[0:MEM_DEPTH - 1]};   
				if(start_word_addr <= end_word_addr){
					!(temp_addrs[i] inside {[start_word_addr:end_word_addr]}); // outside xact zone
				}
				else{
					(temp_addrs[i] inside {[end_word_addr + 1: start_word_addr - 1]}); // outside xact zone
				}
				
			}
		}) begin
			$error("[ADDITIONAL RANDOM ADDRESS GENERATOR] Failed to find unique addresses");
		end
		
		extra_addr_queue = temp_addrs; 
		//off by one addresses 
		mem_word_addr = ( (base_addr>>2) - 1)&(MEM_DEPTH - 1);
		extra_addr_queue.push_back(mem_word_addr);
		
		mem_word_addr = ((base_addr>>2) + burst_len)&(MEM_DEPTH - 1);
		extra_addr_queue.push_back(mem_word_addr);
		
		foreach(extra_addr_queue[i]) begin //capture memory values of addresses that are out of xact range
			mem_snapshot[extra_addr_queue[i]] = axi4_top.dut.mem_inst.memory[extra_addr_queue[i]];
		end
		
		//--------------- BOUNDARY CHECK ---------------------
		if(last_burst_addr > MAX_BYTE_ADDRESS || (pkt.addr > MAX_BYTE_ADDRESS)) begin
			exp_RESP = `SLVERR;
			// ------------ READ --------------
			if(pkt.write_en == READ) begin
				for(int i = 0; i<burst_len;i++) begin			
					int mem_addr = ((base_addr>>2) + i)&(MEM_DEPTH - 1);
					exp_data_queue.push_back(0);
					extra_addr_queue.push_back(mem_addr);
					mem_snapshot[mem_addr] = axi4_top.dut.mem_inst.memory[mem_addr];
				end
			end
			else begin
				for(int i = 0; i< burst_len ;i++) begin
					int mem_addr = ((base_addr>>2) + i)&(MEM_DEPTH - 1);
					extra_addr_queue.push_back(mem_addr);
					mem_snapshot[mem_addr] = axi4_top.dut.mem_inst.memory[mem_addr];
				end
			end
			
			return ;
		end
		if(pkt.write_en == READ) begin
			// ------------ READ --------------
			for(int i = 0; i<burst_len;i++) begin
				int mem_addr = (base_addr>>2) + i;
				mem_data = axi4_top.dut.mem_inst.memory[mem_addr];
				exp_data_queue.push_back(mem_data);
			end
		end
		else begin
			// ------------ WRITE -------------
			for(int i = 0; i<burst_len;i++) begin
				int mem_addr = (base_addr>>2) + i;
				expected_mem[mem_addr] = pkt.burst_data[i];
				//$display("calculated address: %0d, burst_data[%0d] = %h",mem_addr, i, pkt.burst_data[i]);
				//$display("expected_mem[%0d]= %h",mem_addr,expected_mem[mem_addr]);
			end
		end
	endfunction
	
	//-------------------- Checking results ---------------------//
	task automatic check_results(ref Axi4_packet pkt,ref logic[1:0] exp_RESP, ref logic [1:0] sampled_resp);
		int first_data_mismatch;
		int first_resp_mismatch;
		int i;
		bit data_mismatch;
		bit resp_mismatch;
		data_mismatch = 0;
		resp_mismatch = 0;
		xact_passed = 1'b1;
		if(pkt.write_en == READ) begin
			// ---------------- READ ---------------------
			if(exp_data_queue.size() != actual_data_queue.size()) begin
				xact_passed = 1'b0;
				if(show_per_test != 1) begin
					$error("[READ XACT FAILED] Size of expected data queue does not match size of obtained queue");
					$error("Expected data queue size = %0d, Obtained data queue size = %0d",exp_data_queue.size(),actual_data_queue.size());
				end
			end
			// ---------------- READ DATA ---------------------
			foreach(exp_data_queue[i]) begin
				if(i >= actual_data_queue.size()) begin
					data_mismatch = 1'b1;
					first_data_mismatch = i;
					xact_passed = 1'b0;
					break;
				end
				else if(actual_data_queue[i].data != exp_data_queue[i]) begin
					data_mismatch = 1'b1;
					first_data_mismatch = i;
					xact_passed = 1'b0;
					break;
				end
			end
			// ---------------- READ RESP ---------------------
			foreach(actual_data_queue[i]) begin
				if(actual_data_queue[i].resp != exp_RESP) begin
					resp_mismatch = 1'b1;
					first_resp_mismatch = i;
					xact_passed = 1'b0;
					break;
				end
			end
			// -------------- READ XACT STATUS ----------------
			if(!resp_mismatch && !data_mismatch) begin
				if(show_per_test == 0) begin
					$display("[READ XACT PASSED] Addr: %h, Burst Len: %0d, Expected Resp: %2b",pkt.addr, pkt.len + 1, exp_RESP);
					foreach(exp_data_queue[i]) begin
						$display("Expected Data @[%0d] = %h, Expected Response = %2b",i,exp_data_queue[i],exp_RESP);
						$display("Obtained Data @[%0d] = %h, Obtained Response = %2b",i,actual_data_queue[i].data, actual_data_queue[i].resp);
					end
				end
			end
			else begin
				if(show_per_test != 1) begin
					$display("[READ XACT FAILED]");
				end
				if(resp_mismatch) begin
					if(show_per_test == 0 || show_per_test == 2) begin
						$error("[READ XACT RESPONSE MISMATCH] Expected response = %2b, First index of mismatch = %0d, obtained response = %2b",exp_RESP, first_resp_mismatch,actual_data_queue[first_resp_mismatch].resp);
						
					end
				end
				if(data_mismatch) begin
					if(show_per_test == 0 || show_per_test == 2) begin
						$error("[READ XACT DATA MISMATCH] First index of data mismatch = %0d",first_data_mismatch);						
					end
				end
				if(show_per_test == 0 || show_per_test == 2) begin
					foreach(exp_data_queue[i]) begin
						$display("Expected Data @[%0d] = %h, Expected Response = %2b",i,exp_data_queue[i],exp_RESP);
						if(i < actual_data_queue.size()) begin
							$display("Obtained Data @[%0d] = %h, Obtained Response = %2b",i,actual_data_queue[i].data, actual_data_queue[i].resp);
						end
					end
					for(i = exp_data_queue.size(); i< actual_data_queue.size();i++) begin
						$display("Obtained Data @[%0d] = %h, Obtained Response = %2b",i,actual_data_queue[i].data, actual_data_queue[i].resp);
					end
				end
			end
			// ------------ CHECK NO MEMORY LEAKAGE OCCURED --------------
			foreach(extra_addr_queue[i]) begin
				mem_data = axi4_top.dut.mem_inst.memory[extra_addr_queue[i]];
				if(mem_snapshot[extra_addr_queue[i]] == mem_data) begin
					if(show_per_test == 0) begin
						$display("[NO MEMORY CORRUPTION FOR READ XACT] @ address = %0d, Expected[%0d] = %h , Obtained[%0d] = %h",
										extra_addr_queue[i],extra_addr_queue[i],mem_snapshot[extra_addr_queue[i]],extra_addr_queue[i], mem_data);
					end
				end
				else begin
					xact_passed = 1'b0;
					if(show_per_test == 0 || show_per_test == 2) begin
						$error("[!!! MEMORY CORRUPTION DETECTED FOR READ XACT] @ address = %0d, Expected[%0d] = %h , Obtained[%0d] = %h",
										extra_addr_queue[i],extra_addr_queue[i],mem_snapshot[extra_addr_queue[i]],extra_addr_queue[i], mem_data);
					end
				end
			end
			// --- Update read test summary ---
			if(xact_passed) begin
				tests_passed_r++;
			end
			else begin
				tests_failed_r++;
			end
			total_tests_r++;
		end
		else begin
			// ---------------- WRITE RESP ---------------------
			if(exp_RESP != sampled_resp) begin
				xact_passed   = 1'b0;
				resp_mismatch = 1'b1;
				if(show_per_test == 0 || show_per_test == 2) begin
					$error("[WRITE XACT RESPONSE MISMATCH] Expected response = %2b, Obtained response = %2b",exp_RESP, sampled_resp);
				end
			end
			else begin
				if(show_per_test == 0) begin
					$display("[WRITE XACT RESPONSE MATCH] Expected response = %2b, Obtained response = %2b",exp_RESP, sampled_resp);
				end
			end
			
			// ---------------- WRITE DATA ---------------------
			if(exp_RESP == `OKAY) begin
				if(show_per_test == 0) begin
					$display("Checking data was written to memory @%0t ....",$time());
				end
				for(int i = 0;i<(pkt.len+1);i++) begin
					int mem_addr = ((pkt.addr&MAX_BYTE_ADDRESS)>>2);
					mem_addr = mem_addr + i;
					mem_data = $root.axi4_top.dut.mem_inst.memory[mem_addr];
					if(mem_data != expected_mem[mem_addr]) begin
						xact_passed = 1'b0;
						data_mismatch = 1'b1;
						if(show_per_test == 0 || show_per_test == 2) begin
							$error("[MEM DATA MISMATCH] expected mem[%0d] = %h, actual mem[%0d] = %h",mem_addr, expected_mem[mem_addr], mem_addr,mem_data);
						end
					end
					else begin
						if(show_per_test == 0) begin
							$display("[MEM DATA MATCH] expected mem[%0d] = %h, actual mem[%0d] = %h",mem_addr, expected_mem[mem_addr], mem_addr,mem_data);
						end
					end
				end
			end
			// ------------ CHECK NO MEMORY LEAKAGE OCCURED --------------
			foreach(extra_addr_queue[i]) begin
				mem_data = axi4_top.dut.mem_inst.memory[extra_addr_queue[i]];
				if(mem_snapshot[extra_addr_queue[i]] == mem_data) begin
					if(show_per_test == 0) begin
						$display("[NO MEMORY CORRUPTION FOR WRITE XACT] @ address = %0d, Expected[%0d] = %h , Obtained[%0d] = %h",
										extra_addr_queue[i],extra_addr_queue[i],mem_snapshot[extra_addr_queue[i]],extra_addr_queue[i], mem_data);
					end
				end
				else begin
					xact_passed = 1'b0;
					if(show_per_test == 0 || show_per_test == 2) begin
						$error("[!!! MEMORY CORRUPTION DETECTED FOR WRITE XACT] @ address = %0d, Expected[%0d] = %h , Obtained[%0d] = %h",
										extra_addr_queue[i],extra_addr_queue[i],mem_snapshot[extra_addr_queue[i]],extra_addr_queue[i], mem_data);
					end
				end
			end
			// --- Update write test summary ---
			if(xact_passed) begin
				tests_passed_w++;
			end
			else begin
				tests_failed_w++;
			end
			total_tests_w++;
		end
		
		// --- Update test summary ---
		if(xact_passed) begin
			tests_passed++;
		end
		else begin
			tests_failed++;
		end
	endtask
	
	//----------------- Back to back read xacts  ----------------//
	
	task automatic back_to_back_read_xact();
		Axi4_packet pkt1;
		Axi4_packet pkt2;
		
		logic [1:0] exp_RESP1, sampled_resp1;
		logic [1:0] exp_RESP2, sampled_resp2;
		int i;
		read_data_channel_t rd_data;
		bit read_terminated;
		
		read_terminated = 0;
		pkt1 = new();
		pkt2 = new();
		
		disable_conf_constraints(pkt1);
		disable_conf_constraints(pkt2);
		
		//------------ Generate two packets --------------
		pkt1.read_xact_c.constraint_mode(1);
		pkt2.read_xact_c.constraint_mode(1);
		generate_stimulus(pkt1);
		generate_stimulus(pkt2);
		
		//------------ Drive packets --------------
		if(show_per_test != 1) begin
			$display("Starting back to back read xacts @ time: %0t",$time());
		end
		
		//--------- Find expected memory updates of packets 1 and 2 ---------
		golden_model(pkt1,exp_RESP1);
		golden_model(pkt2,exp_RESP2);
		mem_snapshot.delete();
		extra_addr_queue.delete();
		
		fork 
			begin 
				// ---- READ ADDRESS/CONTROL CHANNEL ------
				
				//------- pkt1 drive AR signals -----------
				axi4_if.ARVALID = 1'b0; 
				repeat ( $urandom_range(0, MAX_AR_DELAY)) @(negedge axi4_if.ACLK);
				axi4_if.ARVALID = 1'b1;
				axi4_if.ARADDR  = pkt1.addr;
				axi4_if.ARLEN   = pkt1.len;
				axi4_if.ARSIZE  = pkt1.size;
				
				do @(posedge axi4_if.ACLK); while (!axi4_if.ARREADY); 
				if(show_per_test != 1) begin
					$display("[READ ADDRESS CHANNEL] pkt1 Handshake done at time: %0t",$time());
				end
				
				//------- pkt2 drive AR signals -----------
				@(negedge axi4_if.ACLK);
				axi4_if.ARADDR  = pkt2.addr;
				axi4_if.ARLEN   = pkt2.len;
				axi4_if.ARSIZE  = pkt2.size;
				
				do @(posedge axi4_if.ACLK); while (!axi4_if.ARREADY); 
				if(show_per_test != 1) begin
					$display("[READ ADDRESS CHANNEL] pkt2 Handshake done at time: %0t",$time());
				end
				
				@(negedge axi4_if.ACLK);
				axi4_if.ARVALID = 1'b0;
			end
			
			begin 
				// ---- READ DATA CHANNEL ------
				
				//------- pkt1 drive R signals -----------
				axi4_if.RREADY = 1'b0;
				@(negedge axi4_if.ACLK);
				axi4_if.RREADY = 1'b1;
				for(i = 0;i<(pkt1.len + 1);i++) begin
					do @(posedge axi4_if.ACLK); while (!axi4_if.RVALID);
					if(show_per_test != 1) begin
						$display("[READ DATA CHANNEL] pkt1 Handshake done at time: %0t",$time());
					end
					rd_data = '{data: axi4_if.RDATA,
								resp: axi4_if.RRESP
								};
					actual_data_queue.push_back(rd_data);
					if(show_per_test != 1) begin
						$display("pkt1 [READ DATA] @ time: %0t, Address: %h, Beat number: %0d, Data: %h, RLAST: %b",$time(), pkt1.addr,i, axi4_if.RDATA, axi4_if.RLAST);
					end
						
					if (axi4_if.RLAST) begin
						if(show_per_test != 1) begin
							read_terminated = 1;
							$display("pkt1 [READ XACT TERMINATED] @ time: %0t, Address: %h, Burst Length: %0d",$time(),pkt1.addr, pkt1.len + 1);
							$display("---------------------------------------------------------------------------------");
						end
						break;
					end
				end
						
				if(show_per_test != 1 && !read_terminated) begin
					$display("pkt1 [READ XACT COMPLETE] @ time: %0t, Address: %h, Burst Length: %0d",$time(),pkt1.addr, pkt1.len + 1);
					$display("---------------------------------------------------------------------------------");
				end
				
				//------- pkt2 drive R signals -----------
				for(i = 0;i<(pkt2.len + 1);i++) begin
					do @(posedge axi4_if.ACLK); while (!axi4_if.RVALID);
					if(show_per_test != 1) begin
						$display("[READ DATA CHANNEL] pkt2 Handshake done at time: %0t",$time());
					end
					rd_data = '{data: axi4_if.RDATA,
								resp: axi4_if.RRESP
								};
					actual_data_queue.push_back(rd_data);
					if(show_per_test != 1) begin
						$display("pkt2 [READ DATA] @ time: %0t, Address: %h, Beat number: %0d, Data: %h, RLAST: %b",$time(), pkt2.addr,i, axi4_if.RDATA, axi4_if.RLAST);
					end
						
					if (axi4_if.RLAST) begin
						if(show_per_test != 1) begin
							read_terminated = 1;
							$display("pkt2 [READ XACT TERMINATED] @ time: %0t, Address: %h, Burst Length: %0d",$time(),pkt2.addr, pkt2.len + 1);
							$display("---------------------------------------------------------------------------------");
						end
						break;
					end
				end
						
				if(show_per_test != 1 && !read_terminated) begin
					$display("pkt2 [READ XACT COMPLETE] @ time: %0t, Address: %h, Burst Length: %0d",$time(),pkt2.addr, pkt2.len + 1);
					$display("---------------------------------------------------------------------------------");
				end
				
				@(negedge axi4_if.ACLK);
				axi4_if.RREADY = 0;
			end
			
		join
		
		//--------- Find expected memory updates of packets 1 and 2 ---------
		@(negedge axi4_if.ACLK);
		check_results(pkt1, exp_RESP1, sampled_resp1);
		check_results(pkt2, exp_RESP2, sampled_resp2);
		
	endtask
	
	//----------------- Back to back write xacts  ---------------//
	task automatic back_to_back_write_xact(input bit same_pkt);
		Axi4_packet pkt1;
		Axi4_packet pkt2;
		
		logic [1:0] exp_RESP1, sampled_resp1;
		logic [1:0] exp_RESP2, sampled_resp2;
		int i;
		
		pkt1 = new();
		pkt2 = new();
		
		disable_conf_constraints(pkt1);
		disable_conf_constraints(pkt2);
		
		//------------ Generate two packets --------------
		pkt1.write_xact_c.constraint_mode(1);
		pkt2.write_xact_c.constraint_mode(1);
		generate_stimulus(pkt1);
		generate_stimulus(pkt2);
		if(same_pkt) begin
			pkt2.addr = pkt1.addr;
		end
		
		//------------ Drive packets --------------
		if(show_per_test != 1) begin
			$display("Starting back to back write xacts @ time: %0t",$time());
		end
		
		//--------- Find expected memory updates of packets 1 and 2 ---------
		golden_model(pkt1,exp_RESP1);
		golden_model(pkt2,exp_RESP2);
		mem_snapshot.delete();
		extra_addr_queue.delete();
		
		fork 
			begin 
				// ---- WRITE ADDRESS/CONTROL CHANNEL ------
				
				//------- pkt1 drive AW signals -----------
				axi4_if.AWVALID = 1'b0; //master stall 
				repeat ($urandom_range(0, MAX_AW_DELAY)) @(negedge axi4_if.ACLK);
				axi4_if.AWVALID = 1'b1;
				axi4_if.AWADDR  = pkt1.addr;
				axi4_if.AWLEN   = pkt1.len;
				axi4_if.AWSIZE  = pkt1.size;
				
				do @(posedge axi4_if.ACLK); while (!axi4_if.AWREADY); //wait for ready to complete handshake
				if(show_per_test != 1) begin
					$display("[WRITE ADDRESS CHANNEL] Handshake done for pkt1 at time: %0t",$time());
				end
				
				//------- pkt2 drive AW signals -----------
				@(negedge axi4_if.ACLK);
				axi4_if.AWADDR  = pkt2.addr;
				axi4_if.AWLEN   = pkt2.len;
				axi4_if.AWSIZE  = pkt2.size;
				
				do @(posedge axi4_if.ACLK); while (!axi4_if.AWREADY); //wait for ready to complete handshake
				if(show_per_test != 1) begin
					$display("[WRITE ADDRESS CHANNEL] Handshake done for pkt2 at time: %0t",$time());
				end
				@(negedge axi4_if.ACLK);
				axi4_if.AWVALID = 1'b0;			
			end
			
			begin 
				// ---- WRITE DATA CHANNEL ------
				if(show_per_test != 1) begin
					$display("Start sending burst write data at time: %0t",$time());
				end
				//------- pkt1 drive W signals -----------
				for(i = 0;i<(pkt1.len+1);i++) begin
					@(negedge axi4_if.ACLK);
					axi4_if.WVALID = 1'b1;
					axi4_if.WDATA  = pkt1.burst_data[i];
					axi4_if.WLAST  = (i == (pkt1.len));
					do @(posedge axi4_if.ACLK); while (!axi4_if.WREADY);
					if(show_per_test != 1) begin
						$display("[WRITE DATA CHANNEL] Handshake done for pkt1 beat number: %0d, at time: %0t",i,$time());
					end
				end
				
				//------- pkt2 drive W signals -----------
				for(i = 0;i<(pkt2.len+1);i++) begin
					@(negedge axi4_if.ACLK);
					axi4_if.WVALID = 1'b1;
					axi4_if.WDATA  = pkt2.burst_data[i];
					axi4_if.WLAST  = (i == (pkt2.len));
					do @(posedge axi4_if.ACLK); while (!axi4_if.WREADY);
					if(show_per_test != 1) begin
						$display("[WRITE DATA CHANNEL] Handshake done for pkt2 beat number: %0d, at time: %0t",i,$time());
					end
				end
				
				@(negedge axi4_if.ACLK);
				axi4_if.WVALID = 1'b0;
				axi4_if.WLAST  = 1'b0;
			end
			
			begin 
				// ---- WRITE RESPONSE CHANNEL ------
				
				// ----------- pkt1 resp keep BREADY asserted ----------
				axi4_if.BREADY = 1'b0; 
				if($urandom_range(0,VALID_BEFORE_READY_RATIO) == VALID_BEFORE_READY_RATIO) begin
					repeat ($urandom_range(0, MAX_BREADY_DELAY))  @(negedge axi4_if.ACLK);
					axi4_if.BREADY = 1'b1; 
					do @(posedge axi4_if.ACLK); while (!axi4_if.BVALID);
				end
				else begin
					do @(posedge axi4_if.ACLK); while (!axi4_if.BVALID);
					repeat ($urandom_range(1, MAX_BREADY_DELAY))  @(negedge axi4_if.ACLK);
					axi4_if.BREADY = 1'b1;
					@(posedge axi4_if.ACLK);
					if(show_per_test != 1) begin
						$display("BREADY asserted at time: %0t",$time());		
					end

					if(show_per_test != 1 && !axi4_if.BVALID) begin
						$display("[HANDSHAKE VIOLATION] BVALID got asserted and deasserted while BREADY was low");
						do @(posedge axi4_if.ACLK); while (!axi4_if.BVALID);
					end
				end
				
				if(show_per_test != 1) begin
					$display("[WRITE RESPONSE CHANNEL] pkt1 Handshake done at time: %0t",$time());
				end
				sampled_resp1 = axi4_if.BRESP;
				
				// ----------- pkt2 resp keep BREADY asserted ----------
				do @(posedge axi4_if.ACLK); while (!axi4_if.BVALID);
				if(show_per_test != 1) begin
					$display("[WRITE RESPONSE CHANNEL] pkt2 Handshake done at time: %0t",$time());
				end
				sampled_resp2 = axi4_if.BRESP;
				@(negedge axi4_if.ACLK);
				axi4_if.BREADY = 1'b0;	
			end
		join
		
		
		//--------- Find expected memory updates of packets 1 and 2 ---------
		@(negedge axi4_if.ACLK);
		check_results(pkt1, exp_RESP1, sampled_resp1);
		check_results(pkt2, exp_RESP2, sampled_resp2);
			
	endtask
	//--------------- Reset Midst Address Phase -----------------//
	task automatic reset_midst_address(Axi4_packet pkt);
		if(show_per_test != 1) begin
			$display("[RESET ADDR PHASE TEST] Starting @ time: %0t",$time());
		end
		fork 
			begin
				//----- Drive signals in control channel and wait for handshake ----
				drive_stim(pkt,sampled_resp);
			end		
			begin
				//----- Launch reset after address handshake ----
				if(pkt.write_en == WRITE) begin
					@(posedge axi4_if.ACLK iff (axi4_if.AWVALID && axi4_if.AWREADY));
					if(show_per_test != 1) begin
						$display("[WRITE ADDRESS CHANNEL] Handshake observed in reset thread at time: %0t",$time());
					end
					#1ns; //wait delay to ensure we entered ADDR phase
					axi4_if.ARESETn = 1'b0;
					axi4_if.AWVALID = 1'b0;
					if(show_per_test != 1) begin
						$display("[RESET ASSERTED] @ time: %0t",$time());
					end
					@(posedge axi4_if.ACLK);
					@(negedge axi4_if.ACLK);
					axi4_if.ARESETn = 1'b1;
				end
				else begin
					@(posedge axi4_if.ACLK iff (axi4_if.ARVALID && axi4_if.ARREADY));
					if(show_per_test != 1) begin
						$display("[READ ADDRESS CHANNEL] Handshake observed in reset thread at time: %0t",$time());
					end
					#1ns; //wait delay to ensure we entered ADDR phase
					axi4_if.ARESETn = 1'b0;
					axi4_if.ARVALID = 1'b0;
					if(show_per_test != 1) begin
						$display("[RESET ASSERTED] @ time: %0t",$time());
					end
					@(posedge axi4_if.ACLK);
					@(negedge axi4_if.ACLK);
					axi4_if.ARESETn = 1'b1;
				end	
			end
		join_any
		
		//---- check system recovered correctly from reset
		check_sys_recovery();
		
		//-- kill background drive stimulus thread so it does not hang
		disable fork;
		
		//---- cleanup after reset assertion
		@(negedge axi4_if.ACLK);
		reset();
		if(show_per_test != 1) begin
			$display("[RESET TEST COMPLETE] Reset midst address phase is complete @ time: %0t",$time());
		end
	endtask
	
	//--------------- Reset Midst Data Phase -----------------//
	task automatic reset_midst_data(Axi4_packet pkt, int reset_beat);
		int burst_len;
		int base_addr;
		int last_burst_addr; 
		
		bit reset_fired;
		burst_len           = pkt.len + 1;
		base_addr           = (pkt.addr&MAX_BYTE_ADDRESS);
		last_burst_addr     = base_addr + ((pkt.len)<<pkt.size);
		accepted_data_beats = 0;
		reset_fired         = 0;
		exp_RESP = `OKAY;
		
		//--------------- BOUNDARY CHECK ---------------------
		if(last_burst_addr > MAX_BYTE_ADDRESS || (pkt.addr > MAX_BYTE_ADDRESS)) begin
			exp_RESP = `SLVERR;
		end
		
		if(show_per_test != 1) begin
			$display("[RESET DATA PHASE TEST] Starting @ time: %0t",$time());
		end
		fork 
			begin
				//----- Drive signals in data channel and wait for handshake ----
				drive_stim(pkt,sampled_resp);
				forever @(posedge axi4_if.ACLK);
			end		
			begin
				//----- Launch reset after address handshake ----
				if(pkt.write_en == WRITE) begin
					//------------------------- WRITE ------------------------------------
					@(posedge axi4_if.ACLK iff (axi4_if.AWVALID && axi4_if.AWREADY));
					if(show_per_test != 1) begin
						$display("[WRITE ADDRESS CHANNEL] Handshake observed in reset thread at time: %0t",$time());
					end
					//enter ADDR phase
					@(posedge axi4_if.ACLK); 
					#1ns; //enter DATA phase
					for(int i = 0;i<(pkt.len + 1);i++) begin
						int mem_addr = (base_addr>>2) + i;
						
						if(accepted_data_beats == reset_beat) begin
							//-------- Fire Reset -----------
							break;
						end
						do @(posedge axi4_if.ACLK); while (!($sampled(axi4_if.WVALID) && $sampled(axi4_if.WREADY)));
						accepted_data_beats++;
						if(exp_RESP == `OKAY) begin
							expected_mem[mem_addr] = pkt.burst_data[i];
						end
						if(show_per_test != 1) begin
							$display("[WRITE DATA CHANNEL] Handshake observed in reset thread for beat number: %0d, at time: %0t",i,$time());
						end
					end
					if(accepted_data_beats == reset_beat) begin
						//-------- Fire Reset -----------
						#1ns;
						axi4_if.ARESETn = 1'b0;
						if(show_per_test != 1) begin
							$display("[RESET ASSERTED] after transmitting %0d beats @ time: %0t",accepted_data_beats,$time());
						end
						@(posedge axi4_if.ACLK);
						@(negedge axi4_if.ACLK);
						axi4_if.ARESETn = 1'b1;
						if(show_per_test != 1) begin
							$display("[RESET DEASSERTED] @ time: %0t",$time());
						end	
					end
				end
				else begin
					//------------------------- READ ------------------------------------
					@(posedge axi4_if.ACLK iff (axi4_if.ARVALID && axi4_if.ARREADY));
					if(show_per_test != 1) begin
						$display("[READ ADDRESS CHANNEL] Handshake observed in reset thread at time: %0t",$time());
					end
					//enter ADDR phase
					@(posedge axi4_if.ACLK); 
					#1ns; //enter DATA phase
					for(int i = 0;i<(pkt.len + 1);i++) begin
						int mem_addr = (base_addr>>2) + i;
						
						if(accepted_data_beats == reset_beat) begin
							//-------- Fire Reset -----------
							break;
						end
						do @(posedge axi4_if.ACLK); while (!($sampled(axi4_if.RVALID) && $sampled(axi4_if.RREADY)));
						accepted_data_beats++;
						if(exp_RESP == `OKAY) begin
							mem_data = axi4_top.dut.mem_inst.memory[mem_addr];
							exp_data_queue.push_back(mem_data);
						end
						else begin
							exp_data_queue.push_back(0);
						end
						
						if(show_per_test != 1) begin
							$display("[READ DATA CHANNEL] Handshake observed in reset thread for beat number: %0d, at time: %0t",i,$time());
						end
					end
					if(accepted_data_beats == reset_beat) begin
						//-------- Fire Reset -----------
						#1ns;
						axi4_if.ARESETn = 1'b0;
						if(show_per_test != 1) begin
							$display("[RESET ASSERTED] after transmitting %0d beats @ time: %0t",accepted_data_beats,$time());
						end
						@(posedge axi4_if.ACLK);
						@(negedge axi4_if.ACLK);
						axi4_if.ARESETn = 1'b1;
						if(show_per_test != 1) begin
							$display("[RESET DEASSERTED] @ time: %0t",$time());
						end	
					end
					
				end	
			end
		join_any
		
		//---- check system recovered correctly from reset
		check_sys_recovery();
		
		//---- check bursts were executed correctly before termination
		mem_snapshot.delete();
		extra_addr_queue.delete();
		
		// -------- give time to driver to try to drive extra beats ------
		repeat (DRIVER_POST_RESET_MARGIN) @(negedge axi4_if.ACLK);
		
		//-- kill background drive stimulus thread so it does not hang
		disable fork;
		
		check_results(pkt,exp_RESP,exp_RESP); //due to aborting Xact before reaching response phase we ignore checks on response. 	
	
		//---- cleanup after reset assertion
		@(negedge axi4_if.ACLK);
		reset();
		if(show_per_test != 1) begin
			$display("[RESET TEST COMPLETE] Reset midst data phase is complete @ time: %0t",$time());
		end
	endtask

	//------------ Concurrent read and write xacts -----------//
	task automatic parallel_read_write_xact(Axi4_packet pkt_r, Axi4_packet pkt_w);
		
		logic [1:0]                      exp_RESP_r, sampled_resp_r;
		logic [1:0]                      exp_RESP_w, sampled_resp_w;
		logic [axi4_if.DATA_WIDTH - 1:0] mem_data_snapshot[];
		bit                              same_addr;
		int                              read_first_count;
		int                              write_first_count;
		int                              mixed_count;
		int                              min_burst_count;
		
		read_first_count  = 0;
		write_first_count = 0;
		mixed_count       = 0;
		min_burst_count   = (pkt_r.len < pkt_w.len)?(pkt_r.len+1):(pkt_w.len+1);
		
		//------------- Ensure no random delays to have true racing ----------
		
		//------------ Drive packets --------------
		if(show_per_test != 1) begin
			$display("Starting concurrent read and write xacts @ time: %0t",$time());
			pkt_r.display();
			pkt_w.display();
		end
		
		//--------- Find expected memory reads/writes of packets given write has higher priority ---------
		extra_addr_queue.delete();
		
		golden_model(pkt_w,exp_RESP_w);
		extra_addr_queue.delete();
		
		golden_model(pkt_r,exp_RESP_r);
		
		extra_addr_queue.delete();
		mem_snapshot.delete();
		
		same_addr = (exp_RESP_w == `OKAY && exp_RESP_r == `OKAY && ( (pkt_r.addr & MAX_BYTE_ADDRESS) == (pkt_w.addr & MAX_BYTE_ADDRESS) ) );
		
		if(same_addr) begin
			rw_race_tests++;
			mem_data_snapshot = new[pkt_r.len + 1];
			for(int i = 0;i<(pkt_r.len+1);i++) begin
				int mem_addr = ((pkt_r.addr&MAX_BYTE_ADDRESS)>>2);
				mem_addr = (mem_addr + i)&(MEM_DEPTH - 1);
				mem_data = axi4_top.dut.mem_inst.memory[mem_addr];
				mem_data_snapshot[i] = mem_data;
				if(show_per_test != 1) begin
					$display("[MEM CONTENT] Addr: %0d, Data: %h, @ time: %0t",mem_addr,mem_data,$time());
				end
			end
		end
		fork 
			// ---- WRITE PACKET ------
			drive_stim(pkt_w,sampled_resp_w);
				
			// ---- READ PACKET ------
			drive_stim(pkt_r,sampled_resp_r);
		join
		
		//--------- Check xacts ---------
		@(negedge axi4_if.ACLK);
		
		
		if(!same_addr) begin
			// two xacts target different addresses
			check_results(pkt_r, exp_RESP_r, sampled_resp_r);
			check_results(pkt_w, exp_RESP_w, sampled_resp_w);
		end
		else begin
			// two xacts target same addresses
			if(actual_data_queue.size() != (pkt_r.len + 1)) begin
				mixed_count++;
				if(show_per_test != 1) begin
					$display("[W/R RACE STATUS: READ XACT FAILED] In W/R race captured read beats do not match intended.");
					$display("Captured number of beats: %0d, Intended number of beats: %0d",actual_data_queue.size(), pkt_r.len + 1);
				end
			end
			else begin
				foreach(actual_data_queue[i]) begin
					if(actual_data_queue[i].data == mem_data_snapshot[i]) begin
						read_first_count++;
						if(show_per_test != 1) begin
							$display("[READ WINS RACE] For beat number %0d, old memory data is read first before it gets overwritten",i);
							$display("Captured data: %h, Memory data: %h",actual_data_queue[i].data, mem_data_snapshot[i]);
						end
					end
					else if(i < (pkt_w.len+1) && actual_data_queue[i].data == pkt_w.burst_data[i]) begin
						write_first_count++;
						if(show_per_test != 1) begin
							$display("[WRITE WINS RACE] For beat number %0d, new memory data is read first",i);
							$display("[READ WINS RACE] Captured data: %h, Memory data: %h, Written data: %h",actual_data_queue[i].data, mem_data_snapshot[i], pkt_w.burst_data[i]);
						end
					end
					else begin
						mixed_count++;
						if(show_per_test != 1) begin
							$display("[READ CORRUPT] For beat number %0d, read data is neither old memory data or overwritten data",i);
							$display("Captured data: %h, Memory data: %h",actual_data_queue[i].data, mem_data_snapshot[i]);
						end
					end
				end
			end
			
			$display("Total Beats Evaluated: %0d", actual_data_queue.size());
			$display("Read returned NEW data (Write won the race) : %0d beats", write_first_count);
			$display("Read returned OLD data (Read won the race)  : %0d beats", read_first_count);
			$display("Read returned CORRUPT data                  : %0d beats", mixed_count);
			
			if (write_first_count == min_burst_count) begin
				write_first++;
				$display("RESULT: [WRITE PRIORITY] The DUT serves Writes first.");
			end else if (read_first_count == (pkt_r.len + 1)) begin
				read_first++;
				$display("RESULT: [READ PRIORITY] The DUT serves Reads first.");
			end else if(mixed_count > 0) begin
				corrupt_read++;
				$display("RESULT: [READ CORRUPT] The DUT cannot support simultaneous reads and writes to the same address");
			end else begin
				no_rw_order_rule++;
				$display("RESULT: [INTERLEAVED/RACE] The DUT switches mid-burst.");
			end
			
		end
			
	endtask

	//----- Concurrent read and write back to back xacts -----//
	task automatic parallel_read_write_back2back_xact(Axi4_packet pkt_r, Axi4_packet pkt_w);
		
		//------------ Drive packets --------------
		if(show_per_test != 1) begin
			$display("Starting concurrent read and write back to back xacts @ time: %0t",$time());
		end
				
		fork 
			// ---- WRITE PACKET ------
			back_to_back_write_xact(same_pkt);
				
			// ---- READ PACKET ------
			back_to_back_read_xact();
		join
		
		
			
	endtask

	//----------------- WLAST Assertion violation ---------------//
	task automatic wlast_assertion_violation(Axi4_packet pkt, wlast_err_e wlast_err);
		int early_wlast;
		int delayed_wlast;
		logic [axi4_if.DATA_WIDTH] additional_writes[];
		int burst_len;
		int base_addr;
		int last_burst_addr; 
		
		int bvalid_wait_cycles;
		int wready_wait_cycles;
		int intended_no_beats;
		int beat_taken;
		int beats_written;
		int actual_beats_no;
			
		bit slave_bvalid_not_asserted;
		bit slave_wready_not_asserted;
		bit addr_phase_done;
		bit data_phase_done;
		
		addr_phase_done           = 0;
		data_phase_done           = 0;
		burst_len                 = pkt.len + 1;
		base_addr                 = (pkt.addr&MAX_BYTE_ADDRESS);
		last_burst_addr           = base_addr + ((pkt.len)<<pkt.size);
		exp_RESP                  = `OKAY;
		slave_wready_not_asserted = 0;
		slave_bvalid_not_asserted = 0;
		//--------------- BOUNDARY CHECK ---------------------
		if(last_burst_addr > MAX_BYTE_ADDRESS || (pkt.addr > MAX_BYTE_ADDRESS)) begin
			exp_RESP = `SLVERR;
		end
		else begin
			valid_writes++;
		end
		
		early_wlast       = $urandom_range(1,pkt.len);
		delayed_wlast     = (wlast_err == DELAYED_WLAST) ? $urandom_range(1,MAX_WDATA_DELAY) : 0;
		additional_writes = new[delayed_wlast];
		foreach(additional_writes[i]) begin
			additional_writes[i] = $urandom();
		end
		//------------ Drive packets --------------
		if(show_per_test != 1) begin
			$display("Starting %s test @ time: %0t",wlast_err.name(),$time());
			
		end
		beat_taken = 0;	
		fork 
			begin 
				// ---- WRITE ADDRESS/CONTROL CHANNEL ------
				
				//------- pkt drive AW signals -----------
				axi4_if.AWVALID = 1'b0; //master stall 
				repeat ($urandom_range(0, MAX_AW_DELAY)) @(negedge axi4_if.ACLK);
				axi4_if.AWVALID = 1'b1;
				axi4_if.AWADDR  = pkt.addr;
				axi4_if.AWLEN   = pkt.len;
				axi4_if.AWSIZE  = pkt.size;
				
				do @(posedge axi4_if.ACLK); while (!axi4_if.AWREADY); //wait for ready to complete handshake
				if(show_per_test != 1) begin
					$display("[WRITE ADDRESS CHANNEL] Handshake done at time: %0t",$time());
				end
				addr_phase_done = 1;
				@(negedge axi4_if.ACLK);
				axi4_if.AWVALID = 1'b0;			
			end
			
			begin 
				// ---- WRITE DATA CHANNEL ------
				if(show_per_test != 1) begin
					$display("Start sending burst write data at time: %0t",$time());
				end
				//------- pkt drive W signals -----------
				for(int i = 0;i<(pkt.len+1);i++) begin
					@(negedge axi4_if.ACLK);
					axi4_if.WVALID = 1'b0;
					repeat ($urandom_range(0, MAX_WDATA_DELAY))  @(negedge axi4_if.ACLK);
					axi4_if.WVALID = 1'b1;
					axi4_if.WDATA  = pkt.burst_data[i];
					if(wlast_err == EARLY_WLAST) begin
						axi4_if.WLAST  = (i == (pkt.len - early_wlast));
						
					end
					else begin
						axi4_if.WLAST  = ((wlast_err == DELAYED_WLAST)? 0:(i == (pkt.len)));
					end		
					wready_wait_cycles = 0;
					do begin 
						@(posedge axi4_if.ACLK);
						if(addr_phase_done) begin
							wready_wait_cycles++;
						end
						else wready_wait_cycles = 0;
						if(wready_wait_cycles == WREADY_TIMEOUT) begin
							slave_wready_not_asserted = 1'b1;
							if(show_per_test != 1) begin
								$display("[DIAGNOSTIC] WREADY wait time-out for beat number %0d. Terminating sending new beats @ time = ",i,$time());
							end
							break;
						end
					end while (!axi4_if.WREADY);
					if(slave_wready_not_asserted) break;
					beat_taken++;
					if(show_per_test != 1) begin
						$display("[WRITE DATA CHANNEL] Handshake done for beat number: %0d, at time: %0t",i,$time());
					end
				end
				if(wlast_err != DELAYED_WLAST || slave_wready_not_asserted) data_phase_done = 1;
				if(wlast_err == DELAYED_WLAST && !slave_wready_not_asserted) begin
					for(int i = 0;i<delayed_wlast;i++) begin
						
						@(negedge axi4_if.ACLK);
						axi4_if.WVALID = 1'b0;
						repeat ($urandom_range(0, MAX_WDATA_DELAY))  @(negedge axi4_if.ACLK);
						axi4_if.WVALID = 1'b1;
						axi4_if.WDATA  = additional_writes[i];
						axi4_if.WLAST  = (i == (delayed_wlast));	
						wready_wait_cycles = 0;
						do begin 
							@(posedge axi4_if.ACLK);
							if(addr_phase_done) begin
								wready_wait_cycles++;
							end
							else wready_wait_cycles = 0;
							if(wready_wait_cycles == WREADY_TIMEOUT) begin
								slave_wready_not_asserted = 1'b1;
								if(show_per_test != 1) begin
									$display("[DIAGNOSTIC] WREADY wait time-out for beat number %0d. Terminating sending new beats @ time = ",pkt.len+1+i,$time());
								end
								break;
							end
						end while (!axi4_if.WREADY);
						
						if(slave_wready_not_asserted) break;
						beat_taken++;
						if(show_per_test != 1) begin
							$display("[WRITE DATA CHANNEL] Handshake done for additional beat number: %0d, at time: %0t",pkt.len+1+i,$time());
						end
					end
					data_phase_done = 1;
				end
				
				@(negedge axi4_if.ACLK);
				axi4_if.WVALID = 1'b0;
				axi4_if.WLAST  = 1'b0;
			end
			
			begin 
				// ---- WRITE RESPONSE CHANNEL ------
			
				axi4_if.BREADY = 1'b0; 
				repeat ($urandom_range(0, MAX_BREADY_DELAY))  @(negedge axi4_if.ACLK);
				axi4_if.BREADY = 1'b1; 
				bvalid_wait_cycles = 0;
				do begin 
					@(posedge axi4_if.ACLK);
					if(data_phase_done && addr_phase_done) begin
						bvalid_wait_cycles++;
					end
					else bvalid_wait_cycles = 0;
					if(bvalid_wait_cycles == BVALID_TIMEOUT) begin
						slave_bvalid_not_asserted = 1'b1;
						if(show_per_test != 1) begin
							$display("[DIAGNOSTIC] BVALID wait time-out @ time = ",$time());
						end
						break;
					end
				end while (!axi4_if.BVALID);	
				if(!slave_bvalid_not_asserted) begin
					if(show_per_test != 1) begin
						$display("[WRITE RESPONSE CHANNEL] Handshake done at time: %0t",$time());
					end
					sampled_resp = axi4_if.BRESP;
				end
				@(negedge axi4_if.ACLK);
				axi4_if.BREADY = 1'b0;	
			end
		join
				
		// -----------------------------------------------------------------
		// ------------------- THE DIAGNOSTIC MATRIX -----------------------
		// -----------------------------------------------------------------
		if(wlast_err == EARLY_WLAST) begin
			intended_no_beats = pkt.len + 1 - early_wlast;
		end
		else if(wlast_err == DELAYED_WLAST) begin
			intended_no_beats = pkt.len + 1 + delayed_wlast; 
		end
		else begin
			intended_no_beats = pkt.len + 1;
		end
		if(show_per_test == 0) begin
			$display("=====================================================");
			$display("   WLAST DIAGNOSTIC REPORT: %s", wlast_err.name());
			$display("=====================================================");
		end
		@(negedge axi4_if.ACLK);
		
		if(slave_bvalid_not_asserted) begin
			failed_to_terminate ++;
			if(show_per_test != 1) begin
				$display("RESULT: SLAVE CANNOT SUPPORT WLAST VIOLATION");
				$display("The Slave could not complete the write xact. It couldn't handle the WLAST violation and never sent BVALID.");
			end
		end
		else begin
			if(show_per_test != 1) begin
				$display("Number of beats accepted = %0d",beat_taken);
				$display("Intented number of beats with WLAST = %0d",intended_no_beats);
				$display("Intented number of beats with AWLEN = %0d", pkt.len+1);
				if(slave_wready_not_asserted) begin
					$display("Slave stopped accepting additional beats after %0d beats",beat_taken);
				end
			end			
			
			// ----- acceptance to beat behaviour ------ 
			if(wlast_err == EARLY_WLAST) begin //early WLAST 
				if(beat_taken == intended_no_beats && slave_wready_not_asserted && !slave_bvalid_not_asserted) begin
					accepted_on_early_WLAST++;
					if(show_per_test != 1) begin
						$display("RESULT: SLAVE ACCEPTS BEATS UNTIL EARLY LAST ASSERTION");
					end
				end
				else if(beat_taken == (pkt.len + 1) && !slave_wready_not_asserted && !slave_bvalid_not_asserted) begin
					accepted_on_late_WLEN++;
					if(show_per_test != 1) begin
						$display("RESULT: SLAVE ACCEPTS BEATS ACCORDING TO AWLEN REGARDLESS OF EARLY WLAST");
					end
				end
				else begin
					accepted_on_other++;
					if(show_per_test != 1) begin
						$display("RESULT: SLAVE ACCEPTED BEATS DOES NOT DEPENDED ON WLAST OR AWLEN");
						$display("bvalid assertion: %b, data phase completetion: %b, beats taken: %0d, intented no of beats: %0d, beats in awlen: %0d",
							!slave_bvalid_not_asserted, !slave_wready_not_asserted, beat_taken,intended_no_beats,pkt.len+1);
					end
				end
			end
			else if(wlast_err == DELAYED_WLAST) begin //delayed WLAST 
				if(beat_taken == intended_no_beats && !slave_wready_not_asserted && !slave_bvalid_not_asserted) begin
					accepted_on_late_WLAST++;
					if(show_per_test != 1) begin
						$display("RESULT: SLAVE ACCEPTS BEATS UNTIL DELAYED LAST ASSERTION");
					end
				end
				else if(beat_taken == (pkt.len + 1) && slave_wready_not_asserted && !slave_bvalid_not_asserted) begin
					accepted_on_early_WLEN++;
					if(show_per_test != 1) begin
						$display("RESULT: SLAVE ACCEPTS BEATS ACCORDING TO AWLEN REGARDLESS OF LATE WLAST");
					end
				end
				else begin
					accepted_on_other++;
					if(show_per_test != 1) begin
						$display("RESULT: SLAVE ACCEPTED BEATS THAT DO NOT DEPEND ON WLAST OR AWLEN");
						$display("bvalid assertion: %b, data phase completetion: %b, beats taken: %0d, intented no of beats: %0d, beats in awlen: %0d",
							!slave_bvalid_not_asserted, !slave_wready_not_asserted, beat_taken,intended_no_beats,pkt.len+1);
					end
				end
			end
			// ----- writing to beat behaviour ------ 
			if(exp_RESP == `OKAY) begin
				beats_written = 0;
				for(int i = 0;i<(pkt.len+1);i++) begin
					int mem_addr = ((pkt.addr&MAX_BYTE_ADDRESS)>>2);
					mem_addr = (mem_addr + i)&(MEM_DEPTH - 1);
					mem_data = axi4_top.dut.mem_inst.memory[mem_addr];
					if(pkt.burst_data[i] == mem_data) beats_written++;
					else if(show_per_test != 1) begin
						$display("[DIAGNOSTIC] Mem mismatch at index %0d in burst",i);
					end
				end
				for(int i = 0;i<additional_writes.size();i++) begin
					int mem_addr = ((pkt.addr&MAX_BYTE_ADDRESS)>>2);
					mem_addr = (mem_addr + i + pkt.len + 1)&(MEM_DEPTH - 1);
					mem_data = axi4_top.dut.mem_inst.memory[mem_addr];
					if(additional_writes[i] == mem_data) beats_written++;
					else if(show_per_test != 1) begin
						$display("[DIAGNOSTIC] Mem mismatch at index %0d in burst",i + pkt.len + 1);
					end
				end
				
			end
			
			if(show_per_test != 1) begin
				$display("Number of beats written = %0d",beats_written);
				$display("Intented number of beats with WLAST = %0d",intended_no_beats);
				$display("Intented number of beats with AWLEN = %0d", pkt.len+1);
			end			
			
			// ----- write beat behaviour ------ 
			if(wlast_err == EARLY_WLAST) begin //early WLAST 
				if(beats_written == intended_no_beats && slave_wready_not_asserted && !slave_bvalid_not_asserted) begin
					write_on_early_WLAST++;
					if(show_per_test != 1) begin
						$display("RESULT: SLAVE WRITES BEATS UNTIL EARLY LAST ASSERTION");
					end
				end
				else if(beats_written == (pkt.len + 1) && !slave_wready_not_asserted && !slave_bvalid_not_asserted) begin
					write_on_late_WLEN++;
					if(show_per_test != 1) begin
						$display("RESULT: SLAVE WRITES BEATS ACCORDING TO AWLEN REGARDLESS OF EARLY WLAST");
					end
				end
				else begin
					write_on_other++;
					if(show_per_test != 1) begin
						$display("RESULT: SLAVE WRITES ON BEATS THAT DO NOT DEPEND ON WLAST OR AWLEN");
						$display("bvalid assertion: %b, data phase completetion: %b, beats taken: %0d, intented no of beats: %0d, beats in awlen: %0d",
							!slave_bvalid_not_asserted, !slave_wready_not_asserted, beats_written,intended_no_beats,pkt.len+1);
					end
				end
			end
			else if(wlast_err == DELAYED_WLAST) begin //delayed WLAST 
				if(beats_written == intended_no_beats && !slave_wready_not_asserted && !slave_bvalid_not_asserted) begin
					write_on_late_WLAST++;
					if(show_per_test != 1) begin
						$display("RESULT: SLAVE WRITES BEATS UNTIL DELAYED LAST ASSERTION");
					end
				end
				else if(beats_written == (pkt.len + 1) && slave_wready_not_asserted && !slave_bvalid_not_asserted) begin
					write_on_early_WLEN++;
					if(show_per_test != 1) begin
						$display("RESULT: SLAVE WRITES BEATS ACCORDING TO AWLEN REGARDLESS OF LATE WLAST");
					end
				end
				else begin
					write_on_other++;
					if(show_per_test != 1) begin
						$display("RESULT: SLAVE WRITES BEATS THAT DO NOT DEPEND ON WLAST OR AWLEN");
						$display("bvalid assertion: %b, data phase completetion: %b, beats taken: %0d, intented no of beats: %0d, beats in awlen: %0d",
							!slave_bvalid_not_asserted, !slave_wready_not_asserted, beats_written,intended_no_beats,pkt.len+1);
					end
				end
			end
			
		end
	endtask
	
	//-------------- Check dut recovery from reset --------------//
	task check_sys_recovery();
		bit pass ;
		pass = 1;
		pass = pass && (axi4_if.AWREADY == 1'b1);  
        pass = pass && (axi4_if.WREADY  == 1'b0);
        pass = pass && (axi4_if.BVALID  == 1'b0);
        pass = pass && (axi4_if.BRESP   == 2'b00);
            
		pass = pass && (axi4_if.ARREADY == 1'b1);  
        pass = pass && (axi4_if.RLAST   == 1'b0);
        pass = pass && (axi4_if.RVALID  == 1'b0);
        pass = pass && (axi4_if.RRESP   == 2'b00);
		pass = pass && (axi4_if.RDATA   == ({axi4_if.DATA_WIDTH{1'b0}}));

		if(pass) begin
			tests_passed ++;
			if(show_per_test == 0) begin
				$display("[RESET RECOVERY TEST PASSED] @ time: %0t",$time());
			end
		end
		else begin
			tests_failed ++;
			if(show_per_test == 0|| show_per_test == 2) begin
				$error("[RESET RECOVERY TEST FAILED] DUT did not recover correctly from reset @ time: %0t",$time());
			end
		end
		
	endtask
	//----------------- Print Test bench summary  ---------------//
	task print_summary(input int config_no);
		$display("-------- @time: %0t , test bench summary for configuration number: %0d------ ", $time(),config_no);
		$display("tests failed: %0d out of %0d ", tests_failed,total_tests);
		$display("invalid tests: %0d out of %0d", tests_invalid,total_tests);
		$display("tests passed: %0d out of %0d", tests_passed,total_tests);
		$display("[READ] tests failed: %0d out of %0d ", tests_failed_r,total_tests_r);
		$display("[READ] invalid tests: %0d out of %0d", tests_invalid_r,total_tests_r);
		$display("[READ] tests passed: %0d out of %0d", tests_passed_r,total_tests_r);
		$display("[WRITE] tests failed: %0d out of %0d ", tests_failed_w,total_tests_w);
		$display("[WRITE] invalid tests: %0d out of %0d", tests_invalid_w,total_tests_w);
		$display("[WRITE] tests passed: %0d out of %0d", tests_passed_w,total_tests_w);
	endtask

	//----------------- WLAST inspection summary  ---------------//
	task print_wlast_tests_results(input int config_no);	
		$display("-------- @time: %0t , WLAST inspection summary for configuration number: %0d------ ", $time(),config_no);
		
		$display("failed to terminate: %0d out of %0d ", failed_to_terminate,valid_writes);
		
		$display("accepted on early WLAST: %0d out of %0d ", accepted_on_early_WLAST,total_tests);
		$display("accepted on delayed WLAST: %0d out of %0d ", accepted_on_late_WLAST,total_tests);
		$display("accepted on early WLEN: %0d out of %0d ", accepted_on_early_WLEN,total_tests);
		$display("accepted on delayed WLEN: %0d out of %0d ", accepted_on_late_WLEN,total_tests);
		$display("accepted on other: %0d out of %0d ", accepted_on_other,total_tests);
		
		$display("write on early WLAST: %0d out of %0d ", write_on_early_WLAST,valid_writes);
		$display("write on delayed WLAST: %0d out of %0d ", write_on_late_WLAST,valid_writes);
		$display("write on early WLEN: %0d out of %0d ", write_on_early_WLEN,valid_writes);
		$display("write on delayed WLEN: %0d out of %0d ", write_on_late_WLEN,valid_writes);
		$display("write on other: %0d out of %0d ", write_on_other,total_tests);
	
	endtask
	
	//--------------- R/W Race inspection summary  --------------//
	task print_rw_race_tests_results(input int config_no);	
		$display("-------- @time: %0t , R/W race inspection summary for configuration number: %0d------ ", $time(),config_no);
		
		$display("write first: %0d out of %0d tests ", write_first,rw_race_tests);
		$display("read first: %0d out of %0d tests", read_first,rw_race_tests);
		$display("no specific r/w order: %0d out of %0d tests", no_rw_order_rule,rw_race_tests);
		$display("corrupt reads: %0d out of %0d ", corrupt_read,rw_race_tests);
		
	endtask
	
	//-------------------- Reset test values  -------------------//
	task automatic clear_test_info();
		tests_failed            = 0;
		tests_invalid           = 0;
		tests_passed            = 0;
		
		tests_failed_r          = 0;
		tests_invalid_r         = 0;
		tests_passed_r          = 0;
		total_tests_r           = 0;
		
		tests_failed_w          = 0;
		tests_invalid_w         = 0;
		tests_passed_w          = 0;
		total_tests_w           = 0;
		
		accepted_on_early_WLAST = 0;
		accepted_on_late_WLAST  = 0;
		accepted_on_early_WLEN  = 0;
		accepted_on_late_WLEN   = 0;
		accepted_on_other       = 0;
		failed_to_terminate     = 0;
	
		valid_writes            = 0;
		write_on_early_WLAST    = 0;
		write_on_late_WLAST     = 0;
		write_on_early_WLEN     = 0;
		write_on_late_WLEN      = 0;
		write_on_other          = 0;
		
		write_first             = 0;
		read_first              = 0;
		no_rw_order_rule        = 0;
		rw_race_tests           = 0;
		corrupt_read            = 0;
		
		extra_addr_queue.delete();
		mem_snapshot.delete();
		config_no++;
	endtask

	//------------------------ Run tests ------------------------//
	task automatic run_tests(ref Axi4_packet pkt, int config_no);
		extra_addr_queue.delete();
		mem_snapshot.delete();
		exp_data_queue.delete();
		actual_data_queue.delete();
		golden_model(pkt,exp_RESP);
		drive_stim(pkt,sampled_resp);
		@(negedge axi4_if.ACLK);
		check_results(pkt,exp_RESP,sampled_resp);
	endtask
	
	//----------------------- Enable reset ----------------------//
	task automatic reset();
		axi4_if.ARESETn = 1'b0;
		repeat(2) @(negedge axi4_if.ACLK);
		init();
		axi4_if.ARESETn = 1'b1;
	endtask

	//-------------------- Direct memory write  -----------------//
	task automatic direct_mem_write();
		for(int i = 0;i < MEM_DEPTH; i++) begin
			mem_data                        = $urandom();
			axi4_top.dut.mem_inst.memory[i] = mem_data ;
			expected_mem[i]                 = mem_data ;
		end
	endtask
	
	//-------------------- Initialize signals  ------------------//
	task init();
		 // Write address channel
		axi4_if.AWADDR  = 0;
		axi4_if.AWLEN   = 0;
		axi4_if.AWSIZE  = 3'd2;
		axi4_if.AWVALID = 0;

		// Write data channel
		axi4_if.WDATA   = 0;
		axi4_if.WVALID  = 0;
		axi4_if.WLAST   = 0;

		// Write response channel
		axi4_if.BREADY  = 0;

		// Read address channel
		axi4_if.ARADDR  = 0;
		axi4_if.ARLEN   = 0;
		axi4_if.ARSIZE  = 3'd2;
		axi4_if.ARVALID = 0;

		// Read data channel
		axi4_if.RREADY  = 0;
	endtask
	
	//-------------------- Disable constraints ------------------//
	task automatic disable_conf_constraints(ref Axi4_packet pkt);
		pkt.inbound_burst_c.constraint_mode(0);
		pkt.out_of_bound_burst_c.constraint_mode(0);
		pkt.on_boundary_burst_c.constraint_mode(0);
		pkt.word_size_c.constraint_mode(1);
		pkt.single_beat_c.constraint_mode(0);
		pkt.multi_beat_c.constraint_mode(0);
		pkt.read_xact_c.constraint_mode(0);
		pkt.write_xact_c.constraint_mode(0);
		pkt.addr_inbound_c.constraint_mode(0);
		pkt.out_of_bound_addr_c.constraint_mode(0);
	endtask
	
	
	//----- Determine level of detail printed in testbench  -----//
	function automatic int verb_value(string verb, ref string tb_state);
		int ans;
		if(verb == "summary") begin
			ans = 1;
			tb_state = "SUMMARY";
		end
		else if(verb == "error") begin
			ans = 2;
			tb_state = "DISPLAYING ERRORS ONLY";
		end
		else if(verb == "detailed") begin
			ans = 0;
			tb_state = "DETAILED";
		end
		else begin
			ans = 0;
			tb_state = "DETAILED";
		end
		return ans;	
	endfunction
	
	

endmodule