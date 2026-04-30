`define OKAY   2'b0
`define SLVERR 2'b10

import axi4_enum_pkg::*;

class Axi4_packet#(
	parameter int MEM_ADDR_WIDTH   = 10,
	parameter int DATA_WIDTH       = 32,
	parameter int SLAVE_ADDR_WIDTH = 16
);
	
	localparam int                      ADDR_BOUND   = (1<<(MEM_ADDR_WIDTH + 2)) - 1;
	localparam int                      MEM_DEPTH    = (1<<MEM_ADDR_WIDTH);
	localparam int                      ADDR_BOUND_S = (1<<(SLAVE_ADDR_WIDTH)) - 1;
	localparam logic [DATA_WIDTH - 1:0] ALL_ONES     = (1<<DATA_WIDTH) - 1;

	//--------- ADDRESS SIGNALS -----------//
	rand xact_type_e                  write_en;
	rand logic [SLAVE_ADDR_WIDTH-1:0] addr; 
	rand logic [7:0]                  len; 
	rand logic [2:0]                  size;
	rand xact_burst_bounds_e          burst_bounds;
	rand xact_addr_bounds_e           addr_bounds;
	
	//--------- WRITE DATA SIGNALS --------//

	rand logic [DATA_WIDTH-1:0]       burst_data[];
	rand burst_data_type_e            data_pattern;
	logic [1:0]                       resp;
	
	//----------- CONSTRAINTS  ------------//
	constraint read_xact_c{
		write_en == READ;
	}
	
	constraint write_xact_c{
		write_en == WRITE;
	}
	
	constraint inbound_burst_c{
		burst_bounds == WITHIN_BOUNDS;
	}
	
	constraint out_of_bound_burst_c{
		burst_bounds == EXCEED_BOUNDS;
	}
	
	constraint on_boundary_burst_c{
		burst_bounds == ON_BOUNDARY;
	}
	
	constraint addr_inbound_c{
		addr_bounds == ADDR_WITHIN_BOUNDS;
	}
	
	constraint out_of_bound_addr_c{
		addr_bounds == ADDR_EXCEED_BOUNDS;
	}
	
	constraint word_size_c{
		size == 3'd2;
	}
	
	constraint bound_burst_c{
		burst_bounds dist {WITHIN_BOUNDS:=55, ON_BOUNDARY:=25, EXCEED_BOUNDS:=20};
	}
	
	constraint bound_addr_c{
		addr_bounds dist {ADDR_WITHIN_BOUNDS:=75, ADDR_EXCEED_BOUNDS:=25};
	}
	
	constraint burst_size_c{
		burst_data.size() == (len + 1);
	}
	
	constraint single_beat_c{
		len == 0;
	}
	
	constraint multi_beat_c{
		len > 0;
	}
	
	constraint len_c{
		len dist {
			0        :/ 20,
			[1:31]   :/ 60,
			[32:254] :/ 10,
			255      :/ 10
		};
	}
	constraint data_pattern_for_bursts_c{
		if(len >7){
			data_pattern != DATA_ZEROS;
			data_pattern != DATA_ONES;
			data_pattern != DATA_CHECKERBOARD;
		}
	}

	constraint addr_c{
		//physical memory bound: start of burst
		if(addr_bounds == ADDR_WITHIN_BOUNDS){
			addr <= ADDR_BOUND;
		}
		else{
			addr > ADDR_BOUND;
		}
		
		//reconstraint burst to determine if length of burst will lead to 4KB boundary crossing violation
		if(burst_bounds == WITHIN_BOUNDS){
			((addr&ADDR_BOUND) + ((len)<<size)) < ADDR_BOUND;		
		}
		else if(burst_bounds == ON_BOUNDARY){
			((addr&ADDR_BOUND) + ((len)<<size)) == ADDR_BOUND;
		}
		else{
			if(len > 0){
				((addr&ADDR_BOUND) + ((len)<<size)) > ADDR_BOUND;
			}
			
		}
	}
	
	constraint data_pattern_c{
		data_pattern dist{
			DATA_RANDOM       := 50,
			DATA_INC          := 30,
			DATA_CHECKERBOARD := 10,
			DATA_ZEROS        := 5,
			DATA_ONES         := 5
		};
	}
	
	constraint burst_data_c{
		if(data_pattern == DATA_ZEROS || data_pattern == DATA_ONES){
			foreach(burst_data[i]) burst_data[i] == ((data_pattern == DATA_ONES)? ALL_ONES:0);
		}
		else if(data_pattern == DATA_CHECKERBOARD){
			foreach(burst_data[i]) burst_data[i] inside {32'hAAAA_AAAA, 32'h5555_5555};
		}
		else if(data_pattern == DATA_INC){
			foreach(burst_data[i]){
				if(i > 0) burst_data[i] == burst_data[i-1] + 1;
			}
		}
	}
	//------------ COVER GROUPS -----------//
	covergroup axi4_cov;
		xact_type_cp: coverpoint write_en{
			bins read_op  = {READ};
			bins write_op = {WRITE}; 
		}
		
		burst_len_cp: coverpoint len{
			bins single_beat      = {0};
			bins very_short_burst = {[1:7]};
			bins short_burst      = {[8:31]};
			bins medium_burst     = {[32:127]};
			bins long_burst       = {[128:254]};
			bins max_burst        = {255};
		}
		
		burst_type_cp: coverpoint burst_bounds{
			bins burst_inbound      = {WITHIN_BOUNDS};
			bins burst_on_boundary  = {ON_BOUNDARY};
			bins burst_out_of_bound = {EXCEED_BOUNDS};
		}
		
		addr_range_cp: coverpoint addr_bounds{
			bins addr_inbound      = {ADDR_WITHIN_BOUNDS};
			bins addr_out_of_bound = {ADDR_EXCEED_BOUNDS};
		}
		
		addr_cp: coverpoint addr{
			bins low_range    = {[0:1023]};
			bins med_range    = {[1024:2047]};
			bins high_range   = {[2048:4095]};
			bins out_of_range = {[4096:65535]};
		}
		
		data_pattern_cp: coverpoint data_pattern{
			bins random_data = {DATA_RANDOM};
			bins data_inc    = {DATA_INC};
			bins data_zeros  = {DATA_ZEROS};
			bins data_ones   = {DATA_ONES};
			bins data_alt    = {DATA_CHECKERBOARD};
		}
		
		op_len_cross         : cross xact_type_cp, burst_len_cp;
		op_burst_bound_cross : cross xact_type_cp, burst_type_cp;
		op_addr_cross        : cross xact_type_cp, addr_cp;
		addr_len_cross       : cross addr_cp, burst_len_cp;	
		op_data_pattern      : cross xact_type_cp, data_pattern_cp;
	
		len_burst_bound_cross: cross burst_len_cp, burst_type_cp{
			ignore_bins ignore_single_beat_out_of_bounds = binsof(burst_len_cp.single_beat) && binsof(burst_type_cp.burst_out_of_bound);
		}
		
		data_len_cross       : cross burst_len_cp, data_pattern_cp{
			option.cross_auto_bin_max = 0;
			bins rand_all_len     = binsof(data_pattern_cp.random_data);
			bins inc_multi_beat   = binsof(data_pattern_cp.data_inc) && !binsof(burst_len_cp.single_beat);
			bins alt_short_beat   = binsof(data_pattern_cp.data_alt) && (binsof(burst_len_cp.single_beat) || binsof(burst_len_cp.very_short_burst));
			bins zeros_short_beat = binsof(data_pattern_cp.data_zeros) && (binsof(burst_len_cp.single_beat) || binsof(burst_len_cp.very_short_burst));
			bins ones_short_beat  = binsof(data_pattern_cp.data_ones) && (binsof(burst_len_cp.single_beat) || binsof(burst_len_cp.very_short_burst));
		}
		
		addr_burst_bound_cross : cross addr_range_cp, burst_type_cp;
		op_addr_bound_cross    : cross xact_type_cp, addr_range_cp;
		len_addr_bound_cross   : cross burst_len_cp, addr_range_cp;
		
	endgroup 
	
	covergroup burst_data_cov with function sample(logic [DATA_WIDTH - 1:0] data);
		data_cp: coverpoint data{
			bins all_zeros      = {0};
			bins all_ones       = {ALL_ONES};
			bins checkerboard_a = {32'hAAAA_AAAA};
			bins checkerboard_5 = {32'h5555_5555};
			bins low_range      = {[32'h0000_0001:32'h0FFF_FFFF]};
			bins mid_range      = {[32'h1000_0000:32'h7FFF_FFFF]};
			bins high_range     = {[32'h8000_0000:32'hFFFF_FFFE]};
		}
		
	endgroup
	//------- TASKS AND FUNCTIONS  --------//
	function new();
		axi4_cov       = new();
		burst_data_cov = new();
	endfunction
	
	function void burst_sample();
		foreach(burst_data[i]) begin
			burst_data_cov.sample(burst_data[i]);
		end
	endfunction
	
	function void display();
		$display("Sending xact >>> [%s] ADDR = %0d, ADDR WORD = %0d, LEN = %0d",write_en.name(),addr, addr>>2,len);
	endfunction 
	
endclass