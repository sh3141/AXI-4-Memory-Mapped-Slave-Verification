if { [info exists 1] } {
    set v_arg $1
} else {
    set v_arg "detailed"
}

vlib work 
vlog axi4_enum_pkg.sv Axi4_packet.sv axi_memory.v axi4.v axi4_sva.sv axi4_if.sv axi4_tb.sv axi4_top.sv 
vlog -cover bcefst +covercells axi4.v  
vsim -voptargs=+acc work.axi4_top +verbosity=$v_arg -cover
coverage save -onexit cov.ucdb

add wave -r *
run -all 

# Exclude Unreachable Write FSM Default States (Lines 206)
coverage exclude -src axi4.v -line 206 -code s
coverage exclude -src axi4.v -line 206 -code b
 
# Exclude Unreachable Read FSM Default States (Lines 275)
coverage exclude -src axi4.v -line 275 -code s
coverage exclude -src axi4.v -line 275 -code b

# Exclude SIZE from toggle coverage
coverage exclude -du axi4 -toggle {AWSIZE}
coverage exclude -du axi4 -toggle {ARSIZE}

# Exclude RESP LSB from toggle coverage
coverage exclude -du axi4 -toggle {BRESP[0]}
coverage exclude -du axi4 -toggle {RRESP[0]}

# Exclude mem_rdata_reg from toggle coverage
coverage exclude -du axi4 -toggle {mem_rdata_reg}

# Exclude addr_incr from toggle coverage
coverage exclude -du axi4 -toggle {write_addr_incr[1]}
coverage exclude -du axi4 -toggle {write_addr_incr[15:3]}
coverage exclude -du axi4 -toggle {read_addr_incr[1]}
coverage exclude -du axi4 -toggle {read_addr_incr[15:3]}

# Exclude internal size bits from toggle coverage
coverage exclude -du axi4 -toggle {write_size[0]}
coverage exclude -du axi4 -toggle {write_size[2]}
coverage exclude -du axi4 -toggle {read_size[0]}
coverage exclude -du axi4 -toggle {read_size[2]}

# Exclude unused FSM state bits from toggle coverage
coverage exclude -du axi4 -toggle {write_state[2]}
coverage exclude -du axi4 -toggle {read_state[2]}

# Exclude impossible scenario of WVALID = 1 && WREADY = 0 in W_DATA phase.
coverage exclude -src axi4.v -feccondrow 168 3

# Exclude impossible scenario of BVALID = 1 && BREADY = 0 in W_RESP phase.
coverage exclude -src axi4.v -feccondrow 199 3

coverage report -details -output cov_report_rst_addr_data_bk2bk_wr_exclu.txt



