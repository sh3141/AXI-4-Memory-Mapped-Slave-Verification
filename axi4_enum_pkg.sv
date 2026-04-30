package axi4_enum_pkg;

	typedef enum bit{
		READ  = 0,
		WRITE = 1
	} xact_type_e;
	
	typedef enum logic [1:0]{
		WITHIN_BOUNDS,
		EXCEED_BOUNDS,
		ON_BOUNDARY
	} xact_burst_bounds_e;
	
	typedef enum logic{
		ADDR_WITHIN_BOUNDS,
		ADDR_EXCEED_BOUNDS
	} xact_addr_bounds_e;
	
	typedef enum logic [2:0]{
		DATA_INC,
		DATA_RANDOM,
		DATA_ZEROS,
		DATA_ONES,
		DATA_CHECKERBOARD
	} burst_data_type_e;
	
	typedef enum int{
		EARLY_WLAST,
		CORRECT_WLAST,
		DELAYED_WLAST
	} wlast_err_e;
	


endpackage 