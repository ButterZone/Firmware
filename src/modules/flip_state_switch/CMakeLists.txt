px4_add_module(
	MODULE modules__flip_state_switch
	MAIN flip_state_switch
	COMPILE_FLAGS -Os
	STACK_MAIN 1200
	SRCS
		flip_state_switch.cpp
	DEPENDS
		platforms__common
	)