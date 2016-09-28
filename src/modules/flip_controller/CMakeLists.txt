px4_add_module(
	MODULE modules__flip_controller
	MAIN flip_controller
	COMPILE_FLAGS -Os
	STACK_MAIN 1200
	SRCS
		flip_controller.cpp
	DEPENDS
		platforms__common
	)