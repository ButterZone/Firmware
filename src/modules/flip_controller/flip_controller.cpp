#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
/*
 * flip_controller.cpp
 *
 *  Created on: 28Sep.,2016
 *      Author: Zihao
 */


extern "C" __EXPORT int flip_controller(int argc, char *argv[]);

class flip_controller
{
public:
	/**
	 * Constructor
	 */
	FlipController();

	/**
	 * Destructor, also kills the main task
	 */
	~FlipController();

	/**
	 * Start the flip state switch task
	 *
	 * @return OK on success
	 */
	int start();

private:

	/**
	 * Shim for calling task_main from task_create
	 */
	static void task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controle task
	 */
	void 		task_main();
};
