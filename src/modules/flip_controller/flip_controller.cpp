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

class FlipController
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
	bool 		_task_should_exit; 		/**< if true, main task should exit */
	int 		_flip_task;				/**< task handle */

	/**
	 * Shim for calling task_main from task_create
	 */
	static void task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task
	 */
	void 		task_main();
};

namespace flip_controller
{
FlipController *g_flip;
}

FlipController::FlipController() :
		_task_should_exit(false),
		_flip_task(-1)
{

}

FlipController::~FlipController()
{
	_task_should_exit = true;
	flip_controller::g_flip = nullptr;
}

int FlipController::start()
{
	ASSERT(_flip_task == -1);

	/*start the task */
	_flip_task = px4_task_spawn_cmd("flip_controller",
									SCHED_DEFAULT,
									SCHED_PRIORITY_MAX - 5,
									1500,
									(px4_main_t)&FlipController::task_main_trampoline,
									nullptr);

	if (_flip_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;

}

int flip_controller_main(int argc, char *argv[])
{
	/* warn if no input argument */
	if (argc < 2) {
		warnx("usage: flip_controller {start|stop|status}");
		return 1;
	}

	/* start flip_controller manually */
	if (!strcmp(argv[1],"start")) {

		if (flip_controller::g_flip != nullptr) {
			warnx("already running");
			return 1;
		}

		flip_controller::g_flip == new FlipController;

		if (flip_controller::g_flip == nullptr) {
			warnx("allocation failed");
			return 1;
		}

		if (OK != flip_controller::g_flip->start()) {
			delete flip_controller::g_flip;
			flip_controller::g_flip = nullptr;
			warnx("start failed");
			return 1;
		}
	}

	/* stop flip_controller manually */
	if (!strcmp(argv[1], "stop")) {
		if (flip_controller::g_flip == nullptr) {
			warnx("not running");
			return 1;
		}

		delete flip_controller::g_flip;
		flip_controller::g_flip = nullptr;
		return 0;
	}

	/* return running status of the application */
	if (!strcmp(argv[1], "status")) {
		if (flip_controller::g_flip) {
			warnx("running");
			return 0;
		} else {
			warnx("not running");
			return 1;
		}
	}


	/* if argument is not in one of the if statement */
	warnx("unrecognized command");

	return 0;
}
