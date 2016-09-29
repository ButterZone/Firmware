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
 * flip_control.cpp
 *
 *  Created on: 28Sep.,2016
 *      Author: Zihao
 */


extern "C" __EXPORT int flip_control_main(int argc, char *argv[]);

class FlipControl
{
public:
	/**
	 * Constructor
	 */
	FlipControl();

	/**
	 * Destructor, also kills the main task
	 */
	~FlipControl();

	/**
	 * Start the flip state switch task
	 *
	 * @return OK on success
	 */
	int start();

	/**
	 * little function to print current flip state
	 */
	void print_state();

private:
	bool 		_task_should_exit; 		/**< if true, main task should exit */
	int 		_flip_task;				/**< task handle */

	enum FLIP_STATE {
		FLIP_STATE_DISABLED = 0,
		FLIP_STATE_START = 1,
		FLIP_STATE_ROLL = 2,
		FLIP_STATE_RECOVER = 3,
		FLIP_STATE_FINISHED = 4
	}_flip_state;

	/**
	 * Shim for calling task_main from task_create
	 */
	static void task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task
	 */
	void 		task_main();
};

namespace flip_control
{
FlipControl *g_flip;
}

FlipControl::FlipControl() :
		_task_should_exit(false),
		_flip_task(-1),
		_flip_state(FLIP_STATE_DISABLED)
{

}

FlipControl::~FlipControl()
{
	_task_should_exit = true;
	flip_control::g_flip = nullptr;
}

void FlipControl::print_state()
{
	warnx("Current flip state is %d", _flip_state);
}

void FlipControl::task_main_trampoline(int argc, char *argv[])
{
	flip_control::g_flip->task_main();
}

void FlipControl::task_main()
{
	warnx("hello");
}

int FlipControl::start()
{
	ASSERT(_flip_task == -1);

	/*start the task */
	_flip_task = px4_task_spawn_cmd("flip_control",
									SCHED_DEFAULT,
									SCHED_PRIORITY_MAX - 5,
									1500,
									(px4_main_t)&FlipControl::task_main_trampoline,
									nullptr);

	if (_flip_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;

}

int flip_control_main(int argc, char *argv[])
{
	/* warn if no input argument */
	if (argc < 2) {
		warnx("usage: flip_control {start|stop|status}");
		return 1;
	}

	/* start flip_control manually */
	if (!strcmp(argv[1],"start")) {

		if (flip_control::g_flip != nullptr) {
			warnx("already running");
			return 1;
		}

		flip_control::g_flip = new FlipControl;

		if (flip_control::g_flip == nullptr) {
			warnx("allocation failed");
			return 1;
		}

		if (OK != flip_control::g_flip->start()) {
			delete flip_control::g_flip;
			flip_control::g_flip = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	/* stop flip_control manually */
	if (!strcmp(argv[1], "stop")) {
		if (flip_control::g_flip == nullptr) {
			warnx("not running");
			return 1;
		}

		delete flip_control::g_flip;
		flip_control::g_flip = nullptr;
		return 0;
	}

	/* return running status of the application */
	if (!strcmp(argv[1], "status")) {
		if (flip_control::g_flip) {
			warnx("running");
			return 0;
		} else {
			warnx("not running");
			return 1;
		}
	}

	/* print current flip_state */
	if (!strcmp(argv[1], "state")) {
		flip_control::g_flip->print_state();

	}


	/* if argument is not in one of the if statement */
	warnx("unrecognized command");

	return 0;
}
