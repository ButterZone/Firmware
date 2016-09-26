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
 * flip_state_switch.cpp
 *
 *  Created on: 22Sep.,2016
 *      Author: Zihao
 */

#define FLIP_THR_INC		0.20f	// throttle increase during flip_start
#define FLIP_THR_DEC		0.24f	// throttle decrease during flip_roll
#define FLIP_ROTATION_RATE	400.0f*3.14/180 // 400 deg/sec rotation rate
#define FLIP_RECOVERY_ANGLE 5.0f*3.14/180	// consider succesful recovery when roll i back within 5 degrees


extern "C" __EXPORT int flip_state_switch_main(int argc, char *argv[]);



class FlipStateSwitch
{
public:
	/**
	 * Constructor
	 */
	FlipStateSwitch();

	/**
	 * Destructor, also kills the main task
	 */
	~FlipStateSwitch();

	/**
	 * Start the flip state switch task
	 *
	 * @return OK on success
	 */
	int start();

private:

	bool 	_task_should_exit;		/**< if true, task_main() should exit */
	int 	_control_task; 			/**< task handle */

	/**
	 * Shim for calling task_main from task_create
	 */
	static void task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task
	 */
	void 		task_main();
};

namespace flip_state_switch
{
FlipStateSwitch *g_control;
}

FlipStateSwitch::FlipStateSwitch() :
		_task_should_exit(false),
		_control_task(-1)
{

}

FlipStateSwitch::~FlipStateSwitch()
{
	_task_should_exit = true;

	flip_state_switch::g_control = nullptr;
}

void FlipStateSwitch::task_main_trampoline(int argc, char *argv[])
{
	flip_state_switch::g_control->task_main();
}

void FlipStateSwitch::task_main()
{

}

int FlipStateSwitch::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("flip_state_switch",
						SCHED_DEFAULT,
						SCHED_PRIORITY_MAX - 5,
						1500,
						(px4_main_t)&FlipStateSwitch::task_main_trampoline,
						nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int flip_state_switch_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: flip_state_switch {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (flip_state_switch::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		flip_state_switch::g_control = new FlipStateSwitch;

		if (flip_state_switch::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != flip_state_switch::g_control->start()) {
			delete flip_state_switch::g_control;
			flip_state_switch::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (flip_state_switch::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete flip_state_switch::g_control;
		flip_state_switch::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (flip_state_switch::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");

	return 0;
}
