/*
 * flip_state_swtich.cpp
 *
 *  Created on: 22Sep.,2016
 *      Author: root
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>


__EXPORT int flip_state_switch_main(int argc, char *argv[]);

int flip_state_switch_main(int argc, char *argv[])
{x
	/* advertise vehicle_status topic */

	/* subscribe to vehicle_control_mode topic */

	/* advertise vehicle_control_mode topic */

	/* wait for topics */

	/* copy vehicle_status topic */

	/* copy vehicle_control_mode topic */

	/* do mode switch while flip_mode is enabled */

	return 0;
}
