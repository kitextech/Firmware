
/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hello_example.cpp
 * Example for Linux
 *
 * @author Bertalan Kovács <bertalan@kitex.tech>
 */

#include "adc_monitor.h"
#include <px4_time.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/debug_key_value.h>

extern "C" __EXPORT int adc_monitor_main(int argc, char *argv[]);

namespace adc_monitor
{
AdcMonitor	*adc_m = nullptr;
}

double temp_calc(double voltage)
{
	double T = 22.5 + (voltage - 1.27) * 100.0;

	return T;
}

AdcMonitor::AdcMonitor(){};


AdcMonitor::~AdcMonitor()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			px4_usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	adc_monitor::adc_m = nullptr;
}

int
AdcMonitor::task_main_trampoline(int argc, char *argv[])
{
	adc_monitor::adc_m->task_main();
	return 0;
}

void AdcMonitor::task_main()
{

	int adc_sub_fd = orb_subscribe(ORB_ID(adc_report));
	/* limit the update rate to 5 Hz */
	orb_set_interval(adc_sub_fd, 200);

	struct debug_key_value_s dbg;
	memset(&dbg, 0, sizeof(dbg));
	strncpy(dbg.key, "bcm_temp", 10);
	orb_advert_t dbg_pub_fd = orb_advertise(ORB_ID(debug_key_value), &dbg);

	// /* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = adc_sub_fd;
	fds[0].events = POLLIN;

	int error_counter = 0;

	_task_running = true;

	while (!_task_should_exit) {
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the second file descriptor */
				struct adc_report_s adc;
				orb_copy(ORB_ID(adc_report), adc_sub_fd, &adc);
				double temp = temp_calc((double)adc.channel_value[10]);
				PX4_INFO("Temperature:\t%8.4f", temp);
				dbg.value = temp;
				orb_publish(ORB_ID(debug_key_value), dbg_pub_fd, &dbg);
				usleep(100);

			}
		}
	}

	_control_task = -1;
	_task_running = false;
	orb_unsubscribe(adc_sub_fd);
}

int
AdcMonitor::start()
{
	/* start the task */
	_control_task = px4_task_spawn_cmd("adc_monitor",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&AdcMonitor::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_ERR("task start failed");
		return -errno;
	}

	return PX4_OK;
}


int adc_monitor_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("usage: adc_monitor {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (adc_monitor::adc_m != nullptr) {
			PX4_WARN("already running");
			return 1;
		}

		adc_monitor::adc_m = new AdcMonitor;

		if (adc_monitor::adc_m == nullptr) {
			PX4_ERR("alloc failed");
			return 1;
		}

		if (PX4_OK != adc_monitor::adc_m->start()) {
			delete adc_monitor::adc_m;
			adc_monitor::adc_m = nullptr;
			PX4_ERR("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (adc_monitor::adc_m == nullptr || !adc_monitor::adc_m->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (adc_monitor::adc_m == nullptr || !adc_monitor::adc_m->task_running()) {
				px4_usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (adc_monitor::adc_m == nullptr) {
			PX4_WARN("not running");
			return 1;
		}

		delete adc_monitor::adc_m;
		adc_monitor::adc_m = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (adc_monitor::adc_m) {
			PX4_INFO("running");
			return 0;

		} else {
			PX4_INFO("not running");
			return 1;
		}
	}

	PX4_WARN("unrecognized command");
	return 1;
}
