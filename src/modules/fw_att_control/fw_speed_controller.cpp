/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 KiteX Development Team. All rights reserved.
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


#include "fw_speed_controller.hpp"


/**
 * @class SpeedController
 * Creates a speed controller to control the thrust while in fixed wing mode.
 */
SpeedController::SpeedController():
_last_run(0),
_k_p(0.0f),
_k_i(0.0f),
_k_ff(0.0f),
_integrator_max(0.0f),
_last_output(0.0f),
_integrator(0.0f),
_min_throttle(0.3f),
_target_speed(22.0f)
{}

float SpeedController::control_thrust(float speeed) {
  // do control

  /* get the usual dt estimate */
  uint64_t dt_micros = hrt_elapsed_time(&_last_run);
  _last_run = hrt_absolute_time();
  float dt = (float)dt_micros * 1e-6f;

  /* lock integral for long intervals */
  bool lock_integrator = false;

  if (dt_micros > 500000) {
    lock_integrator = true;
  }

  float error = _target_speed - speeed;

	if (!lock_integrator && _k_i > 0.0f) {

		float id = error * dt;

		/*
		 * anti-windup: do not allow integrator to increase if actuator is at limit
		 */
		if (_last_output < _min_throttle) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		_integrator += id * _k_i;
	}

	/* integrator limit */
	//xxx: until start detection is available: integral part in control signal is limited here
	float integrator_constrained = math::constrain(_integrator, -_integrator_max, _integrator_max);

	/* Apply PI rate controller and store non-limited output */
	_last_output = _target_speed * _k_ff +
		       error * _k_p
		       + integrator_constrained;  //scaler is proportional to 1/airspeed

	return math::constrain(_last_output, _min_throttle, 1.0f);
}


void SpeedController::set_k_p(float k_p) {
	_k_p = k_p;
}

void SpeedController::set_k_i(float k_i) {
	_k_i = k_i;
}

void SpeedController::set_k_ff(float k_ff) {
	_k_ff = k_ff;
}

void SpeedController::set_integrator_max(float max){
	_integrator_max = max;
}

void SpeedController::set_min_throttle(float min_throttle){
	_min_throttle = min_throttle;
}

void SpeedController::reset_integrator() {
  _integrator = 0.0f;
}

void SpeedController::set_target_speed(float speed) {
  _target_speed = speed;
}
