/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
* @file tailsitter.cpp
*
* @author Andreas Okholm   <bapstroman@gmail.com>
*
*/

#include "kite.h"
#include "vtol_att_control_main.h"

Kite::Kite(VtolAttitudeControl *attc) :
	VtolType(attc),
	_yaw_transition_start(0.0f),
	_roll_transition_start(0.0f),
	_pitch_transition_start(0.0f),
	_thrust_transition_start(0.0f),
	_speed(0.0f),
	_airspeed_ratio(0.0f)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_params_handles_kite.airspeed_trans = param_find("VT_ARSP_TRANS");
	_params_handles_kite.trans_forward_roll = param_find("VT_T_F_ROLL");
	_params_handles_kite.trans_forward_thrust = param_find("VT_T_F_THRUST");
	_params_handles_kite.trans_forward_pitch = param_find("VT_T_F_PITCH");
	_params_handles_kite.trans_forward_duration_max = param_find("VT_T_F_DUR_MAX"); // not currently in use
	_params_handles_kite.trans_backwards_pitch = param_find("VT_T_B_PITCH");
	_params_handles_kite.trans_backwards_roll = param_find("VT_T_B_ROLL");
	_params_handles_kite.trans_backwards_thrust = param_find("VT_T_B_THRUST");
	_params_handles_kite.wind_speed = param_find("VT_WIND_SPEED");
}

Kite::~Kite()
{

}

void
Kite::parameters_update()
{
	float f;

	param_get(_params_handles_kite.airspeed_trans, &f);
	_params_kite.airspeed_trans = f;

	param_get(_params_handles_kite.trans_forward_roll, &f);
	_params_kite.trans_forward_roll = f;

	param_get(_params_handles_kite.trans_forward_pitch, &f);
	_params_kite.trans_forward_pitch = f;

	param_get(_params_handles_kite.trans_forward_thrust, &f);
	_params_kite.trans_forward_thrust = f;

	param_get(_params_handles_kite.trans_forward_duration_max, &f);
	_params_kite.trans_forward_duration_max = f;

	param_get(_params_handles_kite.trans_backwards_pitch, &f);
	_params_kite.trans_backwards_pitch = f;

	param_get(_params_handles_kite.trans_backwards_roll, &f);
	_params_kite.trans_backwards_roll = f;

	param_get(_params_handles_kite.trans_backwards_thrust, &f);
	_params_kite.trans_backwards_thrust = f;

	param_get(_params_handles_kite.wind_speed, &f);
	_params_kite.wind_speed = f;
}

void Kite::update_vtol_state()
{

	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start a negative roll in MC control mode, picking up
	 * speed. After the vehicle has picked up sufficient speed the uav will go into FW mode.
	 * For the backtransition the uav is rolled upwards and pitched to a very high angle of attack to stall the main wing.
	 * When the speed is reduced sufficiently and the roll is low enough the MC mode takes over again.
	*/

	// matrix::Eulerf euler = matrix::Quatf(_v_att->q);
	// float roll = euler.phi();
	// float pitch = euler.theta();

	// update velocity
	_speed = sqrtf(
		_local_pos->vx*_local_pos->vx +
		_local_pos->vy*_local_pos->vy +
		_local_pos->vz*_local_pos->vz );

	_airspeed_ratio = _speed/_params_kite.airspeed_trans; // use _airspeed->indicated_airspeed_m_s

	// SAFETY SWITCH go directly to MC mode
	if (_manual_control_sp->aux2 > 0.5f) {
		_vtol_schedule.flight_mode = MC_MODE;


	} else if (_attc->is_fixed_wing_requested()) { // switchig to FW mode
		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			// initialise a front transition
			_vtol_schedule.flight_mode 	= TRANSITION_FRONT;
			set_transition_starting_values(true);
			break;

		case FW_MODE:
			break;

		case TRANSITION_FRONT:
				// transition_ratio for ground testing.
				update_transition_ratio();
				if (_airspeed_ratio >= 1.0f) {
					_vtol_schedule.flight_mode = FW_MODE;
				}
				else if (_transition_ratio >= 2.0f) {
					_vtol_schedule.flight_mode = FW_MODE; // FIXME: For testing
					// _vtol_schedule.flight_mode = TRANSITION_BACK;
				}
			break;

		case TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = FW_MODE;
			break;
		}
	} else {  // Switchig to MC mode
		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			break;

		case FW_MODE:
			_vtol_schedule.flight_mode = TRANSITION_BACK;
			set_transition_starting_values(false);
			break;

		case TRANSITION_FRONT:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = MC_MODE;
			break;

		case TRANSITION_BACK:
			update_transition_ratio();
			// check if we have reached the roll angle to switch to MC mode
			if (_airspeed_ratio <= 0.5f || _transition_ratio >= 1.0f) {
				_vtol_schedule.flight_mode = MC_MODE;
			}

			break;
		}
	}
	update_external_VTOL_state();
}

void Kite::update_external_VTOL_state()
{
	// map kite specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		_vtol_mode = ROTARY_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		break;

	case FW_MODE:
		_vtol_mode = FIXED_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		break;

	case TRANSITION_FRONT:
		_vtol_mode = TRANSITION_TO_FW;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;

	case TRANSITION_BACK:
		_vtol_mode = TRANSITION_TO_MC;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;
	}
}

void Kite::update_transition_state()
{
	float t = math::constrain(_transition_ratio, 0.0f, 1.0f);

	if (_vtol_schedule.flight_mode == TRANSITION_FRONT) {
		/** create time dependant pitch angle set point + 0.2 rad overlap over the switch value*/
		_v_att_sp->roll_body = (1.0f - t)*_roll_transition_start + t*_params_kite.trans_forward_roll;
		_v_att_sp->pitch_body = (1.0f - t)*_pitch_transition_start + t*_params_kite.trans_forward_pitch;
		_v_att_sp->thrust = (1.0f - t)*_thrust_transition_start + t*_params_kite.trans_forward_thrust;

	}
	else if (_vtol_schedule.flight_mode == TRANSITION_BACK) {

		if (!flag_idle_mc) {
			set_idle_mc();
			flag_idle_mc = true;
		}

		/** create time dependant pitch angle set point + 0.2 rad overlap over the switch value*/
		_v_att_sp->roll_body = (1.0f - t)*_roll_transition_start + t*_params_kite.trans_backwards_roll;
		_v_att_sp->pitch_body = (1.0f - t)*_pitch_transition_start + t*_params_kite.trans_backwards_pitch;
		_v_att_sp->thrust = (1.0f - t)*_thrust_transition_start + t*_params_kite.trans_backwards_thrust;
	}

	/** smoothly move control weight to MC */
	_mc_roll_weight = math::constrain(1.0f - _airspeed_ratio, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(1.0f - _airspeed_ratio, 0.0f, 1.0f);
	_mc_pitch_weight = math::constrain(1.0f - _airspeed_ratio, 0.0f, 1.0f);

	// compute desired attitude and thrust setpoint for the transition

	_v_att_sp->timestamp = hrt_absolute_time();
	_v_att_sp->yaw_body = _yaw_transition_start;

	math::Quaternion q_sp;
	q_sp.from_euler(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body);
	memcpy(&_v_att_sp->q_d[0], &q_sp.data[0], sizeof(_v_att_sp->q_d));
}

void Kite::waiting_on_tecs()
{
	// TODO check -- don't do anything
}

void Kite::set_transition_starting_values(bool forward)
{
	_vtol_schedule.transition_start = hrt_absolute_time();
	_transition_ratio = 0.0f;
	matrix::Eulerf euler = matrix::Quatf(_v_att->q);
	// phi, theta, psi

	_yaw_transition_start = euler.psi();

	_thrust_transition_start = _mc_virtual_v_rates_sp->thrust;
	_pitch_transition_start = euler.theta();
	_roll_transition_start = euler.phi();
}

void Kite::update_transition_ratio()
{
	_transition_ratio = (float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params->front_trans_time_min * 1000000.0f);
}

void Kite::update_mc_state()
{
	VtolType::update_mc_state();

	// SAFETY SWITCH
	if (_manual_control_sp->aux2 > 0.5f) {

		_v_att_sp->roll_body = 0;
		_v_att_sp->pitch_body = 0;
		_v_att_sp->yaw_body = _yaw_transition_start;
		_v_att_sp->thrust = 0.6;

		math::Quaternion q_sp;
		q_sp.from_euler(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body);
		memcpy(&_v_att_sp->q_d[0], &q_sp.data[0], sizeof(_v_att_sp->q_d));
	}

	// set idle speed for rotary wing mode
	if (!flag_idle_mc) {
		set_idle_mc();
		flag_idle_mc = true;
	}
}

void Kite::update_fw_state()
{
	VtolType::update_fw_state();

	if (flag_idle_mc) {
		set_idle_fw();
		flag_idle_mc = false;
	}
}

float Kite::elevator_correction()
{
	return  1.7f * (1.0f - math::constrain(_airspeed_ratio, 0.0f, 1.0f)); // simplifed
}

/**
* Write data to actuator output topic.
*/
void Kite::fill_actuator_outputs()
{
	// multirotor controls
	_actuators_out_0->timestamp = _actuators_mc_in->timestamp;

	// SAFETY SWITCH
	if (_manual_control_sp->aux2 > 0.5f) {
		// roll
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_ROLL];
		// pitch
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];
		// yaw
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_YAW];
		// throttle
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

		// pitch
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = -1;
		// roll
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = 0;
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0.0f; 		// yaw - not in use
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] = 0.0f; 		// throttle - not in use

		return;
	}

	if (_vtol_schedule.flight_mode == FW_MODE) {
		// roll
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = 0;
		// pitch
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] = 0;
		// yaw
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = 0;
		// throttle
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];
	}
	else {
		// roll
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] * _mc_roll_weight;
		// pitch
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
		// yaw
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = // NOT USING ROLL WEIGHT
			_actuators_mc_in->control[actuator_controls_s::INDEX_YAW] * _mc_roll_weight; //* _mc_yaw_weight * 0.3f;
		// throttle
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];
	}

	// Fixed wing controls
	_actuators_out_1->timestamp = _actuators_fw_in->timestamp;

	if (_vtol_schedule.flight_mode == FW_MODE) {
		// pitch
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = 0.7f - elevator_correction()
			+ 0.5f * _actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];

		// roll
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];

		// yaw - not in use
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0.0f;

		// throttle - not in use
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
	}
	else {
		// pitch
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = 0.7f - elevator_correction()
			+ 0.5f * _actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];

		// roll
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL] * (1 - _mc_roll_weight);

		// yaw - not in use
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0.0f;

		// throttle - not in use
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
	}
}
