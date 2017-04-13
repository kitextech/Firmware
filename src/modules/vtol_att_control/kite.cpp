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
	_thrust_transition_start(0.0f)

{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_params_handles_kite.airspeed_trans = param_find("VT_ARSP_TRANS");
	_params_handles_kite.trans_forward_roll = param_find("VT_T_F_ROLL");
	_params_handles_kite.trans_forward_thrust = param_find("VT_T_F_THRUST");
	_params_handles_kite.trans_backwards_pitch = param_find("VT_T_B_PITCH");
	_params_handles_kite.trans_backwards_roll = param_find("VT_T_B_ROLL");
	_params_handles_kite.trans_backwards_thrust = param_find("VT_T_B_THRUST");
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

	param_get(_params_handles_kite.trans_forward_thrust, &f);
	_params_kite.trans_forward_thrust = f;

	param_get(_params_handles_kite.trans_backwards_pitch, &f);
	_params_kite.trans_backwards_pitch = f;

	param_get(_params_handles_kite.trans_backwards_roll, &f);
	_params_kite.trans_backwards_roll = f;

	param_get(_params_handles_kite.trans_backwards_thrust, &f);
	_params_kite.trans_backwards_thrust = f;
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

	if (_attc->is_fixed_wing_requested()) { // switchig to FW mode
		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			// initialise a front transition
			_vtol_schedule.flight_mode 	= TRANSITION_FRONT;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case FW_MODE:
			break;

		case TRANSITION_FRONT:
			// check if we have reached sufficient airspeed
			if ((_airspeed->indicated_airspeed_m_s >= _params_kite.airspeed_trans) || can_transition_on_ground()) {
				_vtol_schedule.flight_mode = FW_MODE;
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
			_vtol_schedule.flight_mode 	= TRANSITION_BACK;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case TRANSITION_FRONT:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = MC_MODE;
			break;

		case TRANSITION_BACK:

			// check if we have reached the roll angle to switch to MC mode
			if (_airspeed->indicated_airspeed_m_s >= 8) { // TODO speed
				_vtol_schedule.flight_mode = MC_MODE;
			}

			break;
		}
	}

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
	if (_vtol_schedule.flight_mode == TRANSITION_FRONT) {

		float transition_ratio = (float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params->front_trans_time_min * 1000000.0f);

		if ( transition_ratio < 1 ) {
			/** create time dependant pitch angle set point + 0.2 rad overlap over the switch value*/
			_v_att_sp->roll_body = _roll_transition_start + ( _params_kite.trans_forward_roll - _roll_transition_start) * transition_ratio;
			_v_att_sp->thrust = _thrust_transition_start + ( _params_kite.trans_forward_thrust - _thrust_transition_start) * transition_ratio;
		} else {
			_v_att_sp->roll_body = _params_kite.trans_forward_roll;
			_v_att_sp->thrust = _params_kite.trans_forward_thrust;
		}

		_v_att_sp->pitch_body = _pitch_transition_start;

	} else if (_vtol_schedule.flight_mode == TRANSITION_BACK) {

		if (!flag_idle_mc) {
			set_idle_mc();
			flag_idle_mc = true;
		}

		float transition_ratio = (float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params->front_trans_time_min * 1000000.0f);

		if ( transition_ratio < 1 ) {
			/** create time dependant pitch angle set point + 0.2 rad overlap over the switch value*/
			_v_att_sp->pitch_body = _pitch_transition_start + ( _params_kite.trans_backwards_pitch - _pitch_transition_start) * transition_ratio;
			_v_att_sp->roll_body = _roll_transition_start + ( _params_kite.trans_backwards_roll - _roll_transition_start) * transition_ratio;
			_v_att_sp->thrust = _thrust_transition_start + ( _params_kite.trans_backwards_thrust - _thrust_transition_start) * transition_ratio;
		} else {
			_v_att_sp->pitch_body = _params_kite.trans_backwards_pitch;
			_v_att_sp->roll_body = _params_kite.trans_backwards_roll;
			_v_att_sp->thrust = _params_kite.trans_backwards_thrust;
		}


	}

	float airspeed_ratio = _airspeed->indicated_airspeed_m_s/_params_kite.airspeed_trans;
	/** smoothly move control weight to MC */
	_mc_roll_weight = 1.0f - airspeed_ratio;
	_mc_pitch_weight = 1.0f - airspeed_ratio;
	_mc_yaw_weight = 1.0f - airspeed_ratio;

	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);
	_mc_pitch_weight = math::constrain(_mc_pitch_weight, 0.0f, 1.0f);

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

	// copy the last trust value from the front transition
	//_v_att_sp->thrust = _thrust_transition;
}

void Kite::set_transition_starting_values()
{
	_yaw_transition_start = _v_att_sp->yaw_body;
	_thrust_transition_start = _v_att_sp->thrust;
	_pitch_transition_start = _v_att_sp->pitch_body;
	_roll_transition_start = _v_att_sp->roll_body;
}

void Kite::update_mc_state()
{
	VtolType::update_mc_state();

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

/**
* Write data to actuator output topic.
*/
void Kite::fill_actuator_outputs()
{
	// multirotor controls
	_actuators_out_0->timestamp = _actuators_mc_in->timestamp;

	// roll
	_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] * _mc_roll_weight;
	// pitch
	_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
	// yaw
	_actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_YAW] * _mc_yaw_weight;
	// throttle
	_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];


	// fixed wing controls
	_actuators_out_1->timestamp = _actuators_fw_in->timestamp;

	//roll
	_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
		-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL] * (1 - _mc_roll_weight);
	//pitch
	if (_vtol_mode != FIXED_WING) {
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = (1 - _mc_pitch_weight); // TODO temporary
	} else {
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			(_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim) * (1 - _mc_pitch_weight);
	}

	// yaw
	_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0; // we don't have ailerons

	// throttle
	_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] =
		_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

}
