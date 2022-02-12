/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file RateControl.cpp
 */

#include <RateControl.hpp>
#include <px4_platform_common/defines.h>

// #define __float_f %d.%.6d
#define __value_f(num) (int)num, (int)((num-(int)num)*1000000)

using namespace matrix;


void RateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RateControl::setSaturationStatus(const MultirotorMixer::saturation_status &status)
{
	_mixer_saturation_positive[0] = status.flags.roll_pos;
	_mixer_saturation_positive[1] = status.flags.pitch_pos;
	_mixer_saturation_positive[2] = status.flags.yaw_pos;
	_mixer_saturation_negative[0] = status.flags.roll_neg;
	_mixer_saturation_negative[1] = status.flags.pitch_neg;
	_mixer_saturation_negative[2] = status.flags.yaw_neg;
}

Vector3f RateControl::update(const Vector3f &rate,
			     const Vector3f &rate_sp,
			     const Vector3f &angular_accel,
			     const float dt,
			     const bool landed){

	// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
	vehicle_smc_gains_s smc_gains{};
	_vehicle_smc_gains_sub.copy(&smc_gains);

	// define lambda, K and beta
	const Vector3f _gain_lambda(20.0f, 20.0f, 10.0f);
	const Vector3f _gain_K(0.23f, 0.23f, 0.1f);
	const Vector3f _gain_beta(0.05f, 0.05f, 0.1f);

	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// sliding gain
	// Vector3f sliding_gain = tanh_v(rate_error);

	// PID control with feed forward
	const Vector3f torque_old = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// SMC with tanh
	const Vector3f torque_new = _gain_K.emult(tanh_v(_gain_beta.emult(_gain_lambda.emult(rate_error) - angular_accel)));

	// PX4_INFO("###\nError: [%d.%.6d, %d.%.6d, %d.%.6d]\nAccel: [%d.%.6d, %d.%.6d, %d.%.6d]\nOld: [%d.%.6d, %d.%.6d, %d.%.6d]\nNew: [%d.%.6d, %d.%.6d, %d.%.6d]",
	// 		__value_f(rate_error(0)),
	// 		__value_f(rate_error(1)),
	// 		__value_f(rate_error(2)),
	// 		__value_f(angular_accel(0)),
	// 		__value_f(angular_accel(1)),
	// 		__value_f(angular_accel(2)),
	// 		__value_f(torque_old(0)),
	// 		__value_f(torque_old(1)),
	// 		__value_f(torque_old(2)),
	// 		__value_f(torque_new(0)),
	// 		__value_f(torque_new(1)),
	// 		__value_f(torque_new(2))
	// 	);

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque_old;
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_mixer_saturation_positive[i]) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_mixer_saturation_negative[i]) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}

// void RateControl::getSMCParams(vehicle_smc_gains_s &rate_smc_params_s)
// {
// 	rate_smc_params.rollspeed_integ = _rate_int(0);
// 	rate_smc_params.pitchspeed_integ = _rate_int(1);
// 	rate_smc_params.yawspeed_integ = _rate_int(2);
// }

Vector3f RateControl::signale(const Vector3f &vector)
{
	Vector3f result_signale(0, 0, 0);
	for (int i = 0; i < 3; i++){
		result_signale(i) = (vector(i)>=0.f) ? 1.f : -1.f;
	}
	return result_signale;
}


Vector3f RateControl::tanh_v(const Vector3f &vector)
{
	// const Vector3f dummy_vec(1, 5, 10);
	Vector3f result_signale(0, 0, 0);
	for (int i = 0; i < 3; i++){
		// result_signale(i) = std::tanh(dummy_vec(i));
		result_signale(i) = std::tanh(vector(i));
	}

	// PX4_INFO("###\nOriginal: [%d.%.6d, %d.%.6d, %d.%.6d]\nModified: [%d.%.6d, %d.%.6d, %d.%.6d]",
	// 		__value_f(vector(0)),
	// 		__value_f(vector(1)),
	// 		__value_f(vector(2)),
	// 		__value_f(result_signale(0)),
	// 		__value_f(result_signale(1)),
	// 		__value_f(result_signale(2))
	// 	);

	return result_signale;
}
