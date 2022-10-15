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

Vector3f RateControl::update(
	const Vector3f &rate,
	const Vector3f &rate_sp,
	const Vector3f &angular_accel,
	const float dt,
	const bool landed)
{

	// define gains P, I, D and FF
	// const Vector3f _gain_p(0.23f, 0.23f, 0.1f);
	// const Vector3f _gain_i(0.23f, 0.23f, 0.1f);
	// const Vector3f _gain_d(0.23f, 0.23f, 0.1f);
	// const Vector3f _gain_ff(0.23f, 0.23f, 0.1f);

	// define gains Kappa, Alpha, Beta and Gamma
	const Vector3f _gain_kappa(0.23f, 0.23f, 0.1f);
	const Vector3f _gain_alpha(1.0f, 1.0f, 2.0f);
	const Vector3f _gain_beta(1.0f, 1.0f, 1.0f);
	const Vector3f _gain_gamma(0.05f, 0.05f, 0.1f);

	// angular rates error
	Vector3f rate_error = rate_sp - rate;


	// PID control with feed forward
	// const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// SMC with tanh (PD surface)
	// const Vector3f torque = _gain_kappa.emult(tanh_v(
	// 	_gain_alpha.emult(rate_error) - _gain_gamma.emult(angular_accel)
	// ));

	// SMC with tanh (PID surface)
	const Vector3f torque = _gain_kappa.emult(tanh_v(
		_gain_alpha.emult(rate_error) + _gain_beta.emult(_rate_int).edivide(_gain_i) - _gain_gamma.emult(angular_accel)
	));
	

	PX4_INFO("###\nError: [%d.%.6d, %d.%.6d, %d.%.6d]\nErrorIntegral: [%d.%.6d, %d.%.6d, %d.%.6d]\nAccel: [%d.%.6d, %d.%.6d, %d.%.6d]\nTorque: [%d.%.6d, %d.%.6d, %d.%.6d]",
		__value_f(rate_error(0)),
		__value_f(rate_error(1)),
		__value_f(rate_error(2)),
		__value_f(_rate_int(0)),
		__value_f(_rate_int(1)),
		__value_f(_rate_int(2)),
		__value_f(angular_accel(0)),
		__value_f(angular_accel(1)),
		__value_f(angular_accel(2)),
		__value_f(torque(0)),
		__value_f(torque(1)),
		__value_f(torque(2))
	);

	PX4_INFO("###\nRateSetpoint: [%d.%.6d, %d.%.6d, %d.%.6d]\nRateCurrent: [%d.%.6d, %d.%.6d, %d.%.6d]",
		__value_f(rate_sp(0)),
		__value_f(rate_sp(1)),
		__value_f(rate_sp(2)),
		__value_f(rate(0)),
		__value_f(rate(1)),
		__value_f(rate(2))
	);

	PX4_INFO("###\nGainP: [%d.%.6d, %d.%.6d, %d.%.6d]\nGainI: [%d.%.6d, %d.%.6d, %d.%.6d]\nGainD: [%d.%.6d, %d.%.6d, %d.%.6d]\nGainFF: [%d.%.6d, %d.%.6d, %d.%.6d]",
		__value_f(_gain_p(1)),
		__value_f(_gain_p(0)),
		__value_f(_gain_p(2)),
		__value_f(_gain_i(0)),
		__value_f(_gain_i(1)),
		__value_f(_gain_i(2)),
		__value_f(_gain_d(0)),
		__value_f(_gain_d(1)),
		__value_f(_gain_d(2)),
		__value_f(_gain_ff(0)),
		__value_f(_gain_ff(1)),
		__value_f(_gain_ff(2))
	);

	PX4_INFO("###\nGainKappa: [%d.%.6d, %d.%.6d, %d.%.6d]\nGainAlpha: [%d.%.6d, %d.%.6d, %d.%.6d]\nGainBeta: [%d.%.6d, %d.%.6d, %d.%.6d]\nGainGamma: [%d.%.6d, %d.%.6d, %d.%.6d]",
		__value_f(_gain_kappa(1)),
		__value_f(_gain_kappa(0)),
		__value_f(_gain_kappa(2)),
		__value_f(_gain_alpha(0)),
		__value_f(_gain_alpha(1)),
		__value_f(_gain_alpha(2)),
		__value_f(_gain_beta(0)),
		__value_f(_gain_beta(1)),
		__value_f(_gain_beta(2)),
		__value_f(_gain_gamma(0)),
		__value_f(_gain_gamma(1)),
		__value_f(_gain_gamma(2))
	);

	PX4_INFO("###\n\n###

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}

Vector3f RateControl::tanh_v(const Vector3f &vector)
{
	Vector3f response(0, 0, 0);
	for (int i = 0; i < 3; i++){
		response(i) = std::tanh(vector(i));
	}
	return response;
}

Vector3f RateControl::signale(const Vector3f &vector)
{
	Vector3f response(0, 0, 0);
	for (int i = 0; i < 3; i++){
		response(i) = (vector(i)>=0.f) ? 1.f : -1.f;
	}
	return response;
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

