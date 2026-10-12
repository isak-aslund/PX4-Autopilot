/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "SpinRecoveryControl.hpp"

#include <mathlib/mathlib.h>

using namespace matrix;

namespace
{
// PX4 quadrotor X motor positions in units of the arm length
constexpr float kMotorX[4] {1.f, -1.f, 1.f, -1.f};
constexpr float kMotorY[4] {1.f, -1.f, -1.f, 1.f};
constexpr int kOpposite[4] {1, 0, 3, 2};

constexpr double kGainDt = 0.004;   // sample time the LQR is designed for [s]
constexpr float kGainPeriod = 0.05f; // how often the gains are recomputed [s]
}

void SpinRecoveryControl::reset(int failed_motor)
{
	_failed_motor = failed_motor;
	_torque_est.zero();
	_gain_age = 1e3f;
}

void SpinRecoveryControl::equilibrium(float r, float thrust, Vector3f &n, Vector2f &pq, Vector2f &torque) const
{
	// motor thrusts we want while hovering: the neighbours of the failed motor carry the same,
	// the opposite one a fraction of it
	const float rho = _params.opposite_ratio;
	const float f_neighbour = thrust / (2.f + rho);

	torque.zero();

	for (int i = 0; i < 4; i++) {
		float f = f_neighbour;

		if (i == _failed_motor) {
			f = 0.f;

		} else if (i == kOpposite[_failed_motor]) {
			f = rho * f_neighbour;
		}

		torque(0) += -kMotorY[i] * _params.arm * f;
		torque(1) += kMotorX[i] * _params.arm * f;
	}

	// steady spin: that torque balances gyroscopic and damping moments, w x I w + kdw w
	const float D = _params.izz - _params.ixx;
	const float k = _params.kdw;
	const float det = k * k + D * D * r * r;

	pq(0) = (k * torque(0) - D * r * torque(1)) / det;
	pq(1) = (D * r * torque(0) + k * torque(1)) / det;

	// until the vehicle spins fast enough the torque is not reachable: limit the tilt of the spin axis,
	// which only means that the opposite motor runs lower for now
	const float pq_max = fabsf(r) * tanf(_params.tilt_max);

	if (pq.norm() > pq_max) {
		pq *= pq_max / pq.norm();
		torque(0) = k * pq(0) + D * r * pq(1);
		torque(1) = k * pq(1) - D * r * pq(0);
	}

	n = Vector3f(pq(0), pq(1), fabsf(r) > 1e-3f ? r : -1e-3f).normalized();

	if (n(2) > 0.f) {
		n = -n;
	}
}

void SpinRecoveryControl::updateGains(float yaw_rate, float thrust)
{
	equilibrium(yaw_rate, thrust, _n, _pq0, _torque0);

	const double ix = static_cast<double>(_params.ixx);
	const double D = static_cast<double>(_params.izz - _params.ixx);
	const double k = static_cast<double>(_params.kdw);
	const double tau = static_cast<double>(_params.motor_tau);
	const double r = static_cast<double>(yaw_rate);
	const double p = static_cast<double>(_pq0(0));
	const double q = static_cast<double>(_pq0(1));
	const double hx = static_cast<double>(_n(0));
	const double hy = static_cast<double>(_n(1));
	const double hz = static_cast<double>(_n(2));

	// state [p, q, hx, hy, torque_x, torque_y], h = desired thrust direction in body frame, yaw rate frozen.
	// The 6x6 matrices live in _dare rather than on the stack, which is too small for them.
	Matrix6d &A = _dare.A;
	A.zero();
	A(0, 0) = -k / ix;  A(0, 1) = -D * r / ix; A(0, 4) = 1. / ix;
	A(1, 0) = D * r / ix; A(1, 1) = -k / ix;  A(1, 5) = 1. / ix;
	// dh/dt = h x w, hz = -sqrt(1 - hx^2 - hy^2)
	A(2, 1) = -hz;      A(2, 2) = q * hx / hz;      A(2, 3) = r + q * hy / hz;
	A(3, 0) = hz;       A(3, 2) = -p * hx / hz - r; A(3, 3) = -p * hy / hz;
	A(4, 4) = -1. / tau;
	A(5, 5) = -1. / tau;

	Matrix<double, 6, 2> B{};
	B(4, 0) = 1. / tau;
	B(5, 1) = 1. / tau;

	// discretise
	Matrix6d &Ad = _dare.Ad;
	Matrix6d &T = _dare.T;
	Ad.setIdentity();
	T = A * kGainDt;
	Ad += T;
	Ad += T * T * 0.5;
	const Matrix<double, 6, 2> Bd = B * kGainDt;

	SquareMatrix<double, 2> R{};
	R(0, 0) = R(1, 1) = static_cast<double>(_params.r_torque) * kGainDt;

	// discrete algebraic Riccati equation by structure-preserving doubling
	Matrix6d &Ak = _dare.Ak;
	Matrix6d &Gk = _dare.Gk;
	Matrix6d &Hk = _dare.Hk;
	Matrix6d &W = _dare.W;
	Matrix6d &AW = _dare.AW;
	Ak = Ad;
	Gk = Bd * inv(R) * Bd.T();
	Hk.zero();
	Hk(0, 0) = Hk(1, 1) = kGainDt;
	Hk(2, 2) = Hk(3, 3) = static_cast<double>(_params.q_att) * kGainDt;

	for (int i = 0; i < 30; i++) {
		T = Gk * Hk;

		for (int j = 0; j < 6; j++) {
			T(j, j) += 1.;
		}

		if (!inv(T, W)) {
			return; // keep the old gains
		}

		AW = Ak * W;
		// H_next = H + A^T H W A, G_next = G + A W G A^T, A_next = A W A
		T = Hk * W;
		Hk += Ak.T() * (T * Ak);
		T = AW * Gk;
		Gk += T * Ak.T();
		Ak = AW * Ak;
	}

	SquareMatrix<double, 2> S = R + Bd.T() * Hk * Bd;
	SquareMatrix<double, 2> S_inv;

	if (!inv(S, S_inv)) {
		return;
	}

	const Matrix<double, 2, 6> K = S_inv * Bd.T() * Hk * Ad;

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 6; j++) {
			_K(i, j) = PX4_ISFINITE(K(i, j)) ? static_cast<float>(K(i, j)) : 0.f;
		}
	}
}

void SpinRecoveryControl::update(const Vector3f &rates, const Quatf &q, const Vector3f &thrust_sp_ned, float dt,
				 Vector3f &torque_sp, Vector3f &thrust_sp)
{
	torque_sp.zero();
	thrust_sp.zero();

	if (_failed_motor < 0 || _failed_motor > 3) {
		return;
	}

	// normalised thrust in PX4 is the mean motor command; physical units for the model
	const float thrust_to_newton = 4.f * _params.motor_thrust_max;
	const float thrust_norm = thrust_sp_ned.norm();
	const Vector3f thrust_dir = thrust_norm > 1e-3f ? thrust_sp_ned / thrust_norm : Vector3f(0.f, 0.f, -1.f);

	// desired thrust direction in body frame
	const Vector3f h = q.rotateVectorInverse(thrust_dir);

	// thrust acts along -z_B: scale it so the part along the desired direction is what was asked for
	// Three motors cannot make full thrust and still balance the torques. Cap the thrust where the two
	// neighbours of the failed motor are at 90 % in the hover distribution, leaving room for control.
	const float thrust_max = 0.9f * (2.f + _params.opposite_ratio) / 4.f;
	const float cos_tilt = math::max(-h(2), 0.5f);
	const float thrust_body = math::min(thrust_norm / cos_tilt, thrust_max);

	_gain_age += dt;

	if (_gain_age > kGainPeriod) {
		updateGains(rates(2), thrust_body * thrust_to_newton);
		_gain_age = 0.f;
	}

	Vector<float, 6> x;
	x(0) = rates(0) - _pq0(0);
	x(1) = rates(1) - _pq0(1);
	x(2) = h(0) - _n(0);
	x(3) = h(1) - _n(1);
	x(4) = _torque_est(0) - _torque0(0);
	x(5) = _torque_est(1) - _torque0(1);

	const Vector2f torque = _torque0 - _K * x;

	// motor lag model
	_torque_est += (torque - _torque_est) * math::min(dt / _params.motor_tau, 1.f);

	// physical torque to the normalised setpoint the allocator expects: for the X layout a normalised
	// roll torque of 1 is +-1/sqrt(2) on each motor, i.e. 2 sqrt(2) arm motor_thrust_max newton metres
	const float torque_scale = 2.f * M_SQRT2_F * _params.arm * _params.motor_thrust_max;
	torque_sp(0) = math::constrain(torque(0) / torque_scale, -1.f, 1.f);
	torque_sp(1) = math::constrain(torque(1) / torque_scale, -1.f, 1.f);
	thrust_sp(2) = -math::constrain(thrust_body, 0.f, 1.f);
}
