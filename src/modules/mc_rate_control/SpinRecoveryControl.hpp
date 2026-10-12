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

/**
 * @file SpinRecoveryControl.hpp
 *
 * Quadrotor control after the complete loss of one motor (experimental).
 *
 * Three motors cannot hold yaw, so the vehicle is let spin about yaw. While spinning, the body
 * axis n that the angular velocity is aligned with stays still in the world; the controller
 * points n along the desired thrust direction (reduced attitude) and leaves yaw free.
 * The roll/pitch loop is an LQR on the dynamics linearised about the spinning equilibrium
 * (gyroscopic coupling and motor lag included). The gains are recomputed online as the spin
 * rate changes.
 *
 * Reference: M. W. Mueller and R. D'Andrea, "Stability and control of a quadrocopter despite
 * the complete loss of one, two, or three propellers", ICRA 2014.
 *
 * Assumes the PX4 quadrotor X layout (motor 1 front right, 2 rear left, 3 front left, 4 rear right)
 * and that the allocator removed the failed motor and dropped yaw (CA_FAILURE_MODE 3), so that
 * roll, pitch and thrust are allocated exactly.
 */

#pragma once

#include <matrix/matrix/math.hpp>

class SpinRecoveryControl
{
public:
	struct Params {
		float ixx{0.01f};          ///< roll/pitch inertia [kg m^2]
		float izz{0.018f};         ///< yaw inertia [kg m^2]
		float kdw{0.005f};         ///< rotational damping [N m s/rad]
		float motor_thrust_max{6.f}; ///< thrust of one motor at full command [N]
		float arm{0.2f};           ///< motor offset along body x and y [m]
		float motor_tau{0.05f};    ///< motor time constant [s]
		float opposite_ratio{0.5f}; ///< thrust of the motor opposite the failed one / thrust of its neighbours
		float tilt_max{0.44f};     ///< largest angle between the spin axis and body z [rad]
		float q_att{400.f};        ///< LQR weight on the thrust direction error
		float r_torque{400.f};     ///< LQR weight on the torque [1/(N m)^2]
	};

	void setParams(const Params &params) { _params = params; _gain_age = 1e3f; }

	/**
	 * Start (or restart) spin control.
	 * @param failed_motor index of the failed motor (0..3)
	 */
	void reset(int failed_motor);

	/**
	 * @param rates body rates [rad/s]
	 * @param q attitude
	 * @param thrust_sp_ned desired thrust vector in NED, normalised thrust units (as in vehicle_attitude_setpoint)
	 * @param dt time step [s]
	 * @param torque_sp normalised torque setpoint (z is always 0)
	 * @param thrust_sp normalised thrust setpoint along body z
	 */
	void update(const matrix::Vector3f &rates, const matrix::Quatf &q, const matrix::Vector3f &thrust_sp_ned, float dt,
		    matrix::Vector3f &torque_sp, matrix::Vector3f &thrust_sp);

	/** body axis that is held along the desired thrust direction */
	const matrix::Vector3f &spinAxis() const { return _n; }

private:
	using Vector6d = matrix::Vector<double, 6>;
	using Matrix6d = matrix::SquareMatrix<double, 6>;

	/** spinning equilibrium for yaw rate r and total thrust [N]: body axis, roll/pitch rates and torques */
	void equilibrium(float r, float thrust, matrix::Vector3f &n, matrix::Vector2f &pq, matrix::Vector2f &torque) const;

	/** relinearise around the equilibrium at yaw rate r and solve the LQR */
	void updateGains(float r, float thrust);

	Params _params{};
	int _failed_motor{-1};

	// equilibrium the gains were computed for
	matrix::Vector3f _n{0.f, 0.f, -1.f};
	matrix::Vector2f _pq0{};
	matrix::Vector2f _torque0{};
	matrix::Matrix<float, 2, 6> _K{};
	float _gain_age{1e3f};

	matrix::Vector2f _torque_est{}; ///< torque the motors make now (motor lag model) [N m]

	// workspace of updateGains(), kept out of the stack
	struct {
		Matrix6d A, Ad, Ak, Gk, Hk, W, AW, T;
	} _dare{};
};
