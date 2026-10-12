# Quadrotor motor failure: spin and land (experiment)

Research prototype, SIH only, not for flight. Includes the EKF2 output predictor fix from PX4/PX4-Autopilot#29062,
which the spin needs.

## Question

After one motor of a quad dies, the vehicle yaw-spins. Real logs show that once it spins fast it
partly regains roll/pitch control. Can PX4 detect the failure, give up yaw on purpose, and hold
altitude or land?

## Physics

Motor 1 (front right) dead; motor 2 is diagonally opposite; motors 3 and 4 are its neighbours.

* **Hovering without spinning is impossible.** The body centre lies on the line between motors 3
  and 4. Zero roll/pitch torque therefore needs the opposite motor at zero thrust. Then 3 and 4
  (same spin direction) leave an unbalanced yaw torque. Reversing props doesn't change this: a
  prop at zero thrust makes almost no torque either.
* **Spinning makes it possible.** The vehicle spins up until aerodynamic yaw damping balances the
  net propeller torque (here 23 rad/s, 3.7 rev/s). The spin axis is a fixed body axis *n*, tilted
  a few degrees from body z. The roll/pitch torque from the uneven motors is balanced by the
  gyroscopic moment `ω × Iω` (∝ (Izz − Ixx)·r²·tilt) plus damping. That lets all three motors
  share the load.
* **Steady state needs no motor modulation at the spin frequency.** In the body frame the
  equilibrium is constant: motor 2 ≈ 0.33, motors 3/4 ≈ 0.66 (vs 0.41 each before the failure).
  It's not "three motors averaging to thrust everywhere"; it's a spinning top whose axis we steer.
  Tilting the axis (to move, or to reject a disturbance) is precession. That needs torque fixed
  in the world frame, i.e. modulation at the spin rate. Motor bandwidth vs spin rate matters for
  manoeuvres and recovery, not for holding the hover.
* **The opposite motor's share** (`MC_SPN_OPP`) trades spin rate against axis tilt. More share
  means less net yaw torque (slower spin) but needs a larger gyroscopic moment (more tilt).
  For the 1 kg test vehicle:

  | opposite share | neighbours / opposite (of max thrust) | spin | spin axis vs body z |
  |---|---|---|---|
  | 0 | 82 % / 0 % | 39 rad/s | 0° |
  | 0.25 | 73 % / 18 % | 31 rad/s | 2.4° |
  | 0.5 (default) | 65 % / 33 % | 23.5 rad/s | 7.1° |
  | 0.75 | 59 % / 45 % | 18 rad/s | 16.5° |

  With share 0 the two neighbours (which sit on a diagonal through the centre of mass) carry
  everything and there is no roll/pitch torque to balance at all.
* **Why it settles nearly upright.** In steady spin the spin axis is fixed both in the body and in
  the world, like a top: any world direction is an equilibrium, and the controller picks "along
  the desired thrust". The axis sits close to body z because the unbalanced torque is mostly
  propeller drag about z (it spins up about z), while the roll/pitch imbalance is cancelled by the
  gyroscopic moment, which only needs a small tilt: tilt ≈ τ_xy / ((Izz − Ixx)·r²). A flat quad
  (Izz > Ixx) spinning about its major axis is also the passively stable case. With
  Izz ≈ Ixx (Izz/Ixx 1.05) the same torque needs a 61° tilt, and thrust along body z is then far
  from the spin axis.

The approach is Mueller & D'Andrea, "Relaxed hover", ICRA 2014. The TU Delft/UZH INDI work
(Sun et al., 2020-21) is the robust, model-light variant that was flown on real vehicles.

## What changed in PX4

| Where | Change |
|---|---|
| `simulator_sih` | `SIH_MOT_FAIL`: motor N produces nothing whatever it is commanded. ESC current telemetry, so `FD_ACT_EN` can detect it. `SIH_W_MAX`: body-rate clamp (was a hard-coded 6π). |
| `control_allocator` | `CA_FAILURE_MODE 3`: remove the failed motor **and drop the yaw row**, so roll, pitch and thrust are allocated exactly on three motors. |
| `mc_rate_control` | `SpinRecoveryControl` (`MC_SPN_EN`): once the allocator handles a single quad motor failure, it replaces the rate PID. It points the spin axis along the thrust vector from the attitude setpoint (reduced attitude, yaw free). LQR on the dynamics linearised about the spinning equilibrium (gyroscopic coupling + motor lag), recomputed online every 50 ms as the spin rate changes (6×6 DARE by doubling). Thrust capped at what three motors can give with torque headroom. Needs a vehicle model: `MC_SPN_IXX/IZZ/KDW/TMAX/ARM/TAU`. |
| `ekf2` output predictor | **Bug fix, upstream-relevant.** The attitude correction is a body-frame error at the delayed fusion horizon (~150 ms back), but it was applied in the *current* body frame. Above roughly yaw rate × delay > 90° the correction points the wrong way and `vehicle_attitude` diverges: 145° off within 1.5 s while the delayed EKF state stayed within 2°. Fixed by holding the correction in earth frame. |
| `land_detector` | Skip the roll/pitch rotation check while a motor failure is handled: the tilted spin axis shows up as ~3 rad/s roll/pitch rate, so it never declared landed. |

## Results (SIH)

Vehicle unless noted: 1 kg, Ixx 0.01, Izz 0.018, KDW 0.005, T/W 2.45, motor τ 50 ms. Hovering at
19 m in Position mode, motor 1 dies, Land commanded at t = 25 s. Detection via `FD_ACT_EN` with
`MOTFAIL_TIME` 0.3 s.

| Run | Outcome | Altitude lost | Hold xy | Spin |
|---|---|---|---|---|
| stock (`CA_FAILURE_MODE 0`) | crash 8.0 m/s at 3.4 s | all | – | – |
| upstream (`CA_FAILURE_MODE 1`, still fights yaw) | crash 5.5 m/s at 3.3 s | all | – | – |
| **spin** | **lands 0.36 m/s, disarms** | 9.0 m | 1.5 m | 22.7 rad/s |
| spin, detection 0.1 s | lands 0.42 m/s | **4.2 m** | 1.1 m (alt ±0.8 m) | 22.8 rad/s |
| spin, motor 3 fails | lands 0.23 m/s | 8.8 m | 0.8 m | +23.3 rad/s |
| spin, model −20…30 % (Izz, KDW) | lands 0.45 m/s | 11.5 m | 1.6 m | 25.2 rad/s |
| spin, model +20…30 % (Ixx, Izz, KDW) | lands 0.43 m/s | 9.1 m | 1.7 m | 22.9 rad/s |
| spin, wind 5 m/s | lands 0.48 m/s | 12.9 m | 3.5 m | 27.0 rad/s |
| spin, motors 80 ms (model says 50) | lands 0.25 m/s | 15.9 m | 2.5 m | 24.3 rad/s |
| spin, motors 120 ms (model says 50) | crash 8.9 m/s | all | – | – |
| spin, stock SIH vehicle (T/W 2.04, KDW 0.025, Izz/Ixx 1.2) | crash 2.7 m/s | all | – | ~7 rad/s |

These ran on the PX4 1.18 based prototype. Rechecked on upstream main: spin with 0.1 s
detection lands at 0.33 m/s after losing 4.3 m, holding 0.9 m.

**Flying while spinning** (`spin_sih.py spin --moves`, detection 0.1 s): go-to 20 m N, then 20 m E,
then home, each reached within ~1 m in ~10 s at up to 4 m/s. Altitude stayed 19.0–19.9 m and the
motors were saturated < 0.2 % of the time. Full forward stick in Position mode reached 7 m/s and
stopped cleanly, but went **west**: manual sticks are relative to the heading, which is meaningless
while spinning. Sticks would have to be map-relative after a motor failure.
`render_video.py` turns a log into a 3D video.

Most of the altitude is lost **before detection**. In 0.3 s the stock controller, still driving
the dead motor, rolls the vehicle past 140° (upside down). Detection latency is the biggest lever.

## Python model (`spin_model.py`, `sweep.py`)

Same dynamics as SIH, used to design and sweep the controller.

* Motor lag tolerated in steady hover: about 300 ms at 12 rad/s spin down to 150 ms at 35 rad/s
  (τ·r ≲ 4–5 rad). Slower motors mainly cost altitude in the recovery.
* T/W 1.6 never holds; T/W 2.0 loses ~2–3 m, 2.45 ~1 m, 3.0 ~0 m (immediate detection).
  Izz/Ixx from 1.2 to 2.0 hardly matters as long as the vehicle can spin fast (low yaw damping).
* Reversible props (bidirectional ESC, reverse thrust at half efficiency, no zero-crossing delay
  modelled): they help only in the transient, and only with fast detection: 0.1 s → 7.6 m lost
  without, 1.0 m with. With 0.33 s detection they made it worse (10.8 → 17 m). Steady state never
  uses them.

## Before this could fly

* Detection in < 100 ms that doesn't flag healthy motors when they slam to full: ESC RPM
  telemetry (bidirectional DShot) compared with the command, or an angular-acceleration residual.
  In SIH the ESC current lags the command, and `MOTFAIL_TIME` 0.1 s still worked here.
* Model-light control: INDI using measured angular acceleration instead of an inertia/damping
  model. Real yaw damping is not linear, and the equilibrium depends on it.
* Gyro range: 23 rad/s is 1300°/s, inside 2000°/s, but check every IMU's configured range.
  Faster spin (lighter, low-drag frames) saturates.
* Only quad X, single failure, one model of the allocator normalisation.
* The LQR gains are solved (Riccati, 30 doublings of 6×6 double matrices) inside the rate loop
  every 50 ms. That is a CPU spike in a 1 kHz loop, and slow on an FPU without double precision
  (F4). Solve a gain table over the spin rate in the background instead, when parameters change.
* Flash: +11 KB on px4_fmu-v6x (99.2 % → 99.8 %). It should sit behind a Kconfig option.

## Does it scale to a large, slow quad? (`sweep_scale.py`)

The 1 kg vehicle scaled geometrically (mass s³, inertia s⁵, yaw damping s^4.5), Python model,
start at 20 m (an altitude loss of 20 m is a crash). "ok" means it holds position at the end.

| vehicle | T/W | motor τ that still holds, detection 0 s | … detection 0.1 s | altitude lost at τ 50 ms, 0 / 0.1 s |
|---|---|---|---|---|
| 1 kg, arm 0.2 m, spin 24 rad/s | 1.8 | none | none | – |
| | 2.0 | 200 ms | 300 ms | 2.2 / 10.0 m |
| | 2.45 | 200 ms | 150 ms | 0.7 / 7.6 m |
| 8 kg, arm 0.4 m, spin 17 rad/s | 1.8 | none | none | – |
| | 2.0 | 150 ms | 50 ms | 2.9 / 15.1 m |
| | 2.45 | 150 ms | 100 ms | 0.3 / 7.5 m |
| 21 kg, arm 0.55 m, spin 14 rad/s | 1.8 | none | none | – |
| | 2.0 | 100 ms | 50 ms | 3.3 / 17.5 m |
| | 2.45 | 150 ms | 100 ms | 0.2 / 6.8 m |

* Thrust margin decides first: T/W 1.8 never holds at any size.
* Pure geometric scaling would make a larger quad tolerate proportionally slower motors (time
  scales with √s). In the model it tolerates about the same absolute τ, because the position
  loop and the detection time are not scaled. Large propellers are slower in practice
  (150–300 ms), so a large quad is marginal unless its motors respond within ~100 ms.
* The spin is slower (14 rad/s at 21 kg) but the axis tilt does not grow: it scales as s⁰.

## Running it

```
cd Tools/simulation/motor_failure_spin
python3 spin_model.py --ctrl lqr --plot /tmp/lqr.png      # Python model
python3 sweep.py                                          # motor lag, inertia and T/W sweeps
python3 sweep_scale.py                                    # vehicle size sweep
make px4_sitl_default                                     # from the repository root, then:
python3 spin_sih.py spin [--extra MOTFAIL_TIME=0.1]       # SIH, instance 7; modes stock|remove|spin
python3 spin_sih.py spin --moves --land-after 58 --extra MOTFAIL_TIME=0.1   # go-to moves while spinning
python3 ana.py logs/result_spin_realistic.ulg out.png
python3 render_video.py logs/result_spin_realistic.ulg out.mp4
```
`spin_sih.py` uses the SIH build of the repository it sits in. Logs go to `logs/`, results to
`result_*.json` (both ignored by git).
