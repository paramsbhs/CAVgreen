# CAVgreen Development Plan

Roadmap for finishing the two remaining labs — **Lab 5 (SLAM + Pure Pursuit)** and
**Lab 8 (MPC)** — on top of the already-working stack (safety_node, wall_follow,
gap_follow), in simulation first and then on the car.

---

## 0. Current state (verified 2026-07-09)

| Package | Status | Sim wiring | Hardware wiring |
|---|---|---|---|
| `safety_node` | implemented, on-car tuned | native topics (`/scan`, `/ego_racecar/odom`, `/drive`) | `safety_node_hardware_launch.py` → `/drive_safety`, odom remapped to `/odom` |
| `wall_follow` | implemented | `wall_follow_launch.py` | `wall_follow_hardware_launch.py` → `/drive_wall_follow` |
| `gap_follow` | implemented + sim-proven (7 laps levine_blocked, no contact) | `gap_follow_sim_launch.py` | `gap_follow_hardware_launch.py` → `/drive_gap_follow` |
| `pure_pursuit` (Lab 5) | **skeleton — to implement** | `pure_pursuit_sim_launch.py` | `pure_pursuit_hardware_launch.py` → `/drive_pure_pursuit` |
| `mpc` (Lab 8) | **scaffold — TODOs to fill** | `mpc_sim_launch.py` | `mpc_hardware_launch.py` → `/drive_mpc` |

- The old `pure_pursuit_slam` package was ROS 1 (rospy/catkin) and could never build here; it has been replaced by the ROS 2 `pure_pursuit` package from the lab 5 template.
- All five packages build under Foxy in the sim container; `cvxpy==1.3.2` is installed there (and added to the Dockerfile).
- The sim container now mounts all lab packages (see `f1tenth_gym_ros/docker-compose.yml`).
- Sim map is `levine_blocked` (stock levine has open doorways; the blocked variant is flood-fill-verified closed).
- `ackermann_mux` config (`wall_follow/config/mux_params.yaml`) has inputs for all four algorithms at priority 10 and safety at 100. Run one navigation source at a time.
- `src/f1tenth_system` is an **empty placeholder** on this machine — the driver stack lives on the Jetson. See §5.

## 1. Conventions (the contract every node follows)

**Topics.** Nodes use *relative* names `odom` and `drive` so launch files can remap:

| Context | pose source | drive sink |
|---|---|---|
| Sim | `/ego_racecar/odom` (ground truth from gym bridge) | `/drive` (direct) |
| Car | `/pf/pose/odom` (particle filter) | `/drive_<algo>` → `ackermann_mux` → `/drive` → VESC |

**Launch/config pattern** (already in place for every package):
`launch/<pkg>_sim_launch.py` + `config/sim_params.yaml`, and
`launch/<pkg>_hardware_launch.py` + `config/hardware_params.yaml` with conservative speeds.

**Waypoint CSV format:** header `x,y,yaw,speed`, one row per point, closed loop implied.
Shared CSVs live in `pure_pursuit/waypoints/` (installed to the package share dir).

**Verification.** Every algorithm change is validated headlessly in the sim container before
touching the car: gym bridge + node + a monitor script that checks the odometry trail against
map pixels (the gym has **no collision physics** — cars drive through walls, so trajectory-vs-map
is the only honest check). Metrics: laps completed, lap time, min scan distance, wall contacts.

## 2. Phase 1 — Lab 5: Pure Pursuit (sim), then SLAM + PF (car)

### 2.1 Implement `pure_pursuit_node.py` (primary) 

Implement in Python first (`scripts/pure_pursuit_node.py`); port to C++ later only if the
control loop can't hold 25+ Hz (it will — pure pursuit is trivial computationally).

Node structure:
1. On startup: load `waypoint_csv` param into numpy arrays (x, y, yaw, v). Publish the full
   path once as a `visualization_msgs/MarkerArray` on `waypoints_viz` (latched QoS).
2. Subscribe `odom` (`nav_msgs/Odometry`). In the callback:
   - Extract pose (x, y, yaw from quaternion).
   - **Adaptive lookahead:** `L = clamp(lookahead_gain * v_current, lookahead_min, lookahead_max)`.
   - **Target search:** find the nearest waypoint index, then walk forward (wrapping) to the
     first waypoint at distance ≥ L from the car. Walking forward from nearest — never a global
     argmin on distance-to-L — prevents snapping to points behind or across the track.
   - **Transform to vehicle frame:** `dx, dy` = goal − car; `y_vf = -sin(yaw)*dx + cos(yaw)*dy`,
     `x_vf = cos(yaw)*dx + sin(yaw)*dy`. Skip (keep last command) if `x_vf < 0` and re-search
     with a larger L — the goal should never be behind the car.
   - **Curvature → steering:** `gamma = 2 * y_vf / L^2`; `steering = atan(WB * gamma)` with
     `WB = 0.33`; clamp to ±`max_steering_angle` (0.4189).
   - **Speed:** `v = min(velocity_scale * v_csv[target], max_speed)`; if the CSV speed column
     is empty/zero use `fallback_speed`. Optional: scale down by `abs(steering)` as in gap_follow.
   - Publish `AckermannDriveStamped` on `drive` (stamped, like the other nodes).
3. Publish a single Marker for the current target point (different color) each cycle.

Parameters are already defined in `config/{sim,hardware}_params.yaml` — declare exactly those.

### 2.2 Sim verification (do this before any car work)

- Bridge + `ros2 launch pure_pursuit pure_pursuit_sim_launch.py` with the generated
  `levine_blocked_centerline.csv` (619 points, all ≥ 0.75 m from walls — pre-validated).
- Reuse the lap-test monitor (see memory / `f1tenth-headless-sim-testing`): expect ≥ 5 clean
  laps in 150 s, no wall contact, and additionally compute **cross-track error** against the
  CSV (mean < 0.15 m, max < 0.4 m on corners). Tune `lookahead_*` until corners neither cut
  (too long) nor oscillate (too short).
- Regression: gap_follow lap test must still pass whenever shared files change.

### 2.3 SLAM on the car (slam_toolbox)

1. `sudo apt install ros-foxy-slam-toolbox` on the Jetson.
2. Bring up the base stack (see §5), then
   `ros2 launch slam_toolbox online_async_launch.py` with `scan_topic:=/scan`,
   `base_frame:=base_link`, `odom_frame:=odom`.
3. Drive the Levine loop slowly (~1 m/s) with the joystick, 2 full laps, closing the loop.
4. Save: `ros2 run nav2_map_server map_saver_cli -f levine_2nd` → produces
   `levine_2nd.pgm/.yaml` (Deliverable 1). Commit under `pure_pursuit/maps/`.

### 2.4 Localization (particle filter)

1. Clone `https://github.com/f1tenth/particle_filter` (ROS 2 branch) on the Jetson; it needs
   `range_libc` (build with the ray-marching kernel; GPU kernel optional on Jetson).
2. Configure `map_yaml_path` to `levine_2nd.yaml`, `scan_topic:=/scan`, odom from VESC.
3. Verify in RViz: pose cloud converges after a few metres of joystick driving; publishes
   `/pf/pose/odom` — exactly what the hardware launch files expect.

### 2.5 Waypoints on the real map + on-car run

1. With PF running, record: `ros2 run pure_pursuit waypoint_logger.py --ros-args
   -r odom:=/pf/pose/odom -p output_csv:=levine_2nd_recorded.csv` while joystick-driving a
   clean lap. (Optional: smooth with `scipy.interpolate.splprep/splev`, resample to 0.1 m.)
2. Save as `pure_pursuit/waypoints/levine_2nd_recorded.csv` (the hardware launch default).
3. First run protocol: mux + safety_node up first; hardware config starts mid-envelope
   (`max_speed: 3.5` — the vehicle is race-proven to 6.5 m/s, pure pursuit adds PF pose
   noise the racers didn't have); someone on the deadman/kill switch; verify tracking in
   RViz with markers; raise toward the envelope gradually.

## 3. Phase 2 — Lab 8: MPC

### 3.1 Fill the TODOs in `scripts/mpc_node.py`

The scaffold already builds the sparse block matrices and the solve loop. What's missing:

**ROS plumbing (constructor + `odom_callback`):** mirror pure_pursuit — load the same CSV
(needs a `yaw` and `speed` column: reuse the recorded/generated waypoints), subscribe `odom`,
publish `drive`. State vector is `z = [x, y, v, yaw]`; get `v` from odom twist. Publish
`steer_output = self.odelta_v[0]`, `speed_output = vehicle_state.v + self.oa[0]*DTK`.

**Objective (in `mpc_prob_init`):**
```python
objective += cvxpy.quad_form(cvxpy.vec(self.uk), R_block)                       # input cost
objective += cvxpy.quad_form(cvxpy.vec(self.xk - self.ref_traj_k), Q_block)     # tracking + terminal
objective += cvxpy.quad_form(cvxpy.vec(cvxpy.diff(self.uk, axis=1)), Rd_block)  # input smoothness
```
(cvxpy `vec` is column-major, i.e. stacks per-timestep state/input vectors — which is exactly
the order `R_block`/`Q_block` were built in. Don't reorder.)

**Constraints:**
```python
constraints += [cvxpy.vec(self.xk[:, 1:]) ==
                self.Ak_ @ cvxpy.vec(self.xk[:, :-1])
                + self.Bk_ @ cvxpy.vec(self.uk) + self.Ck_]          # dynamics
constraints += [cvxpy.abs(cvxpy.diff(self.uk[1, :]))
                <= self.config.MAX_DSTEER * self.config.DTK]          # steering slew
constraints += [self.xk[:, 0] == self.x0k]                            # initial state
constraints += [self.xk[2, :] <= self.config.MAX_SPEED,
                self.xk[2, :] >= self.config.MIN_SPEED]               # speed bounds
constraints += [cvxpy.abs(self.uk[0, :]) <= self.config.MAX_ACCEL]    # accel bounds
constraints += [cvxpy.abs(self.uk[1, :]) <= self.config.MAX_STEER]    # steering bounds
```

**Velocity-profiled reference:** the CSV `speed` column feeds `sp` in
`calc_ref_trajectory` (already implemented via `utils.calc_interpolated_ref_trajectory`).
For a smoother profile than the two-tier generator output, interpolate
`v(k) = v_max - (v_max - v_min) * |curvature(k)| / max|curvature|` over the spline.

### 3.2 Sim verification

- Same harness and criteria as pure pursuit (§2.2), plus:
  - **Solve health:** log and alert on any `Cannot solve mpc` status; zero tolerated per lap.
  - **Solve time:** print p95 of `MPC_prob.solve` wall time. Budget: < 20 ms in the container.
- Visualize predicted trajectory (`ox, oy`) as a Marker strip to debug weight tuning:
  sluggish corners → raise Q position terms; steering jitter → raise `Rdk[1,1]`.

### 3.3 On-car

- Check solve time on the Jetson first (`python3 -m timeit` the solve with warm start); if
  p95 > 40 ms, reduce horizon `TK` 8 → 6 or drop control rate to 20 Hz.
- Same first-run protocol as pure pursuit; MPC gets no speed advantage until tracking error
  and solver health look clean at 2 m/s.
- The C++ node (`src/mpc_node.cpp`, OSQP-Eigen) is optional performance headroom: the package
  builds it automatically only where `OsqpEigen` is installed. Not needed unless Python can't
  hold rate on the Jetson.

## 4. Testing methodology (applies to every phase)

1. Build in the sim container: `docker compose up -d sim` (in `src/f1tenth_gym_ros/`), then
   `docker exec f1tenth_gym_ros-sim-1 bash -c "source /opt/ros/foxy/setup.bash && cd /sim_ws && colcon build --packages-select <pkg>"`.
2. Bridge headless: `ros2 run f1tenth_gym_ros gym_bridge --ros-args -r __node:=bridge --params-file /sim_ws/src/f1tenth_gym_ros/config/sim.yaml` (the node rename is mandatory).
3. Launch the node under test via its `*_sim_launch.py`.
4. Run the lap monitor (trajectory-vs-map, laps, min scan, cross-track error for trackers).
5. Record the numbers in the PR/commit message. A change that reduces laps or adds wall
   contact does not merge.
6. Gotchas already learned: use `docker exec -d` for daemons; `pkill -f "patter[n]"` bracket
   trick; container `/tmp` is wiped on recreation — keep monitor scripts on the host.

## 5. Hardware bringup (Jetson) — reference

Bring-up order for every car session:
1. `f1tenth_stack` bringup (VESC + lidar + `ackermann_mux` + joy teleop). **Backlog:** the
   `src/f1tenth_system` directory here is empty — sync the Jetson's copy into this repo (or
   add it as a submodule) so the whole system is version-controlled together.
2. `safety_node_hardware_launch.py` — verify a hand in front of the lidar brakes the car at
   low speed **before every autonomous run**.
   - `ttc_threshold: 0.35` is race-proven (competition-tested with wall_follow at up to
     6.5 m/s). The wall_follow + safety_node hardware configs are the authoritative
     baseline for the vehicle envelope: **6.5 m/s straights / 3.3 m/s corners**.
3. Localization (PF, §2.4) when running pure_pursuit/MPC.
4. One navigation node via its `*_hardware_launch.py` (mux arbitrates; safety wins).
5. Joystick deadman ready; speeds start at hardware-config defaults.

## 6. Milestones

| # | Milestone | Exit criterion |
|---|---|---|
| M1 | Pure pursuit laps in sim | ✅ done 2026-07-15: 7 laps @ 19.9 s, mean cross-track 0.069 m, no wall contact |
| M2 | SLAM map of Levine 2nd | `levine_2nd.pgm/.yaml` committed, loop closed |
| M3 | PF localization stable | pose track survives 2 joystick laps without divergence |
| M4 | Pure pursuit on car | 1 clean lap at 3.5 m/s (Deliverable 3 video) |
| M5 | MPC laps in sim | ≥5 clean laps, zero solver failures (Deliverable 2 video) |
| M6 | MPC on car | 1 clean lap at 3.5 m/s, p95 solve < 40 ms |

Sequence M1 → M5 can proceed entirely in sim while car time is scarce; M2–M4 need the car.

## 7. Risks / open items

- **PF pose rate & latency** (~25 Hz, tens of ms): pure pursuit tolerates it; MPC may need
  state prediction from the last VESC odom twist between PF updates.
- **cvxpy on Jetson (py3.8, aarch64):** wheels exist for 1.3.x; if install fights, the C++
  OSQP-Eigen path is the fallback (already scaffolded).
- **Speed columns in recorded CSVs** come from joystick driving — sanity-cap with
  `velocity_scale`/`max_speed` rather than trusting them.
- **mpc setup.py `console_scripts`** references `mpc.mpc_node:main`, but the node lives in
  `scripts/` (installed by CMake). Harmless under ament_cmake; ignore or clean up later.
- Stock `levine.png` is open — never use it for autonomous sim runs; `levine_blocked` only.
