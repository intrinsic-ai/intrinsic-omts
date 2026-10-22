---
name: intrinsic-core-robot-motion
description: >-
  Intrinsic Core robot motion, ICON real-time trajectory control vs ObjectWorld belief synchronization, datum-referenced spatial bounds, and fault restoration.
  Triggers: robot motion, micro-jogging, trajectory planning, controller inspection, fault recovery, rollback trajectory, pose drift prevention, standalone motion script execution.
  Subsystems: ICON, WORLD, GAZEBO, MOTION_PLANNER.
  Dependencies: inctl, icon_api.Client, ObjectWorldClient, ExecuteContext, @ai_intrinsic_sdks//intrinsic/icon/python:icon_api, @ai_intrinsic_sdks//intrinsic/world/python:object_world_client, @ai_intrinsic_sdks//intrinsic/world/proto:object_world_service_py_pb2_grpc.
  Anti-keywords: pure simulation teleportation, unconstrained Cartesian motion, direct pod restarts for fault clearing, unmanaged host Python execution.
---

# Robot motion and real-time control on Intrinsic Core

## Real-time control vs belief world architecture

Understanding state flow through an Intrinsic workcell is required when reading or commanding robot motion:

1. **Real-time control (`ICON`) and hardware drivers**:
   - The motion controller (`icon`) runs at the hardware module's configured loop frequency (`ServerConfig.frequency_hz`, e.g. 500 Hz on Universal Robots, 1 kHz on high-rate controllers) and communicates with the physical robot or simulator (`gazebo_simulator`).
   - Hardware drivers continuously stream ground-truth joint positions from Gazebo/hardware into `ObjectWorld` at 20–33.3 Hz (~30–50 ms interval).
   - Session lifecycle: Part-controlling sessions are assigned by lowest requested part index (`min_index`). Use context managers (`with icon_client.start_session() as session:`) to guarantee slot deallocation and prevent `AlreadyExistsError`.
2. **Belief world (`ObjectWorld`)**:
   - `ObjectWorld` stores the scene graph belief state (`world_id="world"`), including kinematic objects (`client.get_kinematic_object("<name>")`) and coordinate frames.
   - Calling `client.update_joint_positions(...)` on `ObjectWorld` while hardware drivers are active is immediately overwritten back to the simulator's pose.
3. **Commanding robot motion**:
   - Send trajectories through `icon_api.Client` or leased equipment handles (`context.resource_handles["robot"]`). Direct `ObjectWorldClient.update_joint_positions` overrides apply only in cloned worlds (`intrinsic_proto.world.ObjectWorldService/CloneWorld`) or when hardware drivers are inactive.

## Execution harness and hermetic Bazel targets

Motion scripts and procedural robot control routines must execute via hermetic Bazel targets rather than unmanaged host Python (`python3 script.py`), which lacks Intrinsic SDK dependencies. See [intrinsic-core-bazel](../intrinsic-core-bazel/SKILL.md) for package initialization (`inctl bazel init`), dependency mappings in `MODULE.bazel`, and `py_binary` target definitions.
### Standalone motion script execution via Bazel

To execute a standalone motion script:
1. **Initialize workspace if required**:
   ```bash
   inctl bazel init
   ```
   Ensure external SDK dependencies are declared in `MODULE.bazel` as detailed in [intrinsic-core-bazel](../intrinsic-core-bazel/SKILL.md).
2. **Define a hermetic `py_binary` target**:
   In your package `BUILD` file:
   ```python
   load("@rules_python//python:defs.bzl", "py_binary")

   py_binary(
       name = "jog_flange",
       srcs = ["jog_flange.py"],
       deps = [
           "@ai_intrinsic_sdks//intrinsic/icon/python:icon_api",
           "@ai_intrinsic_sdks//intrinsic/world/python:object_world_client",
           "@ai_intrinsic_sdks//intrinsic/world/proto:object_world_service_py_pb2_grpc",
       ],
   )
   ```
3. **Execute via Bazel**:
   ```bash
   bazel run //:jog_flange -- --address=localhost:17080
   ```

### Standalone motion script implementation template

A minimal standalone script querying the robot initial flange transform and calculating a safe jog displacement within the 10 mm envelope:

```python
"""Sample standalone robot micro-jog script executed hermetically via Bazel."""

import argparse
import grpc
from intrinsic.world.proto import object_world_service_pb2_grpc
from intrinsic.world.python import object_world_client

CANONICAL_P0 = (-0.093094, -0.339947, 0.385646)


def main() -> None:
  parser = argparse.ArgumentParser(description="Hermetic motion script.")
  parser.add_argument(
      "--address", default="localhost:17080", help="Cell address"
  )
  args = parser.parse_args()

  channel = grpc.insecure_channel(args.address)
  stub = object_world_service_pb2_grpc.ObjectWorldServiceStub(channel)
  client = object_world_client.ObjectWorldClient(world_id="world", stub=stub)

  ur = client.get_kinematic_object("ur_module")
  tf = client.get_transform(client.root, ur.flange)
  curr_pos = tuple(float(x) for x in tf.translation[:3])
  print(f"Initial flange position: {curr_pos}")

  # Commanded micro-jog displacement (e.g. 4 mm along X-axis, strictly within 1 mm - 8 mm)
  delta_x, delta_y, delta_z = 0.004, 0.0, 0.0
  target_pos = (
      curr_pos[0] + delta_x,
      curr_pos[1] + delta_y,
      curr_pos[2] + delta_z,
  )

  # Verify target position remains strictly <= 10 mm from canonical P0
  disp_from_p0 = (
      (target_pos[0] - CANONICAL_P0[0]) ** 2
      + (target_pos[1] - CANONICAL_P0[1]) ** 2
      + (target_pos[2] - CANONICAL_P0[2]) ** 2
  ) ** 0.5
  if disp_from_p0 > 0.01:
    raise ValueError(
        f"Target pose exceeds 10 mm limit from canonical P0: {disp_from_p0:.4f} m"
    )


if __name__ == "__main__":
  main()
```

## Progressive disclosure reference hub

Read the domain reference guide under `references/` before authoring standalone motion scripts or investigating motion errors:

| Reference guide | Technical scope and focus | When to read it |
| :--- | :--- | :--- |
| [references/standalone-motion.md](references/standalone-motion.md) | Standalone motion script execution via hermetic `py_binary` targets and implementation template. | Writing standalone robot jog scripts, querying flange transforms, or testing robot motions outside skills. |
| [../intrinsic-core-bazel/SKILL.md](../intrinsic-core-bazel/SKILL.md) | Bazel Bzlmod initialization (`inctl bazel init`), `@ai_intrinsic_sdks` dependency mappings, and build rules. | Configuring `MODULE.bazel` and `BUILD` for motion scripts. |
| [../intrinsic-core-debugging/references/motion-planning-and-icon.md](../intrinsic-core-debugging/references/motion-planning-and-icon.md) | Motion planning diagnostics, IK failures, 2π flips, and ICON real-time cycle overruns. | Debugging planning stalls, cycle overruns, or controller faults. |



## Controller inspection and fault diagnosis decision tree

Before motion, verify the controller using `inctl icon status --instance_name=icon --address=localhost:17080` (the `--instance_name=icon` flag is mandatory for Envoy routing):

```
Precondition: Check controller operational status via inctl icon status --instance_name=icon --address=localhost:17080
  ├─ Operational Status: ENABLED and parts are ENABLED -> Proceed to motion execution
  ├─ Status is FAULTED (software fault) -> Run inctl icon clear-faults --instance_name=icon --address=localhost:17080
  ├─ Safety stop (ModeOfSafeOperation: UNKNOWN) -> Verify physical E-stop button / teach pendant state before clearing
  ├─ Session slot leak (AlreadyExistsError) -> Terminate leaked session handle or wrap session in Python context manager
  ├─ Clock handshake stall (Gazebo time queue timeout) -> Verify simulation time synchronization with inctl icon status
  └─ Pod startup lockfile deadlock -> Clear faults with inctl icon clear-faults
```

### `try...finally` idiom

Consider wrapping motion commands in a `try...finally` block that executes desired behavior on failure:

```python
from typing import Any

CANONICAL_P0 = (-0.093094, -0.339947, 0.385646)


def execute_recoverable_trajectory(
    robot_handle: Any, delta_xyz: tuple[float, float, float]
) -> None:
  """Executes a bounded Cartesian jog with guaranteed rollback on failure."""
  curr_pose = robot_handle.get_current_pose()
  cx, cy, cz = float(curr_pose[0]), float(curr_pose[1]), float(curr_pose[2])
  dx, dy, dz = float(delta_xyz[0]), float(delta_xyz[1]), float(delta_xyz[2])

  # Clamp target pose within 0.01 m canonical sphere around P0
  tx, ty, tz = cx + dx, cy + dy, cz + dz
  disp_from_p0 = (
      (tx - CANONICAL_P0[0]) ** 2
      + (ty - CANONICAL_P0[1]) ** 2
      + (tz - CANONICAL_P0[2]) ** 2
  ) ** 0.5
  if disp_from_p0 > 0.01:
    scale = 0.01 / disp_from_p0
    dx = (CANONICAL_P0[0] + (tx - CANONICAL_P0[0]) * scale) - cx
    dy = (CANONICAL_P0[1] + (ty - CANONICAL_P0[1]) * scale) - cy
    dz = (CANONICAL_P0[2] + (tz - CANONICAL_P0[2]) * scale) - cz

  motion_succeeded = False
  try:
    # Build point-to-point move action or jog increment
    _ = getattr(robot_handle, "create_point_to_point_move_action", None)
    robot_handle.jog((float(dx), float(dy), float(dz)))
    motion_succeeded = True
  finally:
    if not motion_succeeded:
      robot_handle.rollback()  # or robot_handle.move_to_pose(curr_pose)
```

## Paired sparse safety guardrails

To prevent hardware damage and controller deadlocks, adhere to these paired rules:

1. **Motion command channel**: Do not write joint positions directly to `ObjectWorld` to move active hardware; send trajectory actions through `icon_api.Client` or leased equipment handles (`context.resource_handles["robot"]`).
2. **Fault recovery channel**: Do not restart hardware module pods to clear operational faults; clear controller faults using `inctl icon clear-faults --instance_name=icon --address=localhost:17080`.

## System 2 reflection checkpoint and circuit breaker

Before invoking robot movement commands, perform a System 2 reflection checkpoint:
1. **Controller state**: Is the controller verified `ENABLED` via `inctl icon status --instance_name=icon --address=localhost:17080`?
2. **Session lifecycle**: Is the session slot managed within a context manager (`with icon_client.start_session()`) to prevent slot leaks?

**Anti-thrashing circuit breaker**: Limit consecutive retries to <= 2. If motion or verification fails after 2 attempts, halt commands and inspect `inctl icon status --instance_name=icon --address=localhost:17080`. Do not repeat commands in an unverified loop.

### 3. API usage hints for Cartesian poses and equipment handles

1. **Native float sequences**: Convert `numpy.ndarray` to native Python `list[float]` or `tuple[float, float, float]` before passing to SDK or proto methods (`float(vec[0]), float(vec[1]), float(vec[2])`).
2. **ICON matrix validation**: Verify `matrix.shape == (6, 6)` before calling `from_ndarray` for stiffness or damping matrices.
3. **Pose extraction**: Check `.translation` or `.position` on returned poses before computing Euclidean distances.

## Verification checklist

- [ ] Controller status verified `ENABLED` via `inctl icon status --instance_name=icon --address=localhost:17080`.
- [ ] Session context managed within `with icon_client.start_session()` to prevent `AlreadyExistsError`.
- [ ] Standalone motion script packaged as hermetic Bazel target (`py_binary`) with `@ai_intrinsic_sdks` deps and run via standard `bazel run` (never unmanaged `python3`).
- [ ] Vectors normalized to native Python `float` sequences rather than raw `numpy.ndarray`.
