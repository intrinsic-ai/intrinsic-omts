# Manipulating object world, spatial poses, and robot motions in `SBL`

## Multi-world lifecycles and simulation synchronization

In `intrinsic.solutions`, `solution.world` connects to the live runtime belief world (`worlds.EditWorldId.BELIEF` / `world_id="world"`), while the baseline workcell definition resides in the initial world (`worlds.EditWorldId.INITIAL` / `world_id="init_world"`). Physics simulation runs in a distinct simulation world (`"sim_world"`).

### Named world instances and state synchronization

Live workcell deployments maintain distinct world instances managed by `intrinsic_proto.world.ObjectWorldService`:

| World identifier | SBL enum | Primary responsibility | Synchronization mechanism |
| :--- | :--- | :--- | :--- |
| `"world"` | `worlds.EditWorldId.BELIEF` | Active runtime belief state queried by motion planning, skills, and the executive. | Live target of `solution.world` mutations (`update_transform`, `create_object`, `reparent_object`). |
| `"sim_world"` | N/A | Simulation scene queried by the Gazebo physics engine (`gzserver`). | Synchronized from `"world"` prior to execution via `start_from_world_state=worlds.EditWorldId.BELIEF`. |
| `"init_world"` | `worlds.EditWorldId.INITIAL` | Immutable baseline scene graph configured during solution authoring or asset installation. | Restores both `"world"` and `"sim_world"` to factory baseline via `start_from_world_state=worlds.EditWorldId.INITIAL`. |
| `"save_monitor"` | N/A | Internal tracking world used to detect unsaved changes against the solution package. | Managed automatically by the solution deployment backend. |

### Synchronizing belief edits to `Gazebo` simulation

Calling `solution.simulator.reset()` directly is deprecated and can cause real-time socket deadlocks. To synchronize Python belief edits to Gazebo physics, pass `start_from_world_state` to `solution.executive.run()`:

- **Preserve and synchronize Python belief edits**:
```python
# Clones "world" into "sim_world" before Behavior Tree execution begins
solution.executive.run(
    tree,
    start_from_world_state=worlds.EditWorldId.BELIEF,
)
```

- **Reset simulation and belief to initial solution baseline**:
```python
# Resets both "world" and "sim_world" back to "init_world"
solution.executive.run(
    tree,
    start_from_world_state=worlds.EditWorldId.INITIAL,
)
```

- **Run without resetting simulation state (default)**:
```python
# Leaves Gazebo and ObjectWorld running continuously without state resets
solution.executive.run(tree, start_from_world_state=None)
```

### Inspecting the simulation world directly

To inspect Gazebo physics state directly from Python, connect an independent `ObjectWorld` instance to `"sim_world"`:

```python
from intrinsic.solutions import worlds

sim_world = worlds.ObjectWorld.connect("sim_world", solution.grpc_channel)
sim_robot_pose = sim_world.get_transform(sim_world.root, sim_world.robot)
```

---

## Spatial transformation trees and `Pose3` construction

### Constructing `Pose3` and rotation transforms

In `intrinsic.math.python.data_types`, `Pose3.__init__(self, rotation=None, translation=None)` defines `rotation` as the first argument and `translation` as the second argument. Always construct `Pose3` instances using explicit keyword arguments to avoid positional argument inversion:

```python
from intrinsic.math.python import data_types

target_pose = data_types.Pose3(
    rotation=data_types.Rotation3.from_euler_angles(rpy_degrees=[0.0, 90.0, 0.0]),
    translation=[0.40, -0.15, 0.25],
)
```

- **Quaternion normalization**: Ensure orientation quaternions have unit norm (norm(q) == 1.0). Passing unnormalized quaternions raises `ValueError: Quaternion is not normalized`.
- **Antipodal quaternion equality**: `Pose3.__eq__` and `Rotation3.__eq__` treat antipodal quaternions (+q and -q) as equal rotations (`True`), whereas raw protobuf quaternion comparisons evaluate component-wise and return `False`. Compare spatial poses using `Pose3` or `Rotation3`.

### Traversing `ObjectWorld` nodes and comparison rules

- **Immutable local snapshots**: `WorldObject`, `KinematicObject`, and `Frame` instances returned by `world.get_object(...)` or `world.robot` are local immutable snapshots. Calling `world.update_transform(...)` does not mutate existing Python objects. Always re-fetch nodes from `solution.world` to inspect updated transforms.
- **Node equality comparison**: `TransformNode.__eq__` raises `NotImplementedError`. Always compare nodes by unique identifier:
```python
if node_a.id == node_b.id:
  # Nodes reference the same kinematic entity
  pass
```
- **Frame lookup parameter ordering**: In `world.get_frame(frame_name, object_name)`, the frame name comes **first** and the parent object name comes **second**:
```python
flange_frame = solution.world.get_frame("flange", "robot")
```

### Updating transforms on non-neighboring nodes

When updating the relative pose between two nodes that are not immediate parent-child neighbors in the kinematic tree, specify `node_to_update` so the transform solver identifies which branch to adjust:

```python
solution.world.update_transform(
    node_a=solution.world.root,
    node_b=solution.world.workpiece_frame,
    a_t_b=target_pose,
    node_to_update=solution.world.workpiece_frame,
)
```

### High-level cartesian motion targets

`intrinsic.solutions.worlds` provides `CartesianMotionTarget` to model Cartesian alignments (`world_t_target = world_t_frame * offset`):

```python
from intrinsic.solutions import worlds

# Aligns tool with workpiece frame offset by 50 mm along local z-axis
cartesian_target = worlds.CartesianMotionTarget(
    tool=solution.world.gripper.tool_frame,
    frame=solution.world.workpiece_frame,
    offset=data_types.Pose3(translation=[0.0, 0.0, 0.05]),
)
```

---

## Object collision meshes and attachment lifecycles

### Collision mesh scaling and registration

- **CAD mesh unit scaling in meters**: All collision and visual meshes registered in Intrinsic must be uniformly scaled in meters (1.0 unit = 1.0 m). Importing CAD geometries exported in millimeters (1.0 unit = 1.0 mm) inflates the bounding volume by 10^9x (1,000^3), forcing broadphase collision checkers into octree fallback, creating severe memory pressure, and causing multi-minute planning timeouts. Verify mesh bounding boxes prior to registration.
- **Registering custom geometry via `register_geometry_v1`**: Always call `world.register_geometry_v1(geometry=...)` with an `intrinsic_proto.geometry.CreateGeometryRequest` rather than the legacy `register_geometry` method. The v1 API populates `exact_geometry_ref` in `intrinsic_proto.geometry.v1.GeometryStorageRefs`, which is strictly required for collision checking (`CoalCollisionChecker`) and visual rendering:
```python
from intrinsic.geometry.proto import geometry_service_pb2

create_geo_req = geometry_service_pb2.CreateGeometryRequest()
# Populate create_geo_req.data with mesh or primitive geometry
storage_refs = solution.world.register_geometry_v1(geometry=create_geo_req)
```

### Object reparenting and dynamic collision exclusions

When attaching a workpiece to a robot manipulator in `ObjectWorld`:

- **Flange attachment via `reparent_object_to_final_entity`**: Calling `world.reparent_object(child, world.robot)` attaches the child to the robot base link by default. To attach to the moving tool flange, call `reparent_object_to_final_entity`:
```python
# Attaches workpiece to the active end-effector tool flange
solution.world.reparent_object_to_final_entity(
    child_object=solution.world.workpiece,
    new_parent=solution.world.robot,
)
```
Alternatively, specify the flange coordinate frame explicitly:
```python
flange_frame = solution.world.get_frame("flange", "robot")
solution.world.reparent_object(
    child_object=solution.world.workpiece,
    new_parent=flange_frame,
)
```
- **Dynamic collision exclusions across pick-and-place**:
  - Attaching an object to the robot automatically registers a collision exclusion between the gripper links and the attached entity in `CoalCollisionChecker`.
  - Detaching an object removes the automatic collision exclusion. If pre-existing scene exclusions existed between the gripper and workpiece, detaching may reset them. Ensure adequate non-zero clearance (>= 2 mm) before detaching, or define persistent exclusion rules.

---

## Input-aware decision tree for robot motion selection

Select the appropriate motion primitive based on the operational requirements of the task:

```
[Target motion request]
       │
       ├─► Is the target a predefined robot joint posture (e.g. "home")?
       │     └─► YES: Use Joint-space move (move_robot with MotionSegment.JOINT)
       │
       └─► Is the motion a large displacement in free space requiring collision avoidance?
             └─► YES: Use Trajectory planning (move_robot with MotionSegment.LINEAR / ANY)
```

### Motion primitive decision matrix

| Motion primitive | SBL API entrypoint | Precondition | Verification check | Targeted action |
| :--- | :--- | :--- | :--- | :--- |
| **Joint-space move** | `solution.skills.ai.intrinsic.move_robot(motion_segments=[MotionSegment(joint_position=...)])` | Robot repositioning to known global posture; clearing kinematic singularities. | Validate joint targets are within application limits; start pose is not in collision. | Command joint configuration directly: eliminates inverse kinematics ambiguity and branch jumps. |
| **Trajectory planning** | `solution.skills.ai.intrinsic.move_robot(motion_segments=[MotionSegment(cartesian_pose=...)])` | Free-space Cartesian alignment across non-adjacent poses; obstacles present. | Confirm collision scene is populated; start and goal poses have valid IK solutions. | Plan time-optimal collision-free trajectory; configure `ensure_same_branch=True` for linear segments. |

### Trajectory planning code template

In SBL, `move_robot` accepts a sequence of `MotionSegment` protobuf messages:

```python
move_robot = solution.skills.ai.intrinsic.move_robot

# Define Cartesian target constraint
pose_target = move_robot.intrinsic_proto.motion_planning.v1.PoseEquality(
    moving_frame=solution.world.gripper.tool_frame.transform_node_reference,
    target_frame=solution.world.target_frame.transform_node_reference,
)

# Align robot tool frame with target world frame using motion planning
move_cartesian = move_robot(
    motion_segments=[
        move_robot.intrinsic_proto.skills.MotionSegment(
            cartesian_pose=pose_target,
            motion_type=move_robot.intrinsic_proto.skills.MotionSegment.LINEAR,
        )
    ],
    arm_part=solution.resources.robot,
)
```

### Joint-space configuration move code template

```python
move_robot = solution.skills.ai.intrinsic.move_robot

# Move robot to named joint configuration
move_home = move_robot(
    motion_segments=[
        move_robot.intrinsic_proto.skills.MotionSegment(
            joint_position=solution.world.robot.joint_configurations.home.joint_position,
            motion_type=move_robot.intrinsic_proto.skills.MotionSegment.JOINT,
        )
    ],
    arm_part=solution.resources.robot,
)
```

### Direct motion planner client and inverse kinematics

When computing inverse kinematics or planning trajectories outside Behavior Tree execution, instantiate `MotionPlannerClient` directly with `world_id="world"` and a `MotionPlannerServiceStub`:

```python
from intrinsic.motion_planning.motion_planner_client import MotionPlannerClient
from intrinsic.motion_planning.proto.v1 import motion_planner_service_pb2_grpc
from intrinsic.solutions import worlds

stub = motion_planner_service_pb2_grpc.MotionPlannerServiceStub(
    solution.grpc_channel
)
planner_client = MotionPlannerClient(world_id="world", stub=stub)

# Construct Cartesian target aligning tool frame with target frame
target = worlds.CartesianMotionTarget(
    tool=solution.world.gripper.tool_frame,
    frame=solution.world.target_frame,
).proto

# Compute inverse kinematics for robot
ik_result = planner_client.compute_ik(
    robot_name="ur_module",
    target=target,
)
if ik_result.solutions:
  ik_target = ik_result.solutions[0]
else:
  # Candidate rejection diagnostics (collisions, joint limits, or singularities)
  rejected_candidates = ik_result.ik_debug_information.ik_solutions
```

---

## Trajectory generation, kinematic singularities, and retreat clearance

### Multi-segment branch continuity and wrist flip prevention

- **Diagnosing 2*pi wrist flips on linear segments**: When executing multi-segment motions (such as an approach move followed by a Cartesian linear weld or insertion), an error indicating `maximum_absolute_joint_configuration_error` near 6.283 rad (2*pi) indicates that the initial approach segment selected an IK branch near a joint limit (e.g., wrist joint q_5 near -268 deg). A subsequent linear move requiring further rotation crosses the limit and forces the solver to jump to the +2*pi branch (+92 deg).
- **Enforcing branch continuity in `IKOptions`**:
  - Configure `ensure_same_branch=True` or `prefer_same_branch=True` in `IKOptions` (`intrinsic.motion_planning.motion_planner_client`).
  - Constrain approach waypoint joint bounds away from physical limits, or apply a 180 deg tool symmetry rotation offset around the approach axis where tooling geometry permits.

### Commanded versus sensed joint noise on retreat motions

- **Diagnosing retreat collision failures**: When an approach motion succeeds at a contact target, but the immediate retreat motion fails at `Segment 0` with `Invalid initial joint configuration... collision(s) detected`, the failure stems from the difference between commanded and sensed joint states:
  - The approach motion terminates at a commanded target with sub-millimeter clearance.
  - The retreat motion reads the actual sensed joint configuration from hardware or Gazebo telemetry, where sensor noise or settling variance shifts the forward kinematics by tens of micrometers into fixture collision.
- **Targeted remediation**:
  - Set `plan_using_last_commanded_position=True` on `move_robot` to plan directly from the commanded terminal state rather than the noisy sensed state.
  - Configure per-segment `collision_rules` to exclude the contact pair (`is_excluded: true`) on the departure segment of the retreat motion.
  - Maintain >= 2 mm minimum clearance between free-space approach waypoints and surrounding fixtures to prevent spline interpolation from grazing zero-margin boundaries.

### Avoiding collinear pointing constraints

When applying `PointAtConstraint` in numerical inverse kinematics, ensure the target point is not collinear with the pointing vector at the initial guess or intermediate iterates. Collinear orientations cause the condition number of J*J^T to spike, preventing solver convergence.

---

## System 2 reflection checkpoint before motion and scene updates

Before dispatching robot motions or mutating collision scenes, perform a brief System 2 verification check:

1. Coordinate frames & units: Are all translations in meters and rotations normalized unit quaternions (norm == 1.0)? Are keyword arguments used on Pose3(rotation=..., translation=...)?
2. Simulation synchronization: Was start_from_world_state passed to executive.run() if solution.world was mutated in Python?
3. Kinematic branch continuity: Is the start pose away from joint limits, and is ensure_same_branch=True configured for multi-segment linear moves?
4. Collision geometry validation: Are meshes uniformly scaled in meters, and registered via register_geometry_v1 (exact_geometry_ref populated)?
5. Departure clearance: Does the retreat motion set plan_using_last_commanded_position=True or exclude expected contact pairs on Segment 0 (is_excluded: true)?

---

## Anti-thrashing circuit breaker for motion planner failures

When motion planning (`plan_trajectory` or `move_robot`) or inverse kinematics (`compute_ik`) fails, do not repeat identical planning calls in a loop. Follow this bounded retry protocol:

```
[Motion planning failure detected]
       │
       ▼
 [Attempt 1: Cache bypass]
   - Re-verify target pose coordinates.
   - Set skip_fuzzy_cache_check=True in MotionPlanningOptions to bypass stale cached splines.
       │
       ├─► Success: Continue execution.
       │
       ▼
 [Attempt 2: Constraint and diagnostic triage]
   - Inspect result.ik_debug_information.ik_solutions to identify rejecting constraints.
   - Pull motion planner logs: inctl logs pull --resource motion_planner_service.
   - Check for "Large volume mesh detected" warnings (indicates unscaled CAD geometry).
   - Check for kinematic singularities or joint limit clamps.
       │
       ├─► Success: Continue execution.
       │
       ▼
 [Breakout: Stop motion and isolate root cause]
   - After 2 consecutive failures, STOP all motion commands.
   - Verify application joint limits against physical limits: inctl world reset --address=localhost:17080.
   - Fall back to a known safe joint posture (MotionSegment.JOINT) to clear singularity.
```

---

## Paired safety guardrails

To prevent physical collisions, kinematic faults, and state desynchronization, adhere strictly to these 5 paired safety guardrails:

1. **Explicit keyword arguments on `Pose3`**: Do not pass positional arguments to `Pose3(...)`; always pass explicit keyword arguments `Pose3(rotation=..., translation=...)` to prevent rotation and translation coordinate inversion.
2. **Transform node identity comparison**: Do not compare `TransformNode` instances using `==` (`node_a == node_b` raises `NotImplementedError`); always compare unique identifiers via `node_a.id == node_b.id`.
3. **Collision mesh unit scaling**: Uniformly scale all geometries to meters (1.0 unit = 1.0 m) prior to registration to prevent 10^9x bounding volume expansion and planning timeouts.
4. **Retreat motion departure clearance**: Do not command retreat motions from contact with zero collision margin; set `plan_using_last_commanded_position=True` or configure per-segment collision exclusion rules (`is_excluded: true`) on the departure segment to accommodate sensor settling noise.
5. **Anti-thrashing on planner failure**: Do not retry failed motion planning calls without diagnosing root causes; trip the anti-thrashing circuit breaker after 2 consecutive failures and inspect joint limits, kinematic singularities, or mesh overlaps.
