# Debugging `ObjectWorld`, multi-world synchronization, `TF` trees, and `Gazebo` lockstep

## Multi-world state ownership and synchronization (`init_world`, `world`, `sim_world`)

### Architectural layers and world instances
Live workcell clusters maintain four distinct world instances in `intrinsic_proto.world.ObjectWorldService`:
* **`"init_world"`**: Initial static scene graph configured during solution authoring or asset installation.
* **`"world"`**: Active runtime belief world queried by motion planning, skills, and the executive. Always pass `world_id="world"` for the belief world; passing legacy aliases such as `"exec_world"` or `"belief"` fails with `NOT_FOUND: Could not find world with id: ...`.
* **`"sim_world"`**: Physics simulation world queried by Gazebo (`gzserver`) and simulated hardware module launchers.
* **`"save_monitor"`**: Internal tracking world monitoring unsaved modifications against the solution baseline.

State transitions occur through two mechanisms:
1. **Synchronous gRPC API (`intrinsic_proto.world.ObjectWorldService`)**: Read/write RPCs (`ListObjects`, `GetObject`, `UpdateTransform`, `UpdateObjectJoints`, `ReparentObject`, `CloneWorld`, `DeleteWorld`).
2. **Asynchronous `WorldUpdater`**: Subscribes to `/tf` (`intrinsic_proto.TFMessage`), `/assets/instances/<name>/joint_state`, and `/worlds/<world_id>/world_updates` to asynchronously mutate `"world"` and `"sim_world"`.

### Diagnosing missing `Gazebo` models and `/tmp/intrinsic_icon/*.sock` failures
* **Symptom pattern**: Installing an asset or hardware module succeeds, but simulation startup fails with `NOT_FOUND: Failed to get ModelSdf component for model '<name>'`, or real-time control services fail to connect to domain sockets (`/tmp/intrinsic_icon/<module>.sock: No such file or directory`).
* **Root cause mechanism**: Installing an asset via CLI (`inctl asset install` or `inctl service add`) registers the kinematic object exclusively in `"init_world"`. Because `"sim_world"` is not automatically synchronized, the Gazebo hardware module launcher queries `"sim_world"`, finds only the root object, fails to locate `ModelSdf`, and aborts before creating the real-time domain socket.
* **Dynamic attachment desynchronization (`BELIEF_AND_SIM`)**: Operations such as `attach_object_to_robot` update kinematic attachments exclusively in `"world"`, leaving `"sim_world"` un-updated. When subsequent skills spawn child objects with `create_in_world = BELIEF_AND_SIM` relative to a parent present only in `"world"`, the simulation spawner fails with `Unable to resolve ObjectReference`. Synchronize `"sim_world"` before spawning multi-world children.
* **CLI synchronization**: Run `inctl world reset --address=localhost:17080`. This invokes `intrinsic_proto.conductor.v1.ConductorService/Reset`, which pauses the world updater, clones `"init_world"` -> `"world"`, and clones `"world"` -> `"sim_world"` before restarting simulation. Note that `inctl sim reset` does not exist on the external CLI.
* **Python SBL synchronization (`start_from_world_state`)**:
  * `start_from_world_state=worlds.EditWorldId.INITIAL` (`"init_world"`): Resets both `"world"` and `"sim_world"` to match `"init_world"` before starting execution.
  * `start_from_world_state=worlds.EditWorldId.BELIEF` (`"world"`): Preserves runtime modifications in `"world"` and clones `"world"` into `"sim_world"`. Use this whenever mutating `solution.world` in Python (such as `update_transform` or `create_object`) so simulated skills observe updated poses in physics.
  * `start_from_world_state=None` (default): Executes in the current state without resetting physics.

### Programmatic world cloning via `CloneWorld` and lifecycle contracts
* **Overwriting behavior**: `intrinsic_proto.world.ObjectWorldService/CloneWorld` creates an exact replica of a source world. However, `CloneWorld` does not support an in-place overwrite flag and returns `ALREADY_EXISTS` if the destination `cloned_world_id` already exists.
* **Programmatic synchronization sequence**: When synchronizing or resetting worlds programmatically, call `intrinsic_proto.world.ObjectWorldService/DeleteWorld` on the destination world ID before calling `CloneWorld`.
* **Metadata response field**: `CloneWorld` returns `WorldMetadata` where the unique identifier is stored in `.id` (tag 1), not `.world_id`. Accessing `resp.world_id` raises an attribute error.
* **Sandbox lifecycle cleanup**: When creating speculative planning or kinematic evaluation sandboxes via `CloneWorld`, specify `cloned_world_id=""` with a descriptive `cloned_world_hint="sandbox"`, and ensure `DeleteWorld` is executed inside a `try...finally` block to prevent server octree memory exhaustion.

```python
# Programmatic world synchronization pattern
def sync_init_to_sim(stub):
  try:
    stub.DeleteWorld(object_world_service_pb2.DeleteWorldRequest(world_id="sim_world"))
  except grpc.RpcError as e:
    if e.code() != grpc.StatusCode.NOT_FOUND:
      raise
  clone_resp = stub.CloneWorld(
      object_world_service_pb2.CloneWorldRequest(
          source_world_id="init_world",
          cloned_world_id="sim_world",
          cloned_world_hint="sim_sync",
      )
  )
  return clone_resp.id
```

## Transform tree (`TF`) synchronization, wall-clock monotonicity, and frame parenting

### Movable root entity constraints on `/tf`
* To move a `WorldObject` via `/tf`, the `TransformStamped` message's `child_frame_id` must target the object root entity frame (such as `/object_name/base_link`). Publishing transforms targeting non-root child frames to displace a parent object is rejected by default.
* Root coordinate frames must follow `root/<frame_name>` or `/object_name/entity_name` so the TF echo guard distinguishes world-authoritative frames from external TF publishers.

### Frame parenting, `ObjectWorldClient` `SDK` differences, and static sensor assets
* **Python vs. C++ `ObjectWorldClient` reparenting APIs**:
  * In Python (`ObjectWorldClient.reparent_object`), the method accepts reparenting to either a `WorldObject` (`request.parent_object`) or a `Frame` (`request.parent_frame`).
  * In C++ (`ObjectWorldClient::ReparentObject`), the client only populates `request.mutable_new_parent()` (`WorldObject`). When reparenting directly to a frame in C++, populate `ReparentObjectRequest.parent_frame` directly.
* **`update_world` skill footprint validation**: When using `update_world` to reparent an entity, ensure a valid `ObjectReference` or `FrameReference` is set. Omission causes `INVALID_ARGUMENT: The ObjectReference in the request must be set`.
* **SDF `<frame intrinsic:create_entity="true">` vs. links**: SDF frame entities (such as tool attachment frames) appear in both `object_proto.entities` (`eid_<id>`) and `object_proto.frames` (`ofid_<id>`). The C++ entity world maps both to a single `EntityId` and assigns `COLLECTION_TYPE_ATTACHMENT_FRAMES` or `COLLECTION_TYPE_COORDINATE_FRAMES` without synthetic link components. If an entity error reports `Unable to determine collection type for entity: eid_<id>`, verify that child entities are indexed in `PopulateChildEntities` with `frame_proto != nullptr`.
* **Non-neighboring transform updates**: Always pass `node_to_update=frame_node` when calling `update_transform` between non-neighboring nodes in the scene graph hierarchy so the service identifies which local transform absorbs the delta.
* **Root entity transform overrides**: Transforms (`parent_t_this`) set directly on root entities are ignored (`Root entity <name> has parent_t_this which will be ignored`). Apply initial placement transforms to the spawn frame or child entity attachment instead.
* **Static camera and sensor joint update validation**: Camera and sensor assets lack kinematic actuators and joints. Calling `intrinsic_proto.world.ObjectWorldService/UpdateObjectJoints` on a static sensor asset fails validation (`INVALID_ARGUMENT: UpdateObjectJointsRequest ... contained neither 'joint_positions', 'joint_system_limits' nor 'joint_application_limits'`). Extrinsic camera calibration and sensor pose updates must use `UpdateTransform`.
* **Tool center point and coordinate frame calibration (`FrameCalibrationService`)**: Geometric frame and TCP calibration (`intrinsic_proto.world.FrameCalibrationService`, providing `CalibrateTCP` and `CalibratePose`) are gated behind `platform_level:enterprise` and are not deployed on `platform_level:core` workcells where calls return `StatusCode.UNIMPLEMENTED`. For static workcell sensors and cameras, update extrinsic sensor poses using `UpdateTransform` instead.

### Stale `.pbtxt` overrides in `WorldSyncer`
* **Symptom pattern**: Modifying a robot or tool URDF/SDF with new joint limits or collision geometries has no effect at runtime after pushing a solution package.
* **Root cause mechanism**: Solution package synchronization (`WorldSyncer.pull()`) serializes live world properties into `world_updates/*.pbtxt` (`joint_limits.pbtxt`, `object_properties.pbtxt`, `entity_properties.pbtxt`). On subsequent solution pushes, these `.pbtxt` files are applied on top of the base `SceneObject` template, overriding updated CAD/URDF/SDF assets.
* **Remediation**: Inspect and clean up stale override files in `world_updates/*.pbtxt` when asset definition updates appear ignored.

## `Gazebo` simulation lockstep, step deadlines, and robot oscillations

### Diagnosing the five-second simulator step deadline and camera initialization
* **Step wait timeout**: After a simulation reset, the simulator control service waits up to **5 seconds** for Gazebo to complete **2 simulation steps** before returning `OK`.
* **Perception skill race**: If complex collision mesh decomposition or asset loading takes longer than 5 seconds, the step wait times out, logs `"Failed to step simulation within deadline"`, and returns `OK` prematurely.
* **Diagnostic signature**: Perception skills executed immediately after reset fail with `Tried activating camera, but camera sensor has not yet been initialized` or `Unable to activate sensors`.
* **Remediation**: Check `gzserver` container logs for `"Failed to step simulation within deadline"`, and verify Gazebo has completed at least 2 simulation steps before invoking camera capture skills.

### Distinguishing lockstep deadlocks from physics bottlenecks in `PreUpdate`
Gazebo and real-time control hardware modules advance in lockstep. When simulation hangs or runs slowly:
* **Lockstep deadlocks in `PreUpdate`**: If simulator profiler logs show `PreUpdate` consuming >95% of cycle time (`Physics::Update: 0s`), Gazebo is blocked waiting for real-time control hardware modules to complete their lockstep tick (`WaitForIconTicksToFinish()`).
  * Check for shared memory lockfile contention: `File '/tmp/intrinsic_icon/<module>_hal_module.lock' is locked by another process`.
  * Look for status queue timeouts: `Timed out waiting for first message from Part status queue` or clock synchronization delays `Timed out waiting for empty sim time queue`.
  * Verify real-time controller status with explicit instance routing: `inctl icon status --instance_name=icon --address=localhost:17080`.
  * Enable `--fatal_heartbeat_check=true` on the simulator container runtime to capture a diagnostic backtrace after 30 seconds of stalled timesteps.
* **Physics bottlenecks in `Physics::Update`**: If `Physics::Update` consumes the step budget, convert non-essential `revolute` or `prismatic` joints on passive scene fixtures (such as pedestals or carrier trays) to `fixed` joints, and match Gazebo `cycle_timestep` with the controller frequency (e.g., 2 ms / 500 Hz).

## Input-aware decision trees for world and kinematic debugging

### Decision tree for multi-world desynchronization
```
Precondition: Hardware module, sensor, or scene asset missing or unresolvable
  │
  ├─> Check: Does object exist in "world" and "sim_world"?
  │     │   Run ObjectWorldService/ListObjects across ["init_world", "world", "sim_world"]
  │     │
  │     ├─> Exists only in "init_world":
  │     │     Action: Execute inctl world reset --address=localhost:17080 to clone init -> world -> sim
  │     │
  │     ├─> Exists in "world" but missing in "sim_world" (e.g., BELIEF_AND_SIM spawn failure):
  │     │     Action: Pass start_from_world_state=worlds.EditWorldId.BELIEF in Python SBL
  │     │
  │     └─> Missing across all three worlds:
  │           Action: Re-install asset via inctl asset install or inctl service add
```

### Decision tree for frozen transform tracking and `/tf` drops
```
Precondition: Robot moves but digital twin remains frozen
  │
  ├─> Check: Are world updater logs reporting paused state?
  │     │   Inspect deployment/world logs for "Skipping TF Message while world updater is paused"
  │     │
  │     ├─> Paused state detected:
  │     │     Action: Run inctl world reset --address=localhost:17080 to resume background updater
  │     │
  │     └─> Not paused:
  │           │
  │           ├─> Check: Are transforms rejected with "Update rejected at <time>"?
  │           │     Action: Verify external publisher timestamps are strictly monotonic and NTP-synchronized
  │           │
  │           └─> Check: Is child_frame_id targeting a non-root link?
  │                 Action: Retarget /tf publication to object root entity frame (/object_name/base_link)
```

### Decision tree for simulation lockstep stalls and initialization failures
```
Precondition: Gazebo simulation hangs or camera capture skills fail
  │
  ├─> Check: Did perception fail immediately after reset with "camera sensor has not yet been initialized"?
  │     │
  │     ├─> Step deadline exceeded in gzserver logs:
  │     │     Action: Wait for Gazebo to complete 2 simulation steps before invoking camera capture
  │     │
  │     └─> Step deadline OK:
  │           │
  │           ├─> Profiler shows PreUpdate >95% (Physics::Update: 0s):
  │           │     Action: Clear stale lockfile /tmp/intrinsic_icon/<module>_hal_module.lock and restart icon
  │           │
  │           └─> Robot joints oscillating continuously while idle:
  │                 Action: Open Sim view and set robot mounting table or pedestal to static
```

## `System 2` reflection checkpoints and anti-thrashing circuit breakers

### Pre-mutation `System 2` reflection checklist
Before executing high-cost scene mutations, world resets, or kinematic updates, verify:
1. **Target world identifier validation**: Confirm the target `world_id` is explicitly `"world"`, `"init_world"`, or `"sim_world"` (pass strictly canonical IDs; reject legacy `"belief"` or `"exec_world"`).
2. **Cloning destination verification**: If calling `CloneWorld`, ensure `DeleteWorld` has been invoked on the destination world ID, or a unique sandbox ID is specified.
3. **Sensor asset type check**: Confirm whether the target entity is a static sensor or camera, ensuring pose updates use `UpdateTransform` rather than `UpdateObjectJoints`.
4. **Transform hierarchy verification**: If updating non-neighboring scene graph nodes, confirm that `node_to_update` is populated to absorb the relative offset.
5. **State preservation requirement**: Assess whether `inctl world reset` will overwrite uncommitted belief world modifications; if runtime edits in `"world"` must be preserved, use `start_from_world_state=worlds.EditWorldId.BELIEF`.

### Anti-thrashing circuit breakers
1. **World update rejection limit**: If `/tf` updates or gRPC transform mutations are rejected twice consecutively, stop immediately. Transition to inspecting `world` pod logs for paused state or timestamp preemption rather than reissuing identical mutations.
2. **Simulation step timeout limit**: If Gazebo fails to step within the 5-second deadline across two consecutive resets, halt execution. Inspect mesh complexity on custom assets or check for lockfile contention in `/tmp/intrinsic_icon/`.
3. **Cloning conflict breaker**: If `CloneWorld` returns `ALREADY_EXISTS`, halt immediate re-invocation. Query `ListWorlds`, execute `DeleteWorld` on the target ID, or generate a fresh sandbox UUID.

## Paired safety guardrails

### Five core negative constraints with paired affirmative actions
1. **Do not pass an existing destination world ID directly to `CloneWorld` without deletion**; always call `DeleteWorld` on the target destination ID prior to invoking `CloneWorld` to prevent `ALREADY_EXISTS` errors.
2. **Do not publish `/tf` transforms targeting non-root child frames to displace a parent `WorldObject`**; always publish transforms targeting the object root entity frame (such as `/object_name/base_link`) with strictly monotonically increasing timestamps (`header.stamp`).
3. **Do not call `UpdateObjectJoints` on static camera or sensor assets lacking kinematic actuators**; always update extrinsic sensor poses via `UpdateTransform` to satisfy validation.
4. **Do not omit `node_to_update` when invoking `update_transform` between non-neighboring nodes in the scene graph**; always specify the target child or intermediate node whose local transform absorbs the relative delta.
5. **Do not execute simulation resets via deprecated commands such as `inctl sim reset` or unconstrained manual loops**; always use `inctl world reset --address=localhost:17080` or pass `start_from_world_state` via Python `SBL`.
