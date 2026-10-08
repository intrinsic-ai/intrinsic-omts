# SBL Skills, Parameter Protos, and Discovery Reference

This reference documents how to discover, inspect, and construct skill
parameters and nested protobuf messages in open-source Intrinsic Core
(`@intrinsic-core` and `@intrinsic_apis`) and the Open Machine Tending Solution
(`intrinsic-omts`).

---

## 1. Open-Source Skill & Protobuf Discovery Workflow

### 1.1 Locating `@intrinsic-core`, `@intrinsic_apis`, and Release Bundles

In `intrinsic-omts` (`MODULE.bazel`), Intrinsic Core dependencies and
prebuilt skill bundles are fetched via Bazel Bzlmod into Bazel's `output_base`:
- `@intrinsic-core`: SBL SDK internals (`intrinsic/solutions/internal/...`),
  skill implementations and manifests (`intrinsic/skills/...`,
  `intrinsic/manipulation/skills/...`, `intrinsic/world/skills/...`,
  `intrinsic_motion_planning/...`, `intrinsic_perception/...`,
  `intrinsic_control/...`), and internal protos.
- `@intrinsic_apis`: Public API `.proto` definitions (`intrinsic/world/proto/...`,
  `intrinsic/motion_planning/proto/v1/...`, `intrinsic/manipulation/proto/v1/...`,
  `intrinsic/perception/proto/v1/...`, `intrinsic/icon/proto/...`,
  `intrinsic/math/proto/...`).
- `@<name>_bundle`: Prebuilt skill/service `.bundle.tar` archives fetched via
  `use_extension("//bazel:gh_release.bzl", "gh_release")` (e.g.,
  `@hand_e_gripper_cmd_skill_bundle`).

```bash
# Fetch only skill manifests and proto descriptors (fast, no container images):
bazel fetch //:omts_solution_manifest

# Resolve Bazel's external directory on local disk:
EXT="$(bazel info output_base)/external"
# $EXT/intrinsic-core+/    -> @intrinsic-core source tree
# $EXT/intrinsic_apis+/    -> @intrinsic_apis proto tree
# $EXT/+gh_release+<name>/ -> prebuilt .bundle.tar archives
```

### 1.2 Finding Skill Manifests, `.proto` Files, and Bazel `_py_pb2` Targets

```bash
# 1. List all skill manifests in @intrinsic-core (skill IDs, parameter/return
#    message names, and required equipment capabilities):
find "$EXT/intrinsic-core+" \
  -name "*manifest.textproto" -o -name "*.manifest.textproto"

# 2. Search for a specific proto message or enum definition across both repos:
grep -rn "message CollisionRule\|message PoseEquality\|enum MotionType" \
  "$EXT/intrinsic_apis+" "$EXT/intrinsic-core+"

# 3. Query Bazel for the proto_library and py_proto_library targets:
bazel query "attr(srcs, 'collision_settings.proto', @intrinsic_apis//...)"
# -> @intrinsic_apis//intrinsic/world/proto:collision_settings_proto
bazel query "kind(py_proto_library, @intrinsic_apis//intrinsic/world/proto:*)"
# -> @intrinsic_apis//intrinsic/world/proto:collision_settings_py_pb2
```

### 1.3 Inspecting Prebuilt `.bundle.tar` Skills (No `.proto` Source File)

Skills imported via `imported_asset_bundle` in `BUILD` (such as
`//:hand_e_gripper_cmd_skill` from `@hand_e_gripper_cmd_skill_bundle//file`)
ship as `.bundle.tar` archives containing binary `skill_manifest.binpb` and
`descriptors-transitive-descriptor-set.proto.bin` files rather than `.proto`
source files. Inspect their descriptors directly in Python:

```python
import subprocess
import tarfile

from google.protobuf import descriptor_pb2

output_base = subprocess.check_output(
  ["bazel", "info", "output_base"], text=True
).strip()
bundle_path = (
  f"{output_base}/external/+gh_release+hand_e_gripper_cmd_skill_bundle/"
  "hand_e_gripper_cmd_skill.bundle.tar"
)

with tarfile.open(bundle_path) as tf:
  raw_fds = tf.extractfile("descriptors-transitive-descriptor-set.proto.bin")
  fds = descriptor_pb2.FileDescriptorSet.FromString(raw_fds.read())
  for fd in fds.file:
    if "gripper" in fd.name:
      print(f"File: {fd.name} (package {fd.package})")
      for msg in fd.message_type:
        fields = [(f.name, f.type_name or f.type) for f in msg.field]
        print(f"  message {msg.name}: {fields}")
```

---

## 2. How SBL Maps Skill Protos to Python Keyword Arguments

### 2.1 Build-Time Asset Info (`solution_manifest.bzl` & `.asset_info.binpb`)

1. Each `cc_skill`, `py_skill` (in `@intrinsic-core`), or
   `imported_asset_bundle(asset_type="ASSET_TYPE_SKILL")` (in
   `bazel/imported_asset.bzl`) runs
   `@intrinsic-core//intrinsic/assets/build_defs:assetlocalinfogen` to generate
   `<target>.asset_info.binpb` (`intrinsic_proto.assets.build_defs.AssetInfo`),
   embedding the `SkillManifest` (`id`, `parameter.message_full_name`,
   `return_type.message_full_name`, `dependencies.required_equipment`) and a
   transitive `google.protobuf.FileDescriptorSet`.
2. `solution_manifest_only(name = "omts_solution_manifest", ...)` in
   `bazel/solution_manifest.bzl` (using `_OMTS_SKILL_ASSETS` from `BUILD`)
   exposes all `.asset_info.binpb` files to `tests/unit/test_behaviors.py`
   (`_load_real_skill_infos()`) so unit tests generate the exact production SBL
   skill classes offline.

### 2.2 Runtime Class Generation (`skill_generation.py` & `skill_utils.py`)

When `solution.skills.ai.intrinsic.<skill_name>` is accessed, SBL executes
`skill_generation.gen_skill_class(info, compatible_resources)` (in
`@intrinsic-core//intrinsic/solutions/internal/skill_generation.py`):
1. **Hermetic Descriptor Pool**: `SkillInfoImpl` builds an isolated
   `descriptor_pool.DescriptorPool` and raw protobuf class map from the skill's
   `FileDescriptorSet`.
2. **Nested `MessageWrapper` & Enum Hierarchy**:
   `skill_utils.update_message_class_modules()` wraps every reachable protobuf
   `Message` in a `MessageWrapper` class and every `Enum` in an `enum.IntEnum`,
   attaching them to `<skill_cls>` along a namespace tree matching the
   **protobuf `package` declaration** (not the `.proto` file path):
   - `package intrinsic_proto.world;` ->
     `<skill_cls>.intrinsic_proto.world.<Message>`
   - `package intrinsic_proto.motion_planning.v1;` ->
     `<skill_cls>.intrinsic_proto.motion_planning.v1.<Message>`
   - `package ai.intrinsic;` (in `gripper_cmd_skill.proto` and
     `create_object.proto`) -> `<skill_cls>.ai.intrinsic.<Message>`
   - **Enum Shortcuts**: Enum values are attached on the enum class
     (`move_robot.intrinsic_proto.skills.MotionSegment.MotionType.LINEAR`), on
     the parent message (`move_robot.intrinsic_proto.skills.MotionSegment.LINEAR`),
     and on `<skill_cls>` itself (`move_robot.LINEAR`, `move_robot.ANY`,
     `move_robot.JOINT`) when no name collision exists.
3. **Keyword-Only `__init__` Parameters**:
   - **Proto Fields**: Every top-level field in `parameter_descriptor().fields`
     becomes a keyword-only argument defaulting to `None`.
   - **Equipment Resource Slots**: Every key in `dependencies.required_equipment`
     (`info.resource_selectors`) becomes a keyword-only argument named
     `<slot_name>` (or `<slot_name>_resource` on name collision). **Default
     Resolution Rule**: If `len(compatible_resources[slot]) == 1` in the cell,
     SBL sets that single `ResourceHandle` as the default value; if `0` or `>1`
     compatible resources exist, the parameter has **no default** and must be
     passed explicitly.
   - **Return Value Key**: `return_value_key: str | None = None` (present iff
     the skill manifest declares a `return_type`).

### 2.3 Automatic Pythonic-to-Proto Conversions (`skill_utils.py`)

| Target Protobuf Full Name | Accepted Pythonic SDK Type(s) |
|---|---|
| `intrinsic_proto.Pose` | `intrinsic.solutions.data_types.Pose3` |
| `geometry_msgs.msg.Pose` | `intrinsic.solutions.data_types.Pose3` |
| `intrinsic_proto.icon.JointVec` | `object_world_resources.JointConfiguration` (e.g. `arm.joint_configurations.home`), or `JointVec(joints=[...])` |
| `intrinsic_proto.world.CollisionSettings` | `intrinsic.solutions.worlds.CollisionSettings` |
| `intrinsic_proto.world.ObjectReference` | `object_world_resources.WorldObject` (e.g. `solution.world.ur_module`) |
| `intrinsic_proto.world.FrameReference` | `object_world_resources.Frame` (e.g. `solution.world.root.view_frame`) |
| `intrinsic_proto.world.TransformNodeReference` | `object_world_resources.TransformNode` (`WorldObject` or `Frame`) |
| `intrinsic_proto.world.ObjectOrEntityReference` | `object_world_resources.WorldObject` |
| `google.protobuf.Duration` | `datetime.timedelta`, `float`, or `int` (interpreted as **seconds**) |
| `intrinsic_proto.perception.v1.PoseEstimatorId` | `intrinsic.solutions.pose_estimation.PoseEstimatorId` |
| `intrinsic_proto.world.RobotPayload` | `intrinsic.solutions.robot_payload.RobotPayload` |
| `intrinsic_proto.assets.v1.ResolvedDependency` | `provided.ResourceHandle` (e.g. `solution.resources.raw_stock_2x3x5`) |
| **Any message `M`** | Any static `_pb2` `Message` with `DESCRIPTOR.full_name == M` (cross-pool serialized via `SerializeToString()` / `ParseFromString()`) |

### 2.4 Programmatic Introspection in Python

Given `skill_cls = solution.skills.ai.intrinsic.<skill_name>` and an instance
`action = skill_cls(...)`:

| Expression | Return Type & Purpose |
|---|---|
| `inspect.signature(skill_cls)` | `inspect.Signature` of `__init__` with all proto fields, resource slots, types, and defaults. |
| `print(skill_cls.__init__.__doc__)` | Auto-generated docstring listing accepted proto wrapper paths, enums, required equipment capabilities, field descriptions, and return fields. |
| `inspect.signature(skill_cls.intrinsic_proto.<pkg>.<Msg>)` | `inspect.Signature` of any nested `MessageWrapper` class. |
| `skill_cls.info.id` | Skill ID string (e.g. `"ai.intrinsic.move_robot"`). |
| `skill_cls.info.parameter_message_full_name` | Full parameter proto name (e.g. `"intrinsic_proto.skills.MoveRobotParams"`). |
| `skill_cls.info.return_value_message_full_name` | Full return proto name (or `None`). |
| `skill_cls.info.parameter_descriptor()` | `google.protobuf.descriptor.Descriptor` for the parameter proto (`.fields_by_name`, `.oneofs_by_name`). |
| `skill_cls.info.return_value_descriptor()` | `google.protobuf.descriptor.Descriptor` for the return proto (or `None`). |
| `skill_cls.info.field_names` | `set[str]` of top-level parameter field names. |
| `skill_cls.info.resource_selectors` | `dict[str, equipment_pb2.ResourceSelector]` mapping equipment slot name -> required `capability_names`. |
| `skill_cls.info.create_param_message()` | Instantiates a raw parameter protobuf message populated with manifest defaults. |
| `skill_cls.compatible_resources` | `SkillCompatibleResourcesMap` mapping each equipment slot to compatible `ResourceHandle`s. |
| `skill_cls.message_classes` | `dict[str, type[Message]]` mapping every `DESCRIPTOR.full_name` in the skill's pool to its raw protobuf class. |
| `action.proto` | `behavior_call_pb2.BehaviorCall` (`skill_id`, `parameters` `Any`, `resources`, `assignments`, `return_value_name`). |
| `action.result` | `BlackboardValue` proxy supporting `.field` and `[index]` to build CEL expressions for downstream skills. |

### 2.5 The 4-Way Protobuf Translation Rules & Master Table

1. **Full Protobuf Name**: `<package>.<Message>[.<NestedMessage>]` from the
   `.proto` file's `package` statement (not the directory path).
2. **Dynamic SBL Wrapper**: `<skill_cls>.<Full Protobuf Name>(...)`. Requires
   zero proto imports or extra Bazel `deps`.
3. **Static Python `_pb2` Import**: Convert `.proto` file path
   `path/to/foo.proto` -> `from path.to import foo_pb2` and access
   `foo_pb2.<Message>[.<NestedMessage>]`. Static `_pb2` messages can be passed
   directly to SBL skill kwargs.
4. **Bazel `deps` Target**: `@intrinsic_apis//path/to:foo_py_pb2` or
   `@intrinsic-core//path/to:foo_py_pb2`.

| Full Protobuf Name | Dynamic SBL Path on `<skill_cls>` | Static Python `_pb2` Import & Symbol | Bazel `py_proto_library` Target |
|---|---|---|---|
| `intrinsic_proto.skills.MoveRobotParams` | *(top-level kwargs of `move_robot(...)`)* | `from intrinsic.motion_planning.skills import move_robot_pb2`<br>`move_robot_pb2.MoveRobotParams` | `@intrinsic-core//intrinsic_motion_planning/intrinsic/motion_planning/skills:move_robot_py_pb2` |
| `intrinsic_proto.skills.MotionSegment` | `move_robot.intrinsic_proto.skills.MotionSegment` | `move_robot_pb2.MotionSegment` | `@intrinsic-core//intrinsic_motion_planning/intrinsic/motion_planning/skills:move_robot_py_pb2` |
| `intrinsic_proto.skills.MotionSegment.MotionType` | `move_robot.intrinsic_proto.skills.MotionSegment.MotionType` *(or `move_robot.ANY`, `.LINEAR`, `.JOINT`)* | `move_robot_pb2.MotionSegment.MotionType` | `@intrinsic-core//intrinsic_motion_planning/intrinsic/motion_planning/skills:move_robot_py_pb2` |
| `intrinsic_proto.motion_planning.v1.PoseEquality` | `move_robot.intrinsic_proto.motion_planning.v1.PoseEquality` | `from intrinsic.motion_planning.proto.v1 import geometric_constraints_pb2`<br>`geometric_constraints_pb2.PoseEquality` | `@intrinsic_apis//intrinsic/motion_planning/proto/v1:geometric_constraints_py_pb2` |
| `intrinsic_proto.motion_planning.v1.PositionEquality` | `move_robot.intrinsic_proto.motion_planning.v1.PositionEquality` | `geometric_constraints_pb2.PositionEquality` | `@intrinsic_apis//intrinsic/motion_planning/proto/v1:geometric_constraints_py_pb2` |
| `intrinsic_proto.motion_planning.v1.RotationCone` | `move_robot.intrinsic_proto.motion_planning.v1.RotationCone` | `geometric_constraints_pb2.RotationCone` | `@intrinsic_apis//intrinsic/motion_planning/proto/v1:geometric_constraints_py_pb2` |
| `intrinsic_proto.motion_planning.v1.ConstraintIntersection` | `move_robot.intrinsic_proto.motion_planning.v1.ConstraintIntersection` | `geometric_constraints_pb2.ConstraintIntersection` | `@intrinsic_apis//intrinsic/motion_planning/proto/v1:geometric_constraints_py_pb2` |
| `intrinsic_proto.motion_planning.v1.RelativePoseEquality` | `move_robot.intrinsic_proto.motion_planning.v1.RelativePoseEquality` | `geometric_constraints_pb2.RelativePoseEquality` | `@intrinsic_apis//intrinsic/motion_planning/proto/v1:geometric_constraints_py_pb2` |
| `intrinsic_proto.motion_planning.v1.GeometricConstraint` | `move_robot.intrinsic_proto.motion_planning.v1.GeometricConstraint` | `geometric_constraints_pb2.GeometricConstraint` | `@intrinsic_apis//intrinsic/motion_planning/proto/v1:geometric_constraints_py_pb2` |
| `intrinsic_proto.motion_planning.v1.DynamicCartesianLimits` | `move_robot.intrinsic_proto.motion_planning.v1.DynamicCartesianLimits` | `from intrinsic.motion_planning.proto.v1 import motion_planning_limits_pb2`<br>`motion_planning_limits_pb2.DynamicCartesianLimits` | `@intrinsic_apis//intrinsic/motion_planning/proto/v1:motion_planning_limits_py_pb2` |
| `intrinsic_proto.motion_planning.v1.JointLimitsUpdate` | `move_robot.intrinsic_proto.motion_planning.v1.JointLimitsUpdate` | `motion_planning_limits_pb2.JointLimitsUpdate` | `@intrinsic_apis//intrinsic/motion_planning/proto/v1:motion_planning_limits_py_pb2` |
| `intrinsic_proto.motion_planning.v1.BlendingParameters` | `move_robot.intrinsic_proto.motion_planning.v1.BlendingParameters` | `from intrinsic.motion_planning.proto.v1 import motion_blending_parameter_pb2`<br>`motion_blending_parameter_pb2.BlendingParameters` | `@intrinsic_apis//intrinsic/motion_planning/proto/v1:motion_blending_parameter_py_pb2` |
| `intrinsic_proto.world.CollisionSettings` | `move_robot.intrinsic_proto.world.CollisionSettings` | `from intrinsic.world.proto import collision_settings_pb2`<br>`collision_settings_pb2.CollisionSettings` | `@intrinsic_apis//intrinsic/world/proto:collision_settings_py_pb2` |
| `intrinsic_proto.world.CollisionSettings.CollisionRule` | `move_robot.intrinsic_proto.world.CollisionSettings.CollisionRule` | `collision_settings_pb2.CollisionSettings.CollisionRule` | `@intrinsic_apis//intrinsic/world/proto:collision_settings_py_pb2` |
| `intrinsic_proto.world.ObjectOrEntityReference` | `move_robot.intrinsic_proto.world.ObjectOrEntityReference` | `collision_settings_pb2.ObjectOrEntityReference` | `@intrinsic_apis//intrinsic/world/proto:collision_settings_py_pb2` |
| `intrinsic_proto.world.CollisionAction` | `move_robot.intrinsic_proto.world.CollisionAction` | `from intrinsic.world.proto import collision_action_pb2`<br>`collision_action_pb2.CollisionAction` | `@intrinsic_apis//intrinsic/world/proto:collision_action_py_pb2` |
| `intrinsic_proto.world.CollisionMarginPair` | `move_robot.intrinsic_proto.world.CollisionMarginPair` | `collision_action_pb2.CollisionMarginPair` | `@intrinsic_apis//intrinsic/world/proto:collision_action_py_pb2` |
| `intrinsic_proto.world.ObjectReference` | `<skill>.intrinsic_proto.world.ObjectReference` | `from intrinsic.world.proto import object_world_refs_pb2`<br>`object_world_refs_pb2.ObjectReference` | `@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2` |
| `intrinsic_proto.world.ObjectReferenceByName` | `<skill>.intrinsic_proto.world.ObjectReferenceByName` | `object_world_refs_pb2.ObjectReferenceByName` | `@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2` |
| `intrinsic_proto.world.FrameReference` | `<skill>.intrinsic_proto.world.FrameReference` | `object_world_refs_pb2.FrameReference` | `@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2` |
| `intrinsic_proto.world.TransformNodeReference` | `<skill>.intrinsic_proto.world.TransformNodeReference` | `object_world_refs_pb2.TransformNodeReference` | `@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2` |
| `intrinsic_proto.world.TransformNodeReferenceByName` | `<skill>.intrinsic_proto.world.TransformNodeReferenceByName` | `object_world_refs_pb2.TransformNodeReferenceByName` | `@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2` |
| `intrinsic_proto.world.ObjectWorldUpdates` | `update_world.intrinsic_proto.world.ObjectWorldUpdates` | `from intrinsic.world.proto import object_world_updates_pb2`<br>`object_world_updates_pb2.ObjectWorldUpdates` | `@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2` |
| `intrinsic_proto.icon.JointVec` | `move_robot.intrinsic_proto.icon.JointVec` | `from intrinsic.icon.proto import joint_space_pb2`<br>`joint_space_pb2.JointVec` | `@intrinsic_apis//intrinsic/icon/proto:joint_space_py_pb2` |
| `intrinsic_proto.Pose` | `<skill>.intrinsic_proto.Pose` *(or `Pose3`)* | `from intrinsic.math.proto import pose_pb2`<br>`pose_pb2.Pose` | `@intrinsic_apis//intrinsic/math/proto:pose_py_pb2` |
| `intrinsic_proto.Point` | `<skill>.intrinsic_proto.Point` | `from intrinsic.math.proto import point_pb2`<br>`point_pb2.Point` | `@intrinsic_apis//intrinsic/math/proto:point_py_pb2` |
| `intrinsic_proto.Quaternion` | `<skill>.intrinsic_proto.Quaternion` | `from intrinsic.math.proto import quaternion_pb2`<br>`quaternion_pb2.Quaternion` | `@intrinsic_apis//intrinsic/math/proto:quaternion_py_pb2` |
| `intrinsic_proto.Vector3` | `<skill>.intrinsic_proto.Vector3` | `from intrinsic.math.proto import vector3_pb2`<br>`vector3_pb2.Vector3` | `@intrinsic_apis//intrinsic/math/proto:vector3_py_pb2` |
| `intrinsic_proto.perception.v1.PoseEstimateInRoot` | `estimate_pose_multi_view.intrinsic_proto.perception.v1.PoseEstimateInRoot` | `from intrinsic.perception.proto.v1 import pose_estimate_in_root_pb2`<br>`pose_estimate_in_root_pb2.PoseEstimateInRoot` | `@intrinsic_apis//intrinsic/perception/proto/v1:pose_estimate_in_root_py_pb2` |
| `ai.intrinsic.JointState` | `gripper_cmd_skill.ai.intrinsic.JointState` | *(Prebuilt bundle descriptor; use dynamic wrapper)* | N/A (`@hand_e_gripper_cmd_skill_bundle`) |

---

## 3. Core Skills & Nested Proto Schemas

### 3.1 `move_robot` (`ai.intrinsic.move_robot`)

- **Proto**: `$EXT/intrinsic-core+/intrinsic_motion_planning/intrinsic/motion_planning/skills/move_robot.proto`
- **Manifest**: `$EXT/intrinsic-core+/incode/motion_planning/skills/move_robot_manifest.textproto`
- **Bazel Target**: `@intrinsic-core//incode/motion_planning/skills:move_robot_skill`
- **Equipment Slots**: `motion_planner_service` (`MotionPlannerService`),
  `world_service` (`ObjectWorldService`)

#### `MoveRobotParams` & `MotionSegment` Fields
- **Top-Level `MoveRobotParams` kwargs**:
  - `motion_segments: Sequence[MotionSegment]` (field 1, **required**): Ordered
    trajectory waypoints/segments.
  - `planning_parameters: MotionPlanningParameters` (field 2),
    `execution_parameters: MotionExecutionParameters` (field 3).
  - `curve_parameters: BlendingParameters` (field 8): Trajectory blending config
    (`cartesian_blending=CartesianBlendingParameters(...)`,
    `joint_blending=JointBlendingParameters(...)` from
    `motion_blending_parameter.proto`).
  - `arm_part: ObjectReference | WorldObject` (field 9, **required**): Robot arm
    world node (e.g. `solution.world.ur_module`).
  - `plan_using_last_commanded_position: bool` (field 10),
    `motion_planner_service: ResourceHandle` (field 12).
  - **Return proto**: `MoveRobotReturnValue`.
- **`MotionSegment` fields (`intrinsic_proto.skills.MotionSegment`)**:
  1. `motion_type`: `MotionSegment.MotionType.ANY` (`0`), `LINEAR` (`1`), or
     `JOINT` (`2`).
  2. **`waypoint` `oneof`** (must set exactly one):
     - `joint_position: JointVec`: Target joints in radians (`JointVec(joints=[...])`
       or `JointConfiguration`).
     - `cartesian_pose: PoseEquality`: Strict 6D pose alignment with
       `moving_frame: TransformNodeReference` (robot tool frame),
       `target_frame: TransformNodeReference` (target world frame), and optional
       `target_frame_offset: Pose`.
     - `constraint_intersection: ConstraintIntersection`: Combines multiple
       `GeometricConstraint` clauses, such as `PositionEquality`
       (`moving_frame`, `target_frame`, `moving_frame_offset: Point`,
       `target_frame_offset: Point`) and `RotationCone` (`moving_frame`,
       `target_frame`, `moving_axis: Vector3`, `target_axis: Vector3`,
       `cone_opening_half_angle: float`) for relaxed tool-Z rotation.
     - `relative_cartesian_pose: RelativePoseEquality`: Translational/pose offset
       with `moving_frame: TransformNodeReference`, `relative_pose: Pose`, and
       optional `reference_frame: TransformNodeReference`.
  3. `path_constraints: GeometricConstraint`: Optional path-wide constraint.
  4. `collision_settings: CollisionSettings`:
     - `disable_collision_checking: bool`: **Never set `True` in `omts`**; always
       set `False` and pass segment-scoped `collision_rules`.
     - `collision_rules: Sequence[CollisionSettings.CollisionRule]`:
       `left: Sequence[ObjectOrEntityReference]` (field 1),
       `right: Sequence[ObjectOrEntityReference]` (field 2), and
       `collision_action: CollisionAction` (field 5) where `CollisionAction` has
       `oneof action { bool is_excluded = 3; CollisionMarginPair margin = 4; }`
       (e.g. `CollisionAction(is_excluded=True)`).
     - **Post-`move_to_contact` & Support-Surface Exclusions (`StatusCode: ai.intrinsic.move_robot:10201`)**:
       After compliant touchdown (`move_to_contact`), both the workpiece and
       gripper fingers (`tool_object_name`) can be in contact with the support
       surface (`vise_object_name` or `robot.enclosure_object_name`). Always
       exclude `(tool, workpiece)`, `(support_surface, workpiece)`, and
       `(tool, support_surface)` on approach and post-touchdown retract segments.
     - **Out-of-Envelope IK (`StatusCode: ai.intrinsic.move_robot:10301`)**:
       Indicates the target frame is unreachable given the arm's base pose
       (`align_robot.updates.pbtxt`), kinematic reach, or orientation
       constraints.
  5. `cartesian_limits: DynamicCartesianLimits` and
     `joint_limits: JointLimitsUpdate` (from `motion_planning_limits.proto`).

#### Example A: Dynamic `move_robot.intrinsic_proto` Style (`src/hardware/robot.py`)

```python
from intrinsic.solutions import behavior_tree as bt


def build_retract_with_collision_exclusion(solution):
  move_robot = solution.skills.ai.intrinsic.move_robot
  wp = move_robot.intrinsic_proto
  rules = [
    wp.world.CollisionSettings.CollisionRule(
      left=[
        wp.world.ObjectOrEntityReference(
          object=wp.world.ObjectReference(
            by_name=wp.world.ObjectReferenceByName(object_name="workpiece")
          )
        )
      ],
      right=[
        wp.world.ObjectOrEntityReference(
          object=wp.world.ObjectReference(
            by_name=wp.world.ObjectReferenceByName(
              object_name="infeed_3d_printed"
            )
          )
        )
      ],
      collision_action=wp.world.CollisionAction(is_excluded=True),
    )
  ]
  tool_ref = wp.world.TransformNodeReference(
    by_name=wp.world.TransformNodeReferenceByName(
      object_name="ur_module",
      frame_name="tool_frame",
    )
  )
  segment = wp.skills.MotionSegment(
    motion_type=wp.skills.MotionSegment.MotionType.LINEAR,
    relative_cartesian_pose=wp.motion_planning.v1.RelativePoseEquality(
      moving_frame=tool_ref,
      relative_pose=wp.Pose(
        position=wp.Point(x=0.0, y=0.0, z=-0.05),
        orientation=wp.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
      ),
    ),
    collision_settings=wp.world.CollisionSettings(
      disable_collision_checking=False,
      collision_rules=rules,
    ),
  )
  return bt.Task(
    action=move_robot(
      arm_part=wp.world.ObjectReference(
        by_name=wp.world.ObjectReferenceByName(object_name="ur_module")
      ),
      motion_segments=[segment],
    ),
    name="Linear Retract (5.0 cm, -Z Tool)",
  )
```

#### Example B: Static `_pb2` Import Style (`src/hardware/robot.py`)

```python
from intrinsic.math.proto import point_pb2, pose_pb2, quaternion_pb2
from intrinsic.motion_planning.proto.v1 import geometric_constraints_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.world.proto import (
  collision_action_pb2,
  collision_settings_pb2,
  object_world_refs_pb2,
)


def build_cartesian_move_static_pb2(solution):
  move_robot = solution.skills.ai.intrinsic.move_robot
  seg_cls = move_robot.intrinsic_proto.skills.MotionSegment
  pose_eq = geometric_constraints_pb2.PoseEquality(
    moving_frame=object_world_refs_pb2.TransformNodeReference(
      by_name=object_world_refs_pb2.TransformNodeReferenceByName(
        object_name="ur_module",
        frame_name="tool_frame",
      )
    ),
    target_frame=object_world_refs_pb2.TransformNodeReference(
      by_name=object_world_refs_pb2.TransformNodeReferenceByName(
        object_name="root",
        frame_name="view_frame",
      )
    ),
    target_frame_offset=pose_pb2.Pose(
      position=point_pb2.Point(x=0.0, y=0.0, z=-0.05),
      orientation=quaternion_pb2.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    ),
  )
  coll = collision_settings_pb2.CollisionSettings(
    disable_collision_checking=False,
    collision_rules=[
      collision_settings_pb2.CollisionSettings.CollisionRule(
        left=[
          collision_settings_pb2.ObjectOrEntityReference(
            object=object_world_refs_pb2.ObjectReference(
              by_name=object_world_refs_pb2.ObjectReferenceByName(
                object_name="workpiece"
              )
            )
          )
        ],
        right=[
          collision_settings_pb2.ObjectOrEntityReference(
            object=object_world_refs_pb2.ObjectReference(
              by_name=object_world_refs_pb2.ObjectReferenceByName(
                object_name="vise"
              )
            )
          )
        ],
        collision_action=collision_action_pb2.CollisionAction(
          is_excluded=True,
        ),
      )
    ],
  )
  return bt.Task(
    action=move_robot(
      arm_part=solution.world.ur_module,
      motion_segments=[
        seg_cls(
          motion_type=seg_cls.MotionType.LINEAR,
          cartesian_pose=pose_eq,
          collision_settings=coll,
        )
      ],
    ),
    name="Linear Approach to Standoff (root/view_frame)",
  )
```

---

### 3.2 `move_to_contact` (`ai.intrinsic.move_to_contact`)

- **Proto**: `$EXT/intrinsic-core+/intrinsic/manipulation/skills/force/move_to_contact.proto`
  (`package intrinsic_proto.manipulation.skills`,
  `from intrinsic.manipulation.skills.force import move_to_contact_pb2`)
- **Manifest & Implementation**:
  `$EXT/intrinsic-core+/intrinsic/manipulation/skills/force/move_to_contact_manifest.textproto`
  and `move_to_contact.py`
- **Bazel Target**: `@intrinsic-core//intrinsic/manipulation/skills/force:move_to_contact_skill`
- **Equipment Slot**: `icon2`
- **Parameters (`MoveToContactParams`)**: `tool: TransformNodeReference` (field
  1), `fixed_vector: FixedVector(direction=Vector3(...), reference=...)` (field
  2) or `target: TransformNodeReference` (field 3), `contact_force: float`
  (field 4, Newtons `> 0`), `timeout_sec: float` (field 7, seconds),
  `max_approach_velocity: float` (field 14, m/s).
- **Status Codes (`move_to_contact_manifest.textproto`)**:
  - `10201` (`PARAMETER_ERROR_CODE`): `"Invalid parameterization"`
  - `10301` (`STABILIZE_ERROR_CODE`): `"Stabilization timeout"` —
    `"Stabilize action timed out without making contact. Force is ... N."`
  - `10302` (`APPROACH_TIMEOUT_ERROR_CODE`): `"Approach timeout"` —
    `"Approach action timed out without making contact."`
  - `10303` (`TIMEOUT_ERROR_CODE`): `"Move to contact timeout"`
- **4-Stage ICON State Machine & Gazebo Simulation Recovery (`bt.Fallback` + `bt.Retry`)**:
  - `move_to_contact` runs `ActionId.TARE (0)` -> `ActionId.APPROACH (1)`
    (`MakeContact` until `ELAPSED_TIME_SECONDS > 0.05s` and
    `SENSED_FORCE > stop_switching_force`) -> `ActionId.STABILIZE (2)`
    (`MakeContact` with `disable_reference_lowpass_filter=True` waiting for
    `SENSED_FORCE > stop_switching_force` and `IS_SETTLED == True` within
    `SETTLING_TIMEOUT_S = 2.0s`) -> `ActionId.JOINT_STOP (4)`.
  - When `STABILIZE` times out after `2.0s` with
    `sensed_force_magnitude <= min_contact_force` (`1.0 N`), `move_to_contact`
    raises error `10301`. In Gazebo simulation, this happens either when a
    descent force transient (`> 0.05s`) triggers `APPROACH -> STABILIZE` early
    and `JOINT_STOP` halts the arm partway down from standoff, or when the
    tool/workpiece reaches the physical surface and Gazebo's discrete
    rigid-body contact solver fails to sustain a static reaction force `> 1.0 N`
    at zero velocity.
  - Because `move_to_contact` always starts with `ActionId.TARE (0)`, wrap it in
    a `bt.Retry` (`max_tries=2`) whose `recovery` node moves linearly back to
    the unloaded standoff pose (`-touchdown.standoff_m`) so the F/T sensor is
    tared in free space (not while pressing against a surface), and wrap the
    `bt.Retry` in a `bt.Fallback` whose second `Try` executes a linear
    Cartesian move to the seated contact pose (`Fallback Linear Seat`) if
    retries still time out in simulation (`create_compliant_touchdown_task` /
    `create_seated_approach_tasks` in `src/behaviors/motions.py`):

```python
from intrinsic.manipulation.skills.force import move_to_contact_pb2
from intrinsic.math.proto import vector3_pb2
from intrinsic.solutions import behavior_tree as bt


def build_touchdown_task(
  solution,
  tool_frame_ref,
  reapproach_standoff_task,
  fallback_seat_task,
):
  mtc = solution.skills.ai.intrinsic.move_to_contact
  fixed_vector = move_to_contact_pb2.FixedVector(
    direction=vector3_pb2.Vector3(x=0.0, y=0.0, z=1.0)
  )
  contact_task = bt.Task(
    action=mtc(
      tool=tool_frame_ref,
      fixed_vector=fixed_vector,
      contact_force=8.0,
      timeout_sec=40.0,
    ),
    name="Infeed Pick: Compliant Touchdown (+Z Tool)",
  )
  retry_node = bt.Retry(
    max_tries=2,
    child=contact_task,
    recovery=reapproach_standoff_task,
    name="Infeed Pick: Compliant Touchdown (+Z Tool) (Retry)",
  )
  return bt.Fallback(
    tries=[
      bt.Fallback.Try(condition=None, node=retry_node),
      bt.Fallback.Try(condition=None, node=fallback_seat_task),
    ],
    name="Infeed Pick: Compliant Touchdown (+Z Tool)",
  )
```

---

### 3.3 `update_world` (`ai.intrinsic.update_world`)

- **Protos**: `$EXT/intrinsic-core+/intrinsic/skills/apps/update_world.proto`
  and `$EXT/intrinsic-core+/intrinsic/world/proto/object_world_updates.proto`
- **Bazel Target**: `@intrinsic-core//intrinsic/skills/apps:update_world_skill`
- **Equipment Slots**: `geometry_service` (`GeometryService`), `world_service`
  (`ObjectWorldService`)

> [!CAUTION]
> **Executive Universe Lock Rule (`StatusCode: 18201`)**:
> `update_world_manifest.textproto` sets `lock_the_universe: true`. Placing
> `update_world` (or any `DioCncMachine` door/vise task that calls
> `update_world`) inside a `bt.Parallel` alongside `move_robot` or
> `move_to_contact` triggers a deterministic runtime failure (`StatusCode: 18201`,
> *"Requires exclusive lock on the universe while other resources are in use"*).
> Always sequence `update_world` before or after robot motion in a `bt.Sequence`.

- **`UpdateWorldParams` Schema**: `oneof update_input { ObjectWorldUpdate update = 1; ObjectWorldUpdates updates = 2; }`,
  where each `ObjectWorldUpdate` sets one `update` `oneof` branch:
  1. `update_transform: UpdateTransformRequest`
  2. `update_object_joints: UpdateObjectJointsRequest(object=ObjectReference(...), joint_positions=[...])`
  3. `update_kinematic_object_properties: UpdateKinematicObjectPropertiesRequest`

```python
from intrinsic.solutions import behavior_tree as bt
from intrinsic.world.proto import (
  object_world_refs_pb2,
  object_world_updates_pb2,
)


def build_open_enclosure_door_sim_update(solution):
  update_world = solution.skills.ai.intrinsic.update_world
  update_proto = object_world_updates_pb2.ObjectWorldUpdate(
    update_object_joints=object_world_updates_pb2.UpdateObjectJointsRequest(
      object=object_world_refs_pb2.ObjectReference(
        by_name=object_world_refs_pb2.ObjectReferenceByName(
          object_name="cnc_enclosure"
        )
      ),
      joint_positions=[0.6],
    )
  )
  return bt.Task(
    action=update_world(update=update_proto),
    name="Update CNC Door Joint (Open)",
  )
```

---

### 3.4 `attach_object_to_robot` & `detach_object`

- **Protos**: `$EXT/intrinsic-core+/intrinsic/skills/apps/{attach_object_to_robot,detach_object}.proto`
- **Bazel Targets**: `@intrinsic-core//intrinsic/skills/apps:{attach_object_to_robot_skill,detach_object_skill}`
- **Skill Names on `solution.skills.ai.intrinsic`**: `attach_object_to_robot` and
  `detach_object`
- **Equipment Slot**: `world_service` (`ObjectWorldService`)
- **Parameters (`AttachObjectToRobotParams` / `DetachObjectParams`)**:
  `gripper_entity: ObjectReference | WorldObject` (field 1) and
  `object_entity: ObjectReference | WorldObject` (field 2).
- **Dynamic Objects**: Reference runtime-spawned objects by name using
  `ObjectReference(by_name=ObjectReferenceByName(object_name="workpiece"))`:

```python
from intrinsic.solutions import behavior_tree as bt
from intrinsic.world.proto import object_world_refs_pb2


def build_attach_and_detach_tasks(solution):
  attach_skill = solution.skills.ai.intrinsic.attach_object_to_robot
  detach_skill = solution.skills.ai.intrinsic.detach_object
  gripper_ref = object_world_refs_pb2.ObjectReference(
    by_name=object_world_refs_pb2.ObjectReferenceByName(
      object_name="robotiq_hand_e"
    )
  )
  object_ref = object_world_refs_pb2.ObjectReference(
    by_name=object_world_refs_pb2.ObjectReferenceByName(object_name="workpiece")
  )
  attach_task = bt.Task(
    action=attach_skill(gripper_entity=gripper_ref, object_entity=object_ref),
    name="Attach workpiece to robotiq_hand_e",
  )
  detach_task = bt.Task(
    action=detach_skill(gripper_entity=gripper_ref, object_entity=object_ref),
    name="Detach workpiece from robotiq_hand_e",
  )
  return attach_task, detach_task
```

---

### 3.5 `capture_images`, `estimate_pose_multi_view`, & `create_object`

1. **`capture_images` (`ai.intrinsic.capture_images`)**:
   - **Proto / Target**: `$EXT/intrinsic-core+/intrinsic_perception/intrinsic/perception/skills/capture_images.proto`
     (`@intrinsic-core//intrinsic_perception/intrinsic/perception/skills:capture_images`)
   - **Equipment**: `camera` (`CameraConfig`)
   - **Params / Result**: `timeout: Duration | float`, `sensor_ids`,
     `log_debug_data`, `reference_frame` ->
     `CaptureImagesResult(capture_data=CaptureData)`.
2. **`estimate_pose_multi_view` (`ai.intrinsic.estimate_pose_multi_view`)**:
   - **Protos / Target**:
     `$EXT/intrinsic-core+/intrinsic_perception/intrinsic/perception/skills/multi_view/estimate_pose_multi_view.proto`
     and `$EXT/intrinsic_apis+/intrinsic/perception/proto/v1/pose_estimate_in_root.proto`
     (`@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/multi_view:estimate_pose_multi_view`)
   - **Equipment Slots**: `camera_1`, `camera_2`, `camera_3`, `camera_4`
     (`CameraConfig`), `perception` (`PoseEstimationServiceConfig`)
   - **Params / Result**: `pose_estimator=PoseEstimatorId(id=..., package="ai.intrinsic")`
     (field 1), `capture_data=[capture_action.result.capture_data]`,
     `min_num_instances`, `log_debug_data` ->
     `EstimatePoseMultiViewResult(estimates=Sequence[PoseEstimateInRoot])`, where
     each `PoseEstimateInRoot` has `root_t_target: intrinsic_proto.Pose` (field 1),
     `score: float` (field 2), `id: str` (field 3), `visibility_score: float` (field 4).
3. **`create_object` (`ai.intrinsic.create_object`)**:
   - **Proto / Target**: `$EXT/intrinsic-core+/intrinsic/world/skills/create_object/create_object.proto`
     (`package ai.intrinsic`,
     `@intrinsic-core//intrinsic/world/skills/create_object:create_object`)
   - **Equipment**: `world_service` (`ObjectWorldService`), `geometry_service`
     (`GeometryService`)
   - **Params (`ai.intrinsic.CreateObjectParams`)**: `object_to_create: Id`
     (field 1), `poses: Sequence[Pose | BlackboardValue]` (field 2),
     `randomization` (field 3), `create_in_world: TargetWorld` (field 4),
     `create_at_frame: TransformNodeReference` (field 5),
     `attach_to_create_at_frame: bool` (field 6), `object_naming_schema` (field
     7), `object_names: Sequence[str]` (field 8).

```python
from intrinsic.perception.proto.v1 import pose_estimator_id_pb2
from intrinsic.solutions import behavior_tree as bt


def build_perception_and_spawn_sequence(solution, camera_res, perception_res):
  capture_skill = solution.skills.ai.intrinsic.capture_images
  estimate_skill = solution.skills.ai.intrinsic.estimate_pose_multi_view
  create_object_skill = solution.skills.ai.intrinsic.create_object

  capture_action = capture_skill(
    camera=camera_res,
    sensor_ids=[0, 1],
    log_debug_data=False,
  )
  estimate_action = estimate_skill(
    camera_1=camera_res,
    camera_2=camera_res,
    camera_3=camera_res,
    camera_4=camera_res,
    perception=perception_res,
    pose_estimator=pose_estimator_id_pb2.PoseEstimatorId(
      id="foundationpose_workpiece_pose_estimator",
      package="ai.intrinsic",
    ),
    capture_data=[capture_action.result.capture_data],
    min_num_instances=1,
    log_debug_data=False,
  )
  spawn_action = create_object_skill(
    object_names=["workpiece"],
    poses=[estimate_action.result.estimates[0].root_t_target],
  )
  return bt.Sequence(
    name="Estimate and Spawn Workpiece",
    children=[
      bt.Task(action=capture_action, name="1. Capture RGB-D Images"),
      bt.Task(action=estimate_action, name="2. Estimate 6D Workpiece Poses"),
      bt.Task(action=spawn_action, name="3. Spawn Workpiece in Belief World"),
    ],
  )
```

> **Chaining `.result.estimates[0].root_t_target` into `bt.PythonScript`**:
> In `omts`, `OrbbecVision._create_frame_calc_script_task`
> (`src/hardware/vision.py`) wires `first_est = estimate_action.result.estimates[0].root_t_target`
> (`first_est.position.{x,y,z}`, `first_est.orientation.{x,y,z,w}`) via
> `solution.proto_builder.create_signature_with_args(parameters=pb.MessageSpec(fields=[pb.FieldSpec(...)]))`
> into `bt.PythonScript(signature_with_args=signature, function_body=load_python_script(calculate_and_update_dynamic_frames))`
> (`src/utils/dynamic_frame_calculator.py`).

---

### 3.6 `gripper_cmd_skill`, DIO Skills, and Timed Delays (`bt.PythonScript`)

1. **`gripper_cmd_skill` (`ai.intrinsic.gripper_cmd_skill`)**:
   - **Bundle**: `@hand_e_gripper_cmd_skill_bundle//file` (`//:hand_e_gripper_cmd_skill`)
   - **Protobuf Package**: `ai.intrinsic` (`gripper_cmd_skill.proto`). Because the
     proto `package` is `ai.intrinsic` (not `intrinsic_proto`), its dynamic
     wrapper path is `gripper_skill.ai.intrinsic.JointState`.
   - **Equipment Slot**: `pinch_gripper_service` (auto-defaulted in single-gripper
     cells).
   - **Params (`GripperCmdParams`)**:
     `command: ai.intrinsic.JointState(name=[...], position=[...], velocity=[...], effort=[...])`,
     `action_name: str`.
2. **`dio_set_output`, `dio_wait_for_input`, & `dio_read_input`**:
   - **Protos / Targets**: `$EXT/intrinsic-core+/intrinsic_control/intrinsic/icon/skills/dio_{set_output,wait_for_input,read_input}.proto`
     (`@intrinsic-core//intrinsic_control/intrinsic/icon/skills:dio_{set_output,wait_for_input,read_input}_cc_skill`)
   - **Equipment Slot**: `adio` (`Icon2AdioPart`, `StatusSubscribe`). Always
     resolve via `resolve_adio_resource(solution, adio_device)` in
     `src/utils/math_utils.py` (which verifies `"Icon2AdioPart" in handle.types`
     so `icon` is selected rather than `ur_module`).
   - **Parameters**:
     - `dio_set_output` (`DioSetOutputParams`):
       `dio_output_blocks=[DioOutputBlock(block_name="...", indices=[...], values=[...])]`.
     - `dio_wait_for_input` (`DioWaitForInputParams`): `timeout: float | Duration`,
       `block_name: str`, `indices: Sequence[int]`, `values: Sequence[bool]` (or
       `input_block=DigitalInputBlock(block_name="...", indices=[0], values=[True])`).
     - `dio_read_input` (`DioReadInputParams`): `block_name: str`,
       `timeout: Duration` -> returns `DioReadInputReturnValue(values: Sequence[bool])`.
3. **Timed Delays (`create_dwell_task` / `bt.PythonScript`, Not `sleep_for`)**:
   Open-source `@intrinsic-core` and `//:omts_solution` do **not** include a
   `sleep_for` skill. Always implement timed delays using `create_dwell_task`
   (`src/utils/script_utils.py`) or `bt.Task(action=bt.PythonScript(function_body=...), name=...)`:

```python
from intrinsic.solutions import behavior_tree as bt


def build_robotiq_and_dio_examples(solution, adio_handle):
  gripper_skill = solution.skills.ai.intrinsic.gripper_cmd_skill
  close_gripper = bt.Task(
    action=gripper_skill(
      command=gripper_skill.ai.intrinsic.JointState(
        name=["finger_joint"],
        position=[0.0],
      )
    ),
    name="Close Robotiq Gripper",
  )

  dio_set = solution.skills.ai.intrinsic.dio_set_output
  dio_wait = solution.skills.ai.intrinsic.dio_wait_for_input
  block_cls = dio_set.intrinsic_proto.skills.DioOutputBlock
  delay_s = 0.5

  cnc_handshake = bt.Sequence(
    name="Trigger CNC Machining Cycle (DIO)",
    children=[
      bt.Task(
        action=dio_set(
          adio=adio_handle,
          dio_output_blocks=[
            block_cls(block_name="out", indices=[0], values=[True]),
          ],
        ),
        name="Set Cycle Start Pin High",
      ),
      bt.Task(
        action=bt.PythonScript(
          function_body=f"import time\ntime.sleep({float(delay_s)})\n",
        ),
        name=f"Cycle Start Pulse Dwell ({delay_s}s)",
      ),
      bt.Task(
        action=dio_set(
          adio=adio_handle,
          dio_output_blocks=[
            block_cls(block_name="out", indices=[0], values=[False]),
          ],
        ),
        name="Reset Cycle Start Pin Low",
      ),
      bt.Task(
        action=dio_wait(
          adio=adio_handle,
          block_name="in",
          indices=[0],
          values=[True],
          timeout=120.0,
        ),
        name="Wait for CNC Cycle Complete",
      ),
    ],
  )
  return close_gripper, cnc_handshake
```
