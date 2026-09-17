# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""SBL script to calibrate camera to robot using imported waypoints."""

import datetime
import os
import sys

from absl import app, flags
from google.protobuf import text_format
from intrinsic.math.proto import pose_pb2
from intrinsic.perception.proto.v1 import (
  camera_to_robot_calibration_pb2 as calibration_type_pb2,
)
from intrinsic.perception.skills.calibration import sample_calibration_poses_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments, execution, provided
from intrinsic.world.proto import object_world_updates_pb2

# Where a calibration run writes its camera extrinsic by default. Must name the
# same cell as apply_scene_updates.DEFAULT_UPDATE_FILES, which reads it back.
DEFAULT_UPDATES_FILE = "configs/omts/orbbec_gemini.updates.pbtxt"

# Command line input flags
_ADDRESS = flags.DEFINE_string(
  "address",
  "localhost:17080",
  "gRPC address of the running SBL solution deployment.",
)
_ROBOT = flags.DEFINE_string(
  "robot", "icon", "Robot / controller resource name in the workcell."
)
_CAMERA = flags.DEFINE_string(
  "camera", "orbbec_camera", "Camera name in the workcell."
)
_CALIBRATION_OBJECT = flags.DEFINE_string(
  "calibration_object",
  # "charuco_11x15_35mm_26mm_dict_4x4",
  "charuco_9x14_20mm_15mm_dict_5x5",
  "Calibration board object name in ObjectWorld.",
)
_POSE_ESTIMATOR = flags.DEFINE_string(
  "pose_estimator",
  # "charuco_11x15_35mm_26mm_dict_4x4_estimator",
  "charuco_9x14_20mm_15mm_dict_5x5_estimator",
  "Pose estimator asset name for the calibration board.",
)
_MOVING_CAMERA = flags.DEFINE_bool(
  "moving_camera",
  True,
  "Whether it is a moving camera calibration. If false, it is stationary.",
)
_IMPORT_WAYPOINTS_FILE = flags.DEFINE_string(
  "import_waypoints_file",
  "",
  "Import manual waypoints from this local file path.",
)
_TRANSLATION_RMS_THRESHOLD = flags.DEFINE_float(
  "translation_rms_threshold",
  0.02,
  "Threshold on translation RMS error in meters (default 0.02 = 20mm).",
)
_ROTATION_RMS_THRESHOLD = flags.DEFINE_float(
  "rotation_rms_threshold",
  5.0,
  "Threshold on rotation RMS error in degrees (default 5.0 deg).",
)
_DISABLE_COLLISION_CHECKING = flags.DEFINE_bool(
  "disable_collision_checking",
  False,
  "Whether to disable collision checking during calibration waypoints.",
)
_APPLY_TO_WORLD = flags.DEFINE_bool(
  "apply_to_world",
  None,
  "Whether to apply the calibrated camera pose directly to the live"
  " ObjectWorld without prompting. If None, prompts interactively.",
)
_OUTPUT_UPDATES_FILE = flags.DEFINE_string(
  "output_updates_file",
  "",
  "Path to export/overwrite with the ObjectWorldUpdates .pbtxt file.",
)
_ROBOT_MODULE = flags.DEFINE_string(
  "robot_module",
  "ur_module",
  "Robot module object name in ObjectWorld for moving camera attachment.",
)
_ROBOT_FRAME = flags.DEFINE_string(
  "robot_frame",
  "flange",
  "Flange frame name on robot module for moving camera attachment.",
)
_REPARENT_CAMERA = flags.DEFINE_bool(
  "reparent_camera",
  True,
  "Whether to reparent moving camera to the robot module.",
)


_CalibRes = calibration_type_pb2.CameraToRobotCalibrationResult
_StatPoses = _CalibRes.StationaryCameraResultPoses


def build_camera_world_updates(
  camera_name: str,
  moving_camera: bool,
  moving_camera_poses: (
    calibration_type_pb2.CameraToRobotCalibrationResult.MovingCameraResultPoses
    | None
  ) = None,
  stationary_camera_poses: (_StatPoses | None) = None,
  robot_module_name: str = "ur_module",
  robot_flange_frame: str = "flange",
  reparent_camera: bool = True,
) -> object_world_updates_pb2.ObjectWorldUpdates:
  """Constructs ObjectWorldUpdates proto for the calibrated camera transform."""
  object_world_updates = object_world_updates_pb2.ObjectWorldUpdates()

  if moving_camera:
    if not moving_camera_poses:
      raise ValueError(
        "moving_camera_poses must be provided when moving_camera is True."
      )
    new_pose_proto = moving_camera_poses.flange_t_camera

    a_t_b_pose = pose_pb2.Pose()
    a_t_b_pose.ParseFromString(new_pose_proto.SerializeToString())

    # 1. Update Transform: ur_module:flange -> orbbec_camera
    update_request = object_world_updates_pb2.UpdateTransformRequest()
    update_request.node_a.by_name.frame.object_name = robot_module_name
    update_request.node_a.by_name.frame.frame_name = robot_flange_frame
    update_request.node_b.by_name.object.object_name = camera_name
    update_request.node_to_update.by_name.object.object_name = camera_name
    update_request.a_t_b.CopyFrom(a_t_b_pose)

    object_world_updates.updates.append(
      object_world_updates_pb2.ObjectWorldUpdate(
        update_transform=update_request
      )
    )

    # 2. Reparent camera to robot module
    if reparent_camera:
      reparent_request = object_world_updates_pb2.ReparentObjectRequest()
      reparent_request.object.by_name.object_name = camera_name
      reparent_request.new_parent.reference.by_name.object_name = (
        robot_module_name
      )
      reparent_request.new_parent.entity_filter.include_final_entity = True

      object_world_updates.updates.append(
        object_world_updates_pb2.ObjectWorldUpdate(
          reparent_object=reparent_request
        )
      )

  else:
    if not stationary_camera_poses:
      raise ValueError(
        "stationary_camera_poses must be provided when moving_camera is False."
      )
    new_pose_proto = stationary_camera_poses.base_t_camera

    a_t_b_pose = pose_pb2.Pose()
    a_t_b_pose.ParseFromString(new_pose_proto.SerializeToString())

    update_request = object_world_updates_pb2.UpdateTransformRequest()
    update_request.node_a.by_name.object.object_name = "root"
    update_request.node_b.by_name.object.object_name = camera_name
    update_request.node_to_update.by_name.object.object_name = camera_name
    update_request.a_t_b.CopyFrom(a_t_b_pose)

    object_world_updates.updates.append(
      object_world_updates_pb2.ObjectWorldUpdate(
        update_transform=update_request
      )
    )

  return object_world_updates


def apply_camera_updates_to_world(
  world, updates: object_world_updates_pb2.ObjectWorldUpdates
) -> None:
  """Applies ObjectWorldUpdates live to the active ObjectWorld deployment."""
  print(
    f"\nApplying {len(updates.updates)} update rule(s) to live ObjectWorld..."
  )
  world.batch_update(updates)
  print(
    "[✓] Successfully applied updated camera transform to live ObjectWorld!"
  )


def save_updates_to_file(
  updates: object_world_updates_pb2.ObjectWorldUpdates, filepath: str
) -> None:
  """Saves ObjectWorldUpdates proto to a .pbtxt file on disk."""
  workspace_dir = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
  if workspace_dir and not os.path.isabs(filepath):
    resolved_path = os.path.normpath(os.path.join(workspace_dir, filepath))
  else:
    resolved_path = filepath

  parent_dir = os.path.dirname(resolved_path)
  if parent_dir and not os.path.exists(parent_dir):
    os.makedirs(parent_dir, exist_ok=True)

  with open(resolved_path, "w", encoding="utf-8") as f:
    f.write(text_format.MessageToString(updates))
  print(f"[✓] Successfully saved ObjectWorldUpdates to {resolved_path}")


def read_input(prompt: str, choices: list[str]) -> str:
  sys.stdout.write(prompt)
  sys.stdout.flush()
  while True:
    val = sys.stdin.readline().strip().lower()
    if val in choices:
      return val


def log_task(message: str) -> bt.Task:
  safe_name = "".join([c if c.isalnum() else "_" for c in message]).lower()
  return bt.Task(
    action=bt.PythonScript(function_body=f'print("{message}")'),
    name=f"log_{safe_name}",
  )


def main(argv) -> None:
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  import_path = _IMPORT_WAYPOINTS_FILE.value
  if not import_path:
    import_path = input("Enter file path to import waypoints from: ").strip()
    if not import_path:
      raise ValueError("An import waypoint file must be specified.")

  solution = deployments.connect(address=_ADDRESS.value)

  executive = solution.executive
  skills = solution.skills
  world = solution.world

  arm_part_ref = world.get_object(_ROBOT_MODULE.value)
  if not arm_part_ref:
    try:
      arm_part_ref = getattr(world, _ROBOT_MODULE.value)
    except AttributeError:
      raise ValueError(
        f"Robot module '{_ROBOT_MODULE.value}' not found in the world model."
      )

  try:
    calibration_service = solution.resources["calibration_service"]
  except KeyError:
    raise ValueError("calibration_service not found in solution resources.")

  try:
    motion_planner_service = solution.resources["motion_planner_service"]
  except KeyError:
    raise ValueError("motion_planner_service not found in solution resources.")

  calibration_object_ref = world.get_object(_CALIBRATION_OBJECT.value)
  if not calibration_object_ref:
    raise ValueError(
      f"Calibration object '{_CALIBRATION_OBJECT.value}' not found in the"
      " world model."
    )

  pose_estimator = solution.pose_estimators[_POSE_ESTIMATOR.value]
  if not pose_estimator:
    raise ValueError(
      f"Pose estimator '{_POSE_ESTIMATOR.value}' not found in the solution."
    )

  if _MOVING_CAMERA.value:
    calibration_type = (
      calibration_type_pb2.CAMERA_TO_ROBOT_CALIBRATION_TYPE_MOVING_CAMERA
    )
  else:
    calibration_type = (
      calibration_type_pb2.CAMERA_TO_ROBOT_CALIBRATION_TYPE_STATIONARY_CAMERA
    )

  # Import waypoints
  try:
    workspace_dir = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
    if workspace_dir and not os.path.isabs(import_path):
      import_path = os.path.normpath(os.path.join(workspace_dir, import_path))

    result_proto = sample_calibration_poses_pb2.SampleCalibrationPosesResult()
    with open(import_path) as f:
      text_format.Parse(f.read(), result_proto)
    waypoints = list(result_proto.sample_calibration_poses_result)
    print(
      f"Successfully imported {len(waypoints)} waypoints from {import_path}"
    )
  except Exception as e:
    raise ValueError(f"Failed to import waypoints from {import_path}: {e}")

  # Initialize calibration service
  initialize_calibration = skills.ai.intrinsic.initialize_calibration(
    pose_estimator=pose_estimator,
    calibration_object=calibration_object_ref,
    arm_part=arm_part_ref,
    calibration_service=calibration_service,
    data_assets_service=provided.ResourceHandle.create(
      name="intrinsic_runtime",
      capabilities=["intrinsic_proto.data.v1.DataAssets"],
    ),
  )
  collect_calibration_data_skill = skills.ai.intrinsic.collect_calibration_data
  calibrate_camera_to_robot_skill = (
    skills.ai.intrinsic.calibrate_camera_to_robot
  )

  # Setup calibration subtree
  skill_proto = collect_calibration_data_skill.intrinsic_proto.skills
  motion_type_joint = skill_proto.MotionType.MOTION_TYPE_JOINT

  collect_calibration_data = collect_calibration_data_skill(
    calibration_type=calibration_type,
    calibration_object=calibration_object_ref,
    waypoints=waypoints,
    arm_part=arm_part_ref,
    motion_planner_service=motion_planner_service,
    calibration_service=calibration_service,
    disable_collision_checking=_DISABLE_COLLISION_CHECKING.value,
    motion_type=(motion_type_joint),
    skip_return_to_base_between_waypoints=True,
  )
  collect_calibration_data.execute_timeout = datetime.timedelta(seconds=600)

  calibrate = calibrate_camera_to_robot_skill(
    calibration_type=calibration_type,
    translation_root_mean_square_error_threshold=(
      _TRANSLATION_RMS_THRESHOLD.value
    ),
    rotation_root_mean_square_error_threshold=_ROTATION_RMS_THRESHOLD.value,
    calibration_service=calibration_service,
  )

  calibration_children = [
    log_task("Initializing main calibration session..."),
    initialize_calibration,
    log_task("Collecting calibration data..."),
    collect_calibration_data,
    log_task("Running main calibration solver..."),
    calibrate,
    log_task("Main calibration completed successfully."),
  ]

  calibration = bt.SubTree(
    name="Calibration",
    behavior_tree=bt.Sequence(children=calibration_children),
  )

  try:
    executive.run(calibration, silence_outputs=True)
    print("Execution completed successfully.")
  except execution.ExecutionFailedError as e:
    print("=== Execution Failed ===")
    print("Error message:", e)
    if executive.operation and executive.operation.proto.HasField("error"):
      print("Raw status message:", executive.operation.proto.error.message)
      from intrinsic.util.status import extended_status_pb2, status_exception

      def print_extended_status(err, indent=""):
        status_code = err._extended_status.status_code
        print(f"{indent}StatusCode: {status_code.component}:{status_code.code}")
        if err._extended_status.title:
          print(f"{indent}Title: {err._extended_status.title}")
        if err._extended_status.HasField("timestamp"):
          print(
            f"{indent}Timestamp:"
            f" {err._extended_status.timestamp.ToDatetime().strftime('%c')}"
          )
        if (
          err._extended_status.HasField("user_report")
          and err._extended_status.user_report.message
        ):
          print(
            f"{indent}User Report: {err._extended_status.user_report.message}"
          )
        if (
          err._extended_status.HasField("debug_report")
          and err._extended_status.debug_report.message
        ):
          print(
            f"{indent}Debug Report: {err._extended_status.debug_report.message}"
          )
        if err._extended_status.context:
          print(f"{indent}Context:")
          for j, ctx in enumerate(err._extended_status.context):
            print(f"{indent}  Context [{j}]:")
            ctx_err = status_exception.ExtendedStatusError.create_from_proto(
              ctx
            )
            print_extended_status(ctx_err, indent + "    ")

      for i, detail in enumerate(executive.operation.proto.error.details):
        print(f"Error Detail [{i}] Type URL: {detail.type_url}")
        if detail.Is(extended_status_pb2.ExtendedStatus.DESCRIPTOR):
          try:
            ext_status = extended_status_pb2.ExtendedStatus()
            detail.Unpack(ext_status)
            ext_err = status_exception.ExtendedStatusError.create_from_proto(
              ext_status
            )
            print(f"--- Unpacked ExtendedStatus [{i}] ---")
            print_extended_status(ext_err)
            print("----------------------------------")
          except Exception as ex:
            print(f"  Failed to unpack ExtendedStatus: {ex}")
    print(executive.get_errors())
    return

  # Retrieve results from blackboard
  try:
    res = executive.operation.blackboard.get_value(
      calibrate.result
    ).calibration_results[0]
  except Exception:
    res = executive.get_value(calibrate.result).calibration_results[0]

  # Print results
  print("=== Calibration Results ===")
  print(
    "Translation RMS Error:"
    f" {res.translation_root_mean_square_error * 1000.0:.4f} mm"
  )
  print(
    f"Translation Max Error: {res.translation_maximum_error * 1000.0:.4f} mm"
  )
  print(
    "Rotation RMS Error:"
    f" {res.rotation_root_mean_square_error_in_degrees:.4f} deg"
  )
  print(f"Rotation Max Error: {res.rotation_maximum_error_in_degrees:.4f} deg")

  if res.HasField("stationary_camera_result_poses"):
    print("Stationary Camera Result Poses:")
    print(res.stationary_camera_result_poses)
  elif res.HasField("moving_camera_result_poses"):
    print("Moving Camera Result Poses:")
    print(res.moving_camera_result_poses)

  # Build ObjectWorldUpdates proto
  try:
    object_world_updates = build_camera_world_updates(
      camera_name=_CAMERA.value,
      moving_camera=_MOVING_CAMERA.value,
      moving_camera_poses=(
        res.moving_camera_result_poses
        if res.HasField("moving_camera_result_poses")
        else None
      ),
      stationary_camera_poses=(
        res.stationary_camera_result_poses
        if res.HasField("stationary_camera_result_poses")
        else None
      ),
      robot_module_name=_ROBOT_MODULE.value,
      robot_flange_frame=_ROBOT_FRAME.value,
      reparent_camera=_REPARENT_CAMERA.value,
    )
  except Exception as e:
    print(f"Error building ObjectWorldUpdates proto: {e}")
    return

  print("\n=== Generated ObjectWorldUpdates ===")
  print(object_world_updates)

  # Determine whether to apply to live ObjectWorld
  apply_to_world = _APPLY_TO_WORLD.value
  if apply_to_world is None:
    apply_choice = read_input(
      "\nApply the new camera transform directly to the live ObjectWorld?"
      " [y/n]: ",
      ["y", "n"],
    )
    apply_to_world = apply_choice == "y"

  if apply_to_world:
    try:
      apply_camera_updates_to_world(world, object_world_updates)
    except Exception as e:
      print(f"Error applying camera updates to live ObjectWorld: {e}")

  # Determine whether to save to file
  export_path = _OUTPUT_UPDATES_FILE.value
  if not export_path and _APPLY_TO_WORLD.value is None:
    save_choice = read_input(
      "\nSave ObjectWorldUpdates to a .pbtxt config file on disk? [y/n]: ",
      ["y", "n"],
    )
    if save_choice == "y":
      default_path = DEFAULT_UPDATES_FILE
      user_path = input(f"Enter file path [default: {default_path}]: ").strip()
      export_path = user_path if user_path else default_path

  if export_path:
    try:
      save_updates_to_file(object_world_updates, export_path)
    except Exception as e:
      print(f"Error saving ObjectWorldUpdates to file: {e}")


if __name__ == "__main__":
  app.run(main)
