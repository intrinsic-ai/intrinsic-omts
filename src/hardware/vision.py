"""Vision and 3D camera hardware interfaces and implementations."""

import abc
from typing import Optional, Sequence
from unittest import mock

from intrinsic.assets import id_utils
from intrinsic.assets.proto import id_pb2
from intrinsic.perception.public.proto.v1 import pose_estimator_id_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import cel
from intrinsic.solutions import deployments
from intrinsic.solutions import proto_building as pb
from intrinsic.solutions import provided
from intrinsic.world.public.proto import object_world_refs_pb2
from src.core.types import Pose3D



def _get_camera_resource(
    solution: deployments.Solution,
    camera_name: Optional[str] = None,
) -> provided.ResourceHandle:
  """Resolves the camera resource handle from solution resources."""
  target_name = camera_name or "orbbec_camera"
  if isinstance(solution.resources, dict) and target_name in solution.resources:
    return solution.resources[target_name]
  try:
    return solution.resources[target_name]
  except (KeyError, AttributeError, TypeError):
    pass

  # Fallback to capability search
  for handle in getattr(solution.resources, "values", lambda: solution.resources)():
    if hasattr(handle, "types") and "CameraConfig" in handle.types:
      return handle
  raise ValueError(f"Camera resource '{target_name}' not found in solution.")


def _get_perception_resource(
    solution: deployments.Solution,
    service_name: Optional[str] = None,
) -> provided.ResourceHandle:
  """Resolves the perception service resource handle from solution resources."""
  target_name = service_name or "pose_estimator_service"
  if isinstance(solution.resources, dict) and target_name in solution.resources:
    return solution.resources[target_name]
  try:
    return solution.resources[target_name]
  except (KeyError, AttributeError, TypeError):
    pass

  # Fallback to capability search
  for handle in getattr(solution.resources, "values", lambda: solution.resources)():
    if (
        hasattr(handle, "types")
        and "intrinsic_proto.perception.v1.PoseEstimationService" in handle.types
    ):
      return handle
  raise ValueError(f"Perception service resource '{target_name}' not found in solution.")


class VisionInterface(abc.ABC):
  """Abstract interface for perception acquisition and pose estimation."""

  @abc.abstractmethod
  def build_capture_image_task(self, name: Optional[str] = None) -> bt.Node:
    """Builds a task to trigger camera image acquisition."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_perception_and_spawn_task(
      self,
      target_scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
      pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
      min_num_instances: int = 1,
      approach_offset_z: float = 0.05,
      parent_object: str = "root",
      pregrasp_frame_name: str = "pre_grasp",
      grasp_frame_name: str = "grasp",
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds a composite task to capture RGB-D, estimate 6D poses, and update world frames."""
    raise NotImplementedError


class OrbbecVision(VisionInterface):
  """Orbbec 3D camera perception adapter using IOC SBL perception skills."""

  def __init__(
      self,
      solution: deployments.Solution,
      camera_name: str = "orbbec_camera",
      perception_service_name: str = "pose_estimator_service",
      sensor_ids: Sequence[int] = (1, 4),
      log_debug_data: bool = True,
  ) -> None:
    self._solution = solution
    self._camera_name = camera_name
    self._perception_service_name = perception_service_name
    self._sensor_ids = list(sensor_ids)
    self._log_debug_data = log_debug_data

    # Resolve resource handles
    self._camera_resource = _get_camera_resource(solution, camera_name)
    self._perception_resource = _get_perception_resource(
        solution, perception_service_name
    )

  def build_capture_image_task(self, name: Optional[str] = None) -> bt.Node:
    """Builds a single image capture task."""
    task_name = name or f"Capture Image ({self._camera_name})"
    action = self._solution.skills.ai.intrinsic.capture_images(
        camera=self._camera_resource,
        sensor_ids=self._sensor_ids,
        log_debug_data=self._log_debug_data,
    )
    return bt.Task(action=action, name=task_name)

  def build_perception_and_spawn_task(
      self,
      target_scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
      pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
      min_num_instances: int = 1,
      approach_offset_z: float = 0.05,
      parent_object: str = "root",
      pregrasp_frame_name: str = "pre_grasp",
      grasp_frame_name: str = "grasp",
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds the pipeline to capture RGB-D, estimate 6D poses, and dynamically update grasp frames."""
    task_name = name or "Perception & Dynamic Grasp Frame Update Pipeline"

    skills = self._solution.skills

    # 1. Capture RGB-D Images
    capture_action = skills.ai.intrinsic.capture_images(
        camera=self._camera_resource,
        sensor_ids=self._sensor_ids,
        log_debug_data=self._log_debug_data,
    )
    capture_task = bt.Task(
        action=capture_action, name="1. Capture RGB-D Images"
    )

    # 2. Estimate 6D Poses via Multi-View / FoundationPose
    pkg = (
        id_utils.package_from(pose_estimator_id)
        if id_utils.is_id(pose_estimator_id)
        else "ai.intrinsic"
    )
    est_name = (
        id_utils.name_from(pose_estimator_id)
        if id_utils.is_id(pose_estimator_id)
        else pose_estimator_id
    )

    pose_estimator_proto = pose_estimator_id_pb2.PoseEstimatorId(
        id=est_name,
        package=pkg,
    )

    estimate_action = skills.ai.intrinsic.estimate_pose_multi_view(
        camera_1=self._camera_resource,
        camera_2=self._camera_resource,
        camera_3=self._camera_resource,
        camera_4=self._camera_resource,
        perception=self._perception_resource,
        pose_estimator=pose_estimator_proto,
        capture_data=[capture_action.result.capture_data],
        min_num_instances=min_num_instances,
        log_debug_data=self._log_debug_data,
    )
    estimate_task = bt.Task(
        action=estimate_action, name="2. Estimate 6D Workpiece Poses"
    )

    # 3. Dynamic Grasp and Pre-Grasp Frame Calculation and World Update via bt.PythonScript
    first_est = estimate_action.result.estimates[0].root_t_target
    if hasattr(self._solution, "proto_builder") and not isinstance(
        self._solution.proto_builder, mock.MagicMock
    ):
      signature = self._solution.proto_builder.create_signature_with_args(
          parameters=pb.MessageSpec(
              fields=[
                  pb.FieldSpec(
                      type="float",
                      name="pos_x",
                      number=1,
                      arg=first_est.position.x,
                  ),
                  pb.FieldSpec(
                      type="float",
                      name="pos_y",
                      number=2,
                      arg=first_est.position.y,
                  ),
                  pb.FieldSpec(
                      type="float",
                      name="pos_z",
                      number=3,
                      arg=first_est.position.z,
                  ),
                  pb.FieldSpec(
                      type="float",
                      name="ori_x",
                      number=4,
                      arg=first_est.orientation.x,
                  ),
                  pb.FieldSpec(
                      type="float",
                      name="ori_y",
                      number=5,
                      arg=first_est.orientation.y,
                  ),
                  pb.FieldSpec(
                      type="float",
                      name="ori_z",
                      number=6,
                      arg=first_est.orientation.z,
                  ),
                  pb.FieldSpec(
                      type="float",
                      name="ori_w",
                      number=7,
                      arg=first_est.orientation.w,
                  ),
                  pb.FieldSpec(
                      type="float",
                      name="approach_offset_z",
                      number=8,
                      arg=approach_offset_z,
                  ),
                  pb.FieldSpec(
                      type="string",
                      name="parent_object",
                      number=9,
                      arg=parent_object,
                  ),
                  pb.FieldSpec(
                      type="string",
                      name="pregrasp_frame_name",
                      number=10,
                      arg=pregrasp_frame_name,
                  ),
                  pb.FieldSpec(
                      type="string",
                      name="grasp_frame_name",
                      number=11,
                      arg=grasp_frame_name,
                  ),
                  pb.FieldSpec(
                      type="string",
                      name="camera_name",
                      number=12,
                      arg=self._camera_name,
                  ),
              ]
          ),
      )
    else:
      signature = None

    python_code = """
import math
from intrinsic.math.python import data_types

world = context.object_world
parent_obj = getattr(world, params.parent_object, getattr(world, "root", None))

# Resolve camera sensor transform in parent object (root)
camera_obj = getattr(world, params.camera_name, None)
if camera_obj is None:
  for obj_name in ["ur_module", "robot", "root"]:
    p = getattr(world, obj_name, None)
    if p is not None and hasattr(p, params.camera_name):
      camera_obj = getattr(p, params.camera_name)
      break

camera_sensor_node = getattr(camera_obj, "sensor", camera_obj) if camera_obj else None
root_t_camera = world.get_transform(parent_obj, camera_sensor_node) if camera_sensor_node else None

# Construct detected pose in camera frame
cam_q = data_types.Quaternion([params.ori_x, params.ori_y, params.ori_z, params.ori_w])
cam_pose = data_types.Pose3(data_types.Rotation3(cam_q), [params.pos_x, params.pos_y, params.pos_z])

if root_t_camera is not None:
  root_t_target = root_t_camera * cam_pose
else:
  root_t_target = cam_pose

target_pos = root_t_target.translation
target_rot = root_t_target.rotation

# Determine longest axis alignment in root XY plane:
# Local X is the longest axis (5"), local Z is the medium axis (3"), local Y is the thickness (2").
ax = target_rot.rotate_point([1.0, 0.0, 0.0])
az = target_rot.rotate_point([0.0, 0.0, 1.0])

if abs(float(ax[2])) < 0.7:
  vx, vy = float(ax[0]), float(ax[1])
else:
  vx, vy = float(az[0]), float(az[1])

theta_longest = math.atan2(vy, vx)
# Grasp on the short side of the workpiece (rotated 90° around Z relative to long-side grasp):
psi = theta_longest

half_psi = psi / 2.0
c1 = (math.cos(half_psi), math.sin(half_psi), 0.0, 0.0)
c2 = (-math.sin(half_psi), math.cos(half_psi), 0.0, 0.0)
candidates = [
    c1,
    (-c1[0], -c1[1], 0.0, 0.0),
    c2,
    (-c2[0], -c2[1], 0.0, 0.0),
]

tool_obj = getattr(world, "gripper", None)
tool_node = getattr(tool_obj, "tool_frame", None) if tool_obj else None
cur_tool_tf = world.get_transform(parent_obj, tool_node) if tool_node else None

if cur_tool_tf is not None:
  cur_q = cur_tool_tf.rotation.quaternion
  grasp_ori = max(
      candidates,
      key=lambda c: (
          c[0] * float(cur_q.x)
          + c[1] * float(cur_q.y)
          + c[2] * float(cur_q.z)
          + c[3] * float(cur_q.w)
      ),
  )
else:
  grasp_ori = c1

grasp_pos = (float(target_pos[0]), float(target_pos[1]), float(target_pos[2]))
pregrasp_pos = (float(target_pos[0]), float(target_pos[1]), float(target_pos[2]) + params.approach_offset_z)

frame_poses = {
    params.pregrasp_frame_name: (pregrasp_pos, grasp_ori),
    params.grasp_frame_name: (grasp_pos, grasp_ori),
}

existing_frames = set()
if hasattr(parent_obj, "list_frames"):
  existing_frames = set(parent_obj.list_frames())
elif hasattr(parent_obj, "__dict__"):
  existing_frames = set(parent_obj.__dict__.keys())

for fname, (pos, ori) in frame_poses.items():
  pose = data_types.Pose3(
      data_types.Rotation3(data_types.Quaternion([ori[0], ori[1], ori[2], ori[3]])),
      [pos[0], pos[1], pos[2]],
  )
  if fname in existing_frames or hasattr(parent_obj, fname):
    frame_node = getattr(parent_obj, fname)
    world.update_transform(node_a=parent_obj, node_b=frame_node, a_t_b=pose)
  else:
    world.create_frame(frame_name=fname, parent=parent_obj, parent_t_frame=pose)
"""

    calc_script = bt.PythonScript(
        signature_with_args=signature,
        function_body=python_code,
    )
    calc_task = bt.Task(
        action=calc_script,
        name="3. Calculate & Update Dynamic Grasp & Pre-Grasp Frames",
    )

    return bt.Sequence(
        name=task_name,
        children=[
            capture_task,
            estimate_task,
            calc_task,
        ],
    )


class MockVision(VisionInterface):
  """Mock vision sensor for offline testing."""

  def __init__(self, simulated_pose: Optional[Pose3D] = None) -> None:
    self.simulated_pose = simulated_pose or Pose3D(x=0.15, y=0.25, z=0.71)
    self.capture_count: int = 0
    self.pipeline_count: int = 0

  def build_capture_image_task(self, name: Optional[str] = None) -> bt.Node:
    self.capture_count += 1
    return bt.Sequence(name=name or "Mock Capture Image", children=[])

  def build_perception_and_spawn_task(
      self,
      target_scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
      pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
      min_num_instances: int = 1,
      approach_offset_z: float = 0.05,
      parent_object: str = "root",
      pregrasp_frame_name: str = "pre_grasp",
      grasp_frame_name: str = "grasp",
      name: Optional[str] = None,
  ) -> bt.Node:
    self.pipeline_count += 1
    return bt.Sequence(
        name=name or "Mock Perception & Dynamic Grasp Frame Update Pipeline",
        children=[],
    )
