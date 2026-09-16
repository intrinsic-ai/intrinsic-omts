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

"""World interface, ObjectWorld facade, and offline test doubles."""

import abc
import logging
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import proto_building as pb

from src.core.types import JointPosition
from src.utils import dynamic_frame_calculator, math_utils
from src.utils.execution_utils import create_transform_node_ref
from src.utils.script_utils import load_python_script


def _build_dynamic_frame_signature(
  proto_builder: Any,
  approach_offset_z: float,
  parent_object: str,
  pregrasp_frame_name: str,
  grasp_frame_name: str,
  camera_name: str,
  target_scene_object_id: str,
  estimates: Any,
  min_safe_z: float,
) -> Any:
  """Builds the proto_builder signature for dynamic frame calculation."""
  if proto_builder is None:
    return None
  fields = [
    pb.FieldSpec(
      type="float",
      name="approach_offset_z",
      number=1,
      arg=float(approach_offset_z),
    ),
    pb.FieldSpec(
      type="string", name="parent_object", number=2, arg=parent_object
    ),
    pb.FieldSpec(
      type="string",
      name="pregrasp_frame_name",
      number=3,
      arg=pregrasp_frame_name,
    ),
    pb.FieldSpec(
      type="string", name="grasp_frame_name", number=4, arg=grasp_frame_name
    ),
    pb.FieldSpec(type="string", name="camera_name", number=5, arg=camera_name),
    pb.FieldSpec(
      type="string",
      name="target_scene_object_id",
      number=6,
      arg=target_scene_object_id,
    ),
    pb.FieldSpec(
      type="intrinsic_proto.perception.v1.PoseEstimateInRoot",
      name="estimates",
      number=7,
      arg=estimates,
      repeated=True,
    ),
    pb.FieldSpec(
      type="float", name="min_safe_z", number=8, arg=float(min_safe_z)
    ),
  ]
  return proto_builder.create_signature_with_args(
    parameters=pb.MessageSpec(fields=fields)
  )


class WorldInterface(abc.ABC):
  """Abstract interface for scene graph queries and world mutations."""

  @abc.abstractmethod
  def find_object(self, name: str) -> Any:
    """Resolves a scene object by name or short identifier."""
    raise NotImplementedError

  @abc.abstractmethod
  def resolve_frame(self, object_name: str, frame_name: str) -> Any:
    """Resolves a transform frame node on the given object."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_attach_to_gripper_task(
    self,
    object_name: str,
    gripper_name: str = "gripper",
    name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task attaching object_name to gripper_name."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_detach_from_gripper_task(
    self,
    object_name: str,
    gripper_name: str = "gripper",
    name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task detaching object_name from gripper_name."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_joint_update_task(
    self,
    object_name: str,
    joints: JointPosition | Sequence[float],
    name: str | None = None,
    task_name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task updating joint positions on object_name."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_update_grasp_frames_task(
    self,
    estimates: Any,
    camera_name: str = "orbbec_camera",
    target_scene_object_id: str = "raw_stock_2x3x5",
    parent_object: str = "root",
    pregrasp_frame_name: str = "infeed_pre_grasp",
    grasp_frame_name: str = "infeed_grasp",
    approach_offset_z: float = 0.05,
    min_safe_z: float = 0.95,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a task updating dynamic grasp frames and workpiece pose in ObjectWorld."""
    raise NotImplementedError

  @abc.abstractmethod
  def clear_stale_frames(self, names: Sequence[str]) -> list[str]:
    """Removes dynamic frames from the root object in the world."""
    raise NotImplementedError

  @abc.abstractmethod
  def reset(
    self,
    workpiece_name: str = "raw_stock_2x3x5",
    dynamic_frames: Sequence[str] = ("infeed_grasp", "infeed_pre_grasp"),
    scene_update_files: Sequence[str] = ("configs/omts/scene.updates.pbtxt",),
    robot: Any | None = None,
    solution: Any | None = None,
    clear_faults: bool = False,
    **kwargs: Any,
  ) -> list[str]:
    """Clears dynamic frames and reapplies scene updates."""
    raise NotImplementedError


class World(WorldInterface):
  """Facade over live Flowstate ObjectWorld lookups and native world skills."""

  def __init__(self, client: Any, solution: Any | None = None) -> None:
    self._client = client
    self._solution = solution

  @property
  def client(self) -> Any:
    """Returns the underlying Flowstate ObjectWorldClient handle."""
    return self._client

  @property
  def raw(self) -> Any:
    """Alias for client."""
    return self._client

  def find_object(self, name: str) -> Any:
    """Resolves an object in ObjectWorld by name or short asset identifier."""
    if self._client is None:
      raise ValueError(f"Cannot find object '{name}': world client is None.")

    short = name.split(".")[-1]
    for candidate in (name, short):
      if hasattr(self._client, candidate):
        obj = getattr(self._client, candidate)
        if obj is not None:
          return obj
      if hasattr(self._client, "get_object"):
        try:
          obj = self._client.get_object(candidate)
          if obj is not None:
            return obj
        except Exception:  # pylint: disable=broad-exception-caught
          pass

    raise ValueError(f"Object '{name}' not found in ObjectWorld.")

  def resolve_frame(self, object_name: str, frame_name: str) -> Any:
    """Resolves a transform node for the given object and frame name."""
    ref = create_transform_node_ref(object_name, frame_name)
    return self._client.get_transform_node(ref)

  def build_attach_to_gripper_task(
    self,
    object_name: str,
    gripper_name: str = "gripper",
    name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task attaching object_name to gripper_name."""
    label = name or f"Attach {object_name} to {gripper_name}"
    skills = getattr(self._solution, "skills", None)
    if skills is not None:
      action = skills.ai.intrinsic.attach_object_to_robot(
        gripper_entity=self.find_object(gripper_name),
        object_entity=self.find_object(object_name),
      )
      return bt.Task(action=action, name=label)
    return bt.Task(action=bt.PythonScript(function_body="pass"), name=label)

  def build_detach_from_gripper_task(
    self,
    object_name: str,
    gripper_name: str = "gripper",
    name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task detaching object_name from gripper_name."""
    label = name or f"Detach {object_name} from {gripper_name}"
    skills = getattr(self._solution, "skills", None)
    if skills is not None:
      action = skills.ai.intrinsic.detach_object(
        gripper_entity=self.find_object(gripper_name),
        object_entity=self.find_object(object_name),
      )
      return bt.Task(action=action, name=label)
    return bt.Task(action=bt.PythonScript(function_body="pass"), name=label)

  def build_joint_update_task(
    self,
    object_name: str,
    joints: JointPosition | Sequence[float],
    name: str | None = None,
    task_name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task updating joint positions via update_world skill."""
    label = name or task_name or f"Update joints for {object_name}"
    vals = [
      float(p)
      for p in (
        joints.positions if isinstance(joints, JointPosition) else joints
      )
    ]
    skills = getattr(self._solution, "skills", None)
    if skills is not None:
      update_skill = skills.ai.intrinsic.update_world
      req = update_skill.ObjectWorldUpdate(
        update_object_joints=update_skill.UpdateObjectJointsRequest(
          object=self.find_object(object_name),
          joint_positions=vals,
        )
      )
      return bt.Task(action=update_skill(update=req), name=label)
    return bt.Task(action=bt.PythonScript(function_body="pass"), name=label)

  def build_update_grasp_frames_task(
    self,
    estimates: Any,
    camera_name: str = "orbbec_camera",
    target_scene_object_id: str = "raw_stock_2x3x5",
    parent_object: str = "root",
    pregrasp_frame_name: str = "infeed_pre_grasp",
    grasp_frame_name: str = "infeed_grasp",
    approach_offset_z: float = 0.05,
    min_safe_z: float = 0.95,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a task updating dynamic grasp frames and workpiece pose in ObjectWorld."""
    signature = _build_dynamic_frame_signature(
      proto_builder=getattr(self._solution, "proto_builder", None),
      approach_offset_z=approach_offset_z,
      parent_object=parent_object,
      pregrasp_frame_name=pregrasp_frame_name,
      grasp_frame_name=grasp_frame_name,
      camera_name=camera_name,
      target_scene_object_id=target_scene_object_id,
      estimates=estimates,
      min_safe_z=min_safe_z,
    )
    calc_script = bt.PythonScript(
      signature_with_args=signature,
      function_body=load_python_script(
        dynamic_frame_calculator.calculate_and_update_dynamic_frames,
        preludes=(math_utils,),
      ),
    )
    return bt.Task(
      action=calc_script,
      name=name or "Calculate & Update Dynamic Grasp & Pre-Grasp Frames",
    )

  def clear_stale_frames(self, names: Sequence[str]) -> list[str]:
    """Removes dynamic frames from root object in the world."""
    if self._client is None:
      return []
    cleared = []
    for df_name in names:
      try:
        ref = create_transform_node_ref("root", df_name)
        frame = self._client.get_transform_node(ref)
        self._client.delete_frame(frame, force=True)
        cleared.append(df_name)
        logging.info("Deleted stale dynamic frame: %s", df_name)
      except Exception as err:  # pylint: disable=broad-exception-caught
        logging.warning("Failed to delete frame %s: %s", df_name, err)
    return cleared

  def reset(
    self,
    workpiece_name: str = "raw_stock_2x3x5",
    dynamic_frames: Sequence[str] = ("infeed_grasp", "infeed_pre_grasp"),
    scene_update_files: Sequence[str] = ("configs/omts/scene.updates.pbtxt",),
    robot: Any | None = None,
    solution: Any | None = None,
    clear_faults: bool = False,
    **kwargs: Any,
  ) -> list[str]:
    """Clears dynamic frames, reapplies scene updates, and optionally clears robot faults."""
    from tools.world.apply_scene_updates import apply_pbtxt_file

    del workpiece_name, solution, kwargs
    reset_actions: list[str] = []
    cleared = self.clear_stale_frames(dynamic_frames)
    reset_actions.extend(f"frame:{f}" for f in cleared)

    if self._client is not None:
      for fpath in scene_update_files:
        try:
          apply_pbtxt_file(world=self._client, filepath=fpath)
        except Exception as err:  # pylint: disable=broad-exception-caught
          logging.warning("Failed to reapply scene update %s: %s", fpath, err)

    if clear_faults and robot is not None and hasattr(robot, "clear_faults"):
      robot.clear_faults()

    return reset_actions


class OfflineExecutive:
  """Executive that records behavior trees without executing hardware actions."""

  def __init__(self) -> None:
    self.executed: list[Any] = []
    self.last_simulation_mode: Any = None

  def run(self, tree: Any, simulation_mode: Any = None) -> None:
    """Records the tree and simulation mode without executing."""
    self.last_simulation_mode = simulation_mode
    self.executed.append(tree)
    logging.info("Offline executive accepted a behavior tree; not executing.")


class MockWorld(WorldInterface):
  """In-memory stand-in for ObjectWorldClient used in offline execution and tests."""

  def __init__(self, objects: dict[str, Any] | None = None) -> None:
    self.transforms: dict[tuple[str, str], Any] = {}
    self.joint_positions: dict[str, Any] = {}
    self.reparented: list[tuple[str, str]] = []
    self._objects: dict[str, Any] = dict(objects or {})
    for name, obj in self._objects.items():
      setattr(self, name, obj)

  @staticmethod
  def name_of(node: Any) -> str:
    """Resolves a world node, frame, or bare string to its name."""
    if isinstance(node, str):
      return node
    return getattr(node, "name", None) or getattr(node, "id", None) or str(node)

  def add_object(self, name: str, obj: Any) -> None:
    """Registers a scene object under the given name."""
    self._objects[name] = obj
    setattr(self, name, obj)

  def list_objects(self) -> list[Any]:
    """Returns every registered scene object."""
    return list(self._objects.values())

  def get_object(self, name: Any) -> Any:
    """Returns the object registered under name or matching short name, or None."""
    key = self.name_of(name)
    if key in self._objects:
      return self._objects[key]
    short = key.split(".")[-1]
    for k, obj in self._objects.items():
      if k.split(".")[-1] == short or getattr(obj, "name", None) == short:
        return obj
    return None

  def find_object(self, name: str) -> Any:
    obj = self.get_object(name)
    if obj is None:
      raise ValueError(f"Object '{name}' not found in MockWorld.")
    return obj

  def resolve_frame(self, object_name: str, frame_name: str) -> Any:
    return f"{object_name}/{frame_name}"

  def delete_object(self, obj: Any, force: bool = False) -> None:
    del force
    name = self.name_of(obj)
    if name not in self._objects:
      raise KeyError(f"No such object in world: {name}")
    del self._objects[name]
    if hasattr(self, name):
      delattr(self, name)

  def delete_frame(self, frame: Any, force: bool = False) -> None:
    del frame, force

  def get_transform(self, node_a: Any, node_b: Any) -> Any:
    return self.transforms.get((self.name_of(node_a), self.name_of(node_b)))

  def update_transform(self, node_a: Any, node_b: Any, a_t_b: Any) -> None:
    self.transforms[(self.name_of(node_a), self.name_of(node_b))] = a_t_b

  def update_joint_positions(self, target: Any, joint_positions: Any) -> None:
    self.joint_positions[self.name_of(target)] = joint_positions

  def reparent_object(
    self,
    child_object: Any = None,
    new_parent: Any = None,
    target: Any = None,
    parent: Any = None,
  ) -> None:
    c = child_object if child_object is not None else target
    p = new_parent if new_parent is not None else parent
    self.reparented.append((self.name_of(c), self.name_of(p)))

  def build_attach_to_gripper_task(
    self,
    object_name: str,
    gripper_name: str = "gripper",
    name: str | None = None,
  ) -> bt.Node:
    label = name or f"Attach {object_name} to {gripper_name}"
    return bt.Task(action=bt.PythonScript(function_body="pass"), name=label)

  def build_detach_from_gripper_task(
    self,
    object_name: str,
    gripper_name: str = "gripper",
    name: str | None = None,
  ) -> bt.Node:
    label = name or f"Detach {object_name} from {gripper_name}"
    return bt.Task(action=bt.PythonScript(function_body="pass"), name=label)

  def build_joint_update_task(
    self,
    object_name: str,
    joints: JointPosition | Sequence[float],
    name: str | None = None,
    task_name: str | None = None,
  ) -> bt.Node:
    del joints
    label = name or task_name or f"Update joints for {object_name}"
    return bt.Task(action=bt.PythonScript(function_body="pass"), name=label)

  def build_update_grasp_frames_task(
    self,
    estimates: Any,
    camera_name: str = "orbbec_camera",
    target_scene_object_id: str = "raw_stock_2x3x5",
    parent_object: str = "root",
    pregrasp_frame_name: str = "infeed_pre_grasp",
    grasp_frame_name: str = "infeed_grasp",
    approach_offset_z: float = 0.05,
    min_safe_z: float = 0.95,
    name: str | None = None,
  ) -> bt.Node:
    del (
      estimates,
      camera_name,
      target_scene_object_id,
      parent_object,
      pregrasp_frame_name,
      grasp_frame_name,
      approach_offset_z,
      min_safe_z,
    )
    label = name or "Calculate & Update Dynamic Grasp & Pre-Grasp Frames"
    return bt.Task(action=bt.PythonScript(function_body="pass"), name=label)

  def clear_stale_frames(self, names: Sequence[str]) -> list[str]:
    return list(names)

  def reset(
    self,
    workpiece_name: str = "raw_stock_2x3x5",
    dynamic_frames: Sequence[str] = ("infeed_grasp", "infeed_pre_grasp"),
    scene_update_files: Sequence[str] = ("configs/omts/scene.updates.pbtxt",),
    robot: Any | None = None,
    solution: Any | None = None,
    clear_faults: bool = False,
    **kwargs: Any,
  ) -> list[str]:
    del workpiece_name, scene_update_files, solution, kwargs
    if clear_faults and robot is not None and hasattr(robot, "clear_faults"):
      robot.clear_faults()
    return list(dynamic_frames)


class MockSolution:
  """Stand-in solution handle for `--mock_hardware` and unit tests."""

  def __init__(self, world: MockWorld | None = None) -> None:
    """Initializes the mock solution."""
    self.world = world if world is not None else MockWorld()
    self.executive = OfflineExecutive()
    self.skills = None
    self.proto_builder = None
    self.resources: dict[str, Any] = {}

  def run(self, tree: bt.Node, simulation_mode: Any = None) -> None:
    self.executive.run(tree, simulation_mode=simulation_mode)


def resolve_world(
  solution: Any, world: WorldInterface | None = None
) -> WorldInterface:
  """Returns the world facade, wrapping a raw ObjectWorld handle if needed."""
  if isinstance(world, WorldInterface):
    return world
  candidate = getattr(solution, "world", None)
  if isinstance(candidate, WorldInterface):
    return candidate
  return World(candidate, solution=solution)


__all__ = [
  "MockSolution",
  "MockWorld",
  "OfflineExecutive",
  "World",
  "WorldInterface",
  "resolve_world",
]
