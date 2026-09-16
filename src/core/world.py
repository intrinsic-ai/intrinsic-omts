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

"""World interface, Flowstate ObjectWorld facade, and in-memory mock implementation."""

import abc
import logging
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import proto_building as pb

from src.core.types import JointPosition
from src.utils.math_utils import create_transform_node_ref
from src.utils.script_utils import ScriptArg, load_python_script

__all__ = [
  "MockWorld",
  "World",
  "WorldInterface",
  "reparent_object_script",
  "update_object_joints_script",
]


def reparent_object_script(context: Any, params: Any) -> None:
  """BT PythonScript entrypoint reparenting an object in ObjectWorld."""
  world = context.object_world
  target_name = getattr(params, "object_name", "raw_stock_2x3x5")
  parent_name = getattr(params, "parent_name", "root")

  # Prevent attaching directly to static environment fixtures (avoids entity desync).
  if any(k in parent_name.lower() for k in ("schunk", "vise", "cnc_enclosure")):
    parent_name = "root"

  short_target = target_name.split(".")[-1]
  child_obj = getattr(world, target_name, None) or getattr(
    world, short_target, None
  )
  if child_obj is None and hasattr(world, "get_object"):
    for cand in (target_name, short_target):
      try:
        child_obj = world.get_object(cand)
        if child_obj is not None:
          break
      except Exception:
        pass

  parent_obj = getattr(world, parent_name, None)
  if parent_obj is None and hasattr(world, "get_object"):
    try:
      parent_obj = world.get_object(parent_name)
    except Exception:
      pass

  if child_obj is None or parent_obj is None:
    logging.warning(
      "Cannot reparent '%s' to '%s': object not found.",
      target_name,
      parent_name,
    )
    return

  curr_parent = getattr(child_obj, "parent", None)
  curr_parent_name = (
    getattr(curr_parent, "name", None) if curr_parent is not None else None
  )
  target_parent_name = getattr(parent_obj, "name", parent_name)
  if curr_parent_name in (target_parent_name, parent_name):
    return

  if hasattr(world, "reparent_object"):
    world.reparent_object(child_object=child_obj, new_parent=parent_obj)
    if hasattr(child_obj, "parent"):
      child_obj.parent = parent_obj


def update_object_joints_script(context: Any, params: Any) -> None:
  """BT PythonScript entrypoint updating joint positions on a world object."""
  world = context.object_world
  object_name = getattr(params, "object_name", "")
  positions = [float(p) for p in getattr(params, "positions", ())]
  if not object_name:
    return

  obj = getattr(world, object_name, None)
  if obj is None and hasattr(world, "get_kinematic_object"):
    try:
      obj = world.get_kinematic_object(object_name)
    except Exception:
      pass
  if obj is None and hasattr(world, "get_object"):
    try:
      obj = world.get_object(object_name)
    except Exception:
      pass

  if obj is not None and hasattr(world, "update_joint_positions"):
    world.update_joint_positions(obj, positions)
  elif obj is not None and hasattr(obj, "set_joint_positions"):
    obj.set_joint_positions(positions)


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
  def build_reparent_task(
    self,
    target: str,
    new_parent: str,
    name: str | None = None,
    task_name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task that reparents target under new_parent."""
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
    **kwargs: Any,
  ) -> list[str]:
    """Resets workpiece parentage, clears dynamic frames, and reapplies scene updates."""
    raise NotImplementedError


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
    """Returns the object registered under name, or None."""
    return self._objects.get(self.name_of(name))

  def find_object(self, name: str) -> Any:
    short = name.split(".")[-1]
    obj = self._objects.get(name) or self._objects.get(short)
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

  def build_reparent_task(
    self,
    target: str,
    new_parent: str,
    name: str | None = None,
    task_name: str | None = None,
  ) -> bt.Node:
    label = name or task_name or f"Reparent {target} to {new_parent}"
    return bt.Task(
      action=bt.PythonScript(function_body="pass"),
      name=label,
    )

  def build_joint_update_task(
    self,
    object_name: str,
    joints: JointPosition | Sequence[float],
    name: str | None = None,
    task_name: str | None = None,
  ) -> bt.Node:
    del joints
    label = name or task_name or f"Update joints for {object_name}"
    return bt.Task(
      action=bt.PythonScript(function_body="pass"),
      name=label,
    )

  def clear_stale_frames(self, names: Sequence[str]) -> list[str]:
    return list(names)

  def reset(
    self,
    workpiece_name: str = "raw_stock_2x3x5",
    dynamic_frames: Sequence[str] = ("infeed_grasp", "infeed_pre_grasp"),
    scene_update_files: Sequence[str] = ("configs/omts/scene.updates.pbtxt",),
    robot: Any | None = None,
    solution: Any | None = None,
    **kwargs: Any,
  ) -> list[str]:
    del scene_update_files
    target = kwargs.get("workpiece_object_name") or workpiece_name
    self.ensure_workpiece_at_root(target)
    if robot is not None and hasattr(robot, "clear_faults"):
      robot.clear_faults()
    if solution is not None and hasattr(solution, "clear_motion_planner_cache"):
      solution.clear_motion_planner_cache()
    return [f"reparent:{target}"] + list(dynamic_frames)


class World(WorldInterface):
  """Facade over live Flowstate solution world-graph lookups and mutation tasks."""

  def __init__(self, raw_world: Any, solution: Any | None = None) -> None:
    self._raw_world = raw_world
    self._solution = solution

  @property
  def raw(self) -> Any:
    """Returns the underlying Flowstate ObjectWorld handle."""
    return self._raw_world

  def find_object(self, name: str) -> Any:
    """Resolves an object in ObjectWorld by full name or short asset identifier."""
    if self.raw is None:
      raise ValueError(f"Cannot find object '{name}': world is None.")

    short_name = name.split(".")[-1]
    candidates = [name]
    if short_name not in candidates:
      candidates.append(short_name)

    if hasattr(self.raw, "get_object"):
      for cand in candidates:
        try:
          obj = self.raw.get_object(cand)
          if obj is not None:
            return obj
        except Exception:
          pass

    if hasattr(self.raw, "list_objects"):
      try:
        cand_set = set(candidates)
        for obj in self.raw.list_objects():
          obj_id = getattr(obj, "name", None) or getattr(obj, "id", None)
          if obj_id in cand_set:
            return obj
      except Exception:
        pass

    for cand in candidates:
      if hasattr(self.raw, cand):
        obj = getattr(self.raw, cand)
        if obj is not None:
          return obj

    raise ValueError(f"Object '{name}' not found in ObjectWorld.")

  def resolve_frame(self, object_name: str, frame_name: str) -> Any:
    """Resolves a transform node for the given object and frame name."""
    if hasattr(self.raw, "get_transform_node"):
      try:
        ref = create_transform_node_ref(object_name, frame_name)
        return self.raw.get_transform_node(ref)
      except Exception:
        pass

    obj = self.find_object(object_name)
    if hasattr(obj, "get_frame"):
      return obj.get_frame(frame_name)
    if hasattr(obj, frame_name):
      return getattr(obj, frame_name)
    return frame_name

  def ensure_workpiece_at_root(
    self, workpiece_name: str = "raw_stock_2x3x5"
  ) -> None:
    """Ensures the workpiece is parented to 'root' prior to starting a cycle."""
    if self.raw is None:
      return
    try:
      obj = self.find_object(workpiece_name)
      parent = getattr(obj, "parent", None)
      parent_name = getattr(parent, "name", None) or getattr(parent, "id", None)
      if parent_name and parent_name != "root":
        logging.info(
          "Startup reparenting: '%s' is attached to '%s'. Reparenting to 'root'.",
          getattr(obj, "name", workpiece_name),
          parent_name,
        )
        root_obj = getattr(self.raw, "root", "root")
        if hasattr(self.raw, "reparent_object"):
          try:
            self.raw.reparent_object(child_object=obj, new_parent=root_obj)
          except TypeError:
            self.raw.reparent_object(obj, root_obj)
    except Exception as err:
      logging.warning("Startup workpiece reparenting check failed: %s", err)

  def build_reparent_task(
    self,
    target: str,
    new_parent: str,
    name: str | None = None,
    task_name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task that reparents target under new_parent."""
    label = name or task_name or f"Reparent {target} to {new_parent}"
    signature = None
    if self._solution is not None and getattr(
      self._solution, "proto_builder", None
    ):
      args = [
        ScriptArg(1, "object_name", "string", target),
        ScriptArg(2, "parent_name", "string", new_parent),
      ]
      signature = self._solution.proto_builder.create_signature_with_args(
        parameters=pb.MessageSpec(fields=[a.to_field_spec() for a in args])
      )

    action = bt.PythonScript(
      signature_with_args=signature,
      function_body=load_python_script(reparent_object_script),
    )
    return bt.Task(action=action, name=label)

  def build_joint_update_task(
    self,
    object_name: str,
    joints: JointPosition | Sequence[float],
    name: str | None = None,
    task_name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task updating prismatic joints for object_name."""
    label = name or task_name or f"Update joints for {object_name}"
    vals = [
      float(p)
      for p in (
        joints.positions if isinstance(joints, JointPosition) else joints
      )
    ]
    signature = None
    if self._solution is not None and getattr(
      self._solution, "proto_builder", None
    ):
      args = [
        ScriptArg(1, "object_name", "string", object_name),
        ScriptArg(2, "positions", "float", vals, repeated=True),
      ]
      signature = self._solution.proto_builder.create_signature_with_args(
        parameters=pb.MessageSpec(fields=[a.to_field_spec() for a in args])
      )

    action = bt.PythonScript(
      signature_with_args=signature,
      function_body=load_python_script(update_object_joints_script),
    )
    return bt.Task(action=action, name=label)

  def clear_stale_frames(self, names: Sequence[str]) -> list[str]:
    """Removes dynamic frames from root object in the world."""
    if self.raw is None:
      return []
    root_obj = getattr(self.raw, "root", None)
    cleared = []
    for df_name in names:
      if hasattr(self.raw, "delete_frame"):
        target_frame = None
        if root_obj is not None:
          target_frame = getattr(root_obj, df_name, None)
          if target_frame is None and hasattr(root_obj, "get_frame"):
            try:
              target_frame = root_obj.get_frame(df_name)
            except Exception:
              pass
        if target_frame is None:
          target_frame = df_name
        try:
          try:
            self.raw.delete_frame(target_frame, force=True)
          except TypeError:
            self.raw.delete_frame(target_frame)
          cleared.append(df_name)
          logging.info("Deleted stale dynamic frame: %s", df_name)
        except Exception as err:
          logging.warning("Failed to delete frame %s: %s", df_name, err)
    return cleared

  def reset(
    self,
    workpiece_name: str = "raw_stock_2x3x5",
    dynamic_frames: Sequence[str] = ("infeed_grasp", "infeed_pre_grasp"),
    scene_update_files: Sequence[str] = ("configs/omts/scene.updates.pbtxt",),
    robot: Any | None = None,
    solution: Any | None = None,
    **kwargs: Any,
  ) -> list[str]:
    """Resets workpiece parentage, clears dynamic frames, and reapplies scene updates."""
    from tools.world.apply_scene_updates import apply_pbtxt_file

    target = kwargs.get("workpiece_object_name") or workpiece_name
    reset_actions: list[str] = []
    self.ensure_workpiece_at_root(target)
    reset_actions.append(f"reparent:{target}")

    cleared = self.clear_stale_frames(dynamic_frames)
    reset_actions.extend(f"frame:{f}" for f in cleared)

    if self.raw is not None:
      for fpath in scene_update_files:
        try:
          apply_pbtxt_file(world=self.raw, filepath=fpath)
        except Exception as err:
          logging.warning("Failed to reapply scene update %s: %s", fpath, err)

    if robot is not None and hasattr(robot, "clear_faults"):
      robot.clear_faults()

    sol = solution or self._solution
    if sol is not None and hasattr(sol, "clear_motion_planner_cache"):
      sol.clear_motion_planner_cache()

    return reset_actions
