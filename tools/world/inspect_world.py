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

"""Utility to inspect scene objects, frames, and joint configs."""

import argparse
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import deployments


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
    description=(
      "Inspect objects, frames, and joint configurations in the solution world."
    )
  )
  parser.add_argument(
    "--address",
    type=str,
    default="localhost:17080",
    help="Solution address to connect to (default: localhost:17080).",
  )
  return parser.parse_args(argv)


def print_transform(label: str, tf: Any) -> None:
  """Helper to print a Pose3 transform in a clean, readable format."""
  pos = tf.translation
  quat = tf.rotation.quaternion
  rpy_deg = tf.rotation.euler_angles(radians=False)
  print(f"{label}:")
  print(f"  pos:     [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
  print(f"  quat:    {quat}")
  print(f"  rpy_deg: [{rpy_deg[0]:.2f}, {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f}]")


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)
  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  world = solution.world

  print("\n=== Objects in Active Belief World ===")
  try:
    if hasattr(world, "_stub"):
      for m in dir(world._stub):
        if "Collision" in m or "World" in m:
          print(f"Stub method: {m}")
      from intrinsic.world.proto import object_world_service_pb2

      req = object_world_service_pb2.GetCollisionSettingsRequest(
        world_id=world.world_id
      )
      col_settings = world._stub.GetCollisionSettings(req)
      print(f"Collision rules count: {len(col_settings.collision_rules)}")
      for idx, rule in enumerate(col_settings.collision_rules):
        rule_str = str(rule)
        if (
          "1056" in rule_str or "raw_stock" in rule_str or "gripper" in rule_str
        ):
          print(f"Rule {idx}:\n{rule}")
  except Exception as e:
    print(f"Error getting collision settings: {e}")
  objects = []
  try:
    if hasattr(world, "list_objects"):
      objects = world.list_objects()
      print(f"Total objects: {len(objects)}")
      all_known_entities = {}
      all_parent_refs = []
      for obj in objects:
        parent = getattr(obj, "parent", None)
        parent_name = getattr(parent, "name", str(parent)) if parent else "None"
        print(
          f"\nObject: {obj.name} (id={getattr(obj, 'id', None)}, parent={parent_name})"
        )
        if hasattr(obj, "joint_positions") and obj.joint_positions:
          print(f"  Joint positions: {obj.joint_positions}")
        if hasattr(obj, "joint_entity_names") and obj.joint_entity_names:
          print(f"  Joint entity names: {obj.joint_entity_names}")
        if (
          hasattr(obj, "joint_application_limits")
          and obj.joint_application_limits
        ):
          print(f"  Joint application limits:\n{obj.joint_application_limits}")
        if hasattr(obj, "joint_system_limits") and obj.joint_system_limits:
          print(f"  Joint system limits:\n{obj.joint_system_limits}")
        frames = []
        if hasattr(obj, "list_frames"):
          frames = obj.list_frames()
        elif hasattr(obj, "frame_names"):
          frames = obj.frame_names
        elif hasattr(obj, "frames"):
          frames = [
            f.name if hasattr(f, "name") else str(f) for f in obj.frames
          ]
        print(f"  Frames ({len(frames)}): {frames}")
        try:
          proto = world._get_object_proto(obj.name)
          if proto and proto.entities:
            print(f"  Entities ({len(proto.entities)}):")
            for _ent_id, ent in proto.entities.items():
              print(
                f"    - {ent.name} (id={ent.id}, parent_id={ent.parent_id})"
              )
              all_known_entities[str(ent.id)] = (obj.name, ent.name)
              all_parent_refs.append(
                (obj.name, ent.name, str(ent.id), str(ent.parent_id))
              )
        except Exception as err:
          print(f"  Could not get object proto: {err}")
    elif hasattr(world, "list_object_names"):
      names = world.list_object_names()
      print(f"Object names: {names}")
    print("\n=== Entity Graph Integrity Check ===")
    print(f"Total entities recorded: {len(all_known_entities)}")
    missing_parents = []
    for obj_name, ent_name, ent_id, parent_id in all_parent_refs:
      if (
        parent_id
        and parent_id not in all_known_entities
        and parent_id != "eid_root"
        and parent_id != "0"
      ):
        missing_parents.append((obj_name, ent_name, ent_id, parent_id))
    if missing_parents:
      print("WARNING: FOUND ENTITIES WITH MISSING PARENT ENTITIES:")
      for obj_name, ent_name, ent_id, parent_id in missing_parents:
        print(
          f"  Object '{obj_name}' entity '{ent_name}' (id={ent_id}) has MISSING parent_id={parent_id}!"
        )
    else:
      print("All entity parent references resolved within known entities.")
    for eid, (obj_name, ent_name) in all_known_entities.items():
      if "1056" in eid:
        print(
          f"MATCH FOR 1056: eid={eid} in object '{obj_name}' entity '{ent_name}'"
        )
  except Exception as e:
    print(f"Error listing objects: {e}")

  print("\n=== All Frames in World ===")
  total_frames = 0
  try:
    for obj in objects:
      frame_list = []
      if hasattr(obj, "frames") and callable(obj.frames):
        frame_list = obj.frames()
      elif hasattr(obj, "frames") and isinstance(obj.frames, list):
        frame_list = obj.frames
      elif hasattr(obj, "list_frames"):
        frame_list = [getattr(obj, f_name) for f_name in obj.list_frames()]
      elif hasattr(obj, "frame_names"):
        frame_list = [getattr(obj, f_name) for f_name in obj.frame_names]

      for frame in frame_list:
        frame_name = getattr(frame, "name", str(frame))
        frame_id = getattr(frame, "id", None) or getattr(
          frame, "entity_id", None
        )
        total_frames += 1
        try:
          tf_in_root = world.get_transform(world.root, frame)
          print_transform(
            f"Frame '{obj.name}.{frame_name}' (id={frame_id}) in root",
            tf_in_root,
          )
          if obj.name != "root":
            try:
              tf_in_obj = world.get_transform(obj, frame)
              pos = tf_in_obj.translation
              rpy_deg = tf_in_obj.rotation.euler_angles(radians=False)
              print(
                f"  (rel to {obj.name}: pos=[{pos[0]:.4f}, {pos[1]:.4f},"
                f" {pos[2]:.4f}], rpy_deg=[{rpy_deg[0]:.2f},"
                f" {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f}])"
              )
            except Exception as rel_err:
              print(
                f"  (rel to {obj.name}: unable to resolve relative"
                f" transform: {rel_err})"
              )
        except Exception as e:
          print(
            f"Frame '{obj.name}.{frame_name}': error retrieving transform: {e}"
          )
    print(f"\nTotal frames found: {total_frames}")
  except Exception as e:
    print(f"Error enumerating frames: {e}")

  print("\n=== Object Transforms in Root ===")
  try:
    for obj in objects:
      if obj.name == "root":
        continue
      try:
        tf = world.get_transform(world.root, obj)
        print_transform(f"Object '{obj.name}' in root", tf)
      except Exception as e:
        print(f"Object '{obj.name}': error retrieving transform in root: {e}")
  except Exception as e:
    print(f"Error inspecting object transforms: {e}")

  print("\n=== Solution Resources ===")
  try:
    solution.resources.update()
    for res_name in dir(solution.resources):
      if not res_name.startswith("_"):
        res = getattr(solution.resources, res_name)
        if hasattr(res, "name"):
          print(f" - {res.name}")
  except Exception as e:
    print(f"   Error listing resources: {e}")

  print("\n=== Solution Pose Estimators ===")
  try:
    if hasattr(solution, "pose_estimators"):
      for est_name in dir(solution.pose_estimators):
        if not est_name.startswith("_"):
          print(f" - {est_name}")
  except Exception as e:
    print(f"   Error listing pose estimators: {e}")

  print("\n=== Solution Skills ===")
  try:
    if hasattr(solution, "skills"):
      if hasattr(solution.skills, "ai") and hasattr(
        solution.skills.ai, "intrinsic"
      ):
        for skill_name in dir(solution.skills.ai.intrinsic):
          if not skill_name.startswith("_"):
            print(f" - ai.intrinsic.{skill_name}")
  except Exception as e:
    print(f"   Error listing skills: {e}")


if __name__ == "__main__":
  main()
