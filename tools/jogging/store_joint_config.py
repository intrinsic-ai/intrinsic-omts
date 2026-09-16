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

"""Script to store current robot joint configuration with a given name in SBL world."""

import argparse
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import deployments, worlds
from intrinsic.world.proto import object_world_updates_pb2

from src.utils.math_utils import normalize_joint_angles


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
    description=(
      "Store current robot joint position as a named joint configuration."
    )
  )
  parser.add_argument(
    "name",
    type=str,
    help=(
      "Name of the joint configuration to store (e.g. 'home', 'view_pose')."
    ),
  )
  parser.add_argument(
    "--address",
    type=str,
    default="localhost:17080",
    help="Solution address to connect to (default: localhost:17080).",
  )
  parser.add_argument(
    "--robot_name",
    type=str,
    default="ur_module",
    help="Name of the robot object in the world (default: ur_module).",
  )
  return parser.parse_args(argv)


def store_joint_configuration(
  solution: Any,
  name: str,
  robot_name: str = "ur_module",
) -> tuple[list[float], list[float]]:
  """Reads current robot joint positions and stores as named configuration."""
  world = solution.world

  try:
    robot = getattr(world, robot_name)
  except AttributeError:
    robot = world.get_kinematic_object(robot_name)

  current_joint_positions = list(robot.joint_positions)
  normalized_joint_positions = normalize_joint_angles(current_joint_positions)

  print(
    f"Current joint positions for '{robot_name}': {current_joint_positions}"
  )
  print(f"Normalized joint positions: {normalized_joint_positions}")

  named_config = object_world_updates_pb2.NamedJointConfiguration(
    name=name,
    joint_positions=normalized_joint_positions,
  )

  print(f"Storing joint configuration '{name}' to active belief world...")
  world.update_kinematic_object_joint_configurations(
    kinematic_object=robot,
    named_joint_configurations_to_set=[named_config],
  )

  print(
    f"Storing joint configuration '{name}' to initial world ('init_world')..."
  )
  try:
    init_world = worlds.ObjectWorld.connect(
      world_id=worlds.EditWorldId.INITIAL,
      grpc_channel=solution.grpc_channel,
    )
    try:
      init_robot = getattr(init_world, robot_name)
    except AttributeError:
      init_robot = init_world.get_kinematic_object(robot_name)
    init_world.update_kinematic_object_joint_configurations(
      kinematic_object=init_robot,
      named_joint_configurations_to_set=[named_config],
    )
    print(
      f"Saved named joint configuration '{name}' permanently to init_world."
    )
  except Exception as e:  # pylint: disable=broad-exception-caught
    print(f"Warning: Could not update init_world: {e}")

  return current_joint_positions, normalized_joint_positions


def main(argv: Sequence[str] | None = None) -> None:
  """Connects to solution and stores the robot's current joint positions."""
  args = parse_args(argv)

  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  store_joint_configuration(
    solution=solution,
    name=args.name,
    robot_name=args.robot_name,
  )


if __name__ == "__main__":
  main()
