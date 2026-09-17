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

"""CLI tool to fetch and update calibrated robot kinematics from hardware."""

from collections.abc import Sequence

import grpc
from absl import app, flags, logging
from intrinsic.world.service.robot_calibration import (
  robot_update_service_pb2,
  robot_update_service_pb2_grpc,
)

_ADDRESS = flags.DEFINE_string(
  "address",
  "localhost:17080",
  "gRPC address of the running SBL solution deployment.",
)
_RESOURCE_ID = flags.DEFINE_string(
  "resource_id",
  "ur_module",
  "Resource ID of the robot module in the world (e.g. 'ur_module').",
)
_WORLD_ID = flags.DEFINE_string(
  "world_id",
  "world",
  "World ID containing the robot model (default: 'world').",
)
_CHECK_ONLY = flags.DEFINE_bool(
  "check_only",
  False,
  "Only check if the world model matches hardware calibration without"
  " modifying the world.",
)
_TIMEOUT = flags.DEFINE_float(
  "timeout",
  60.0,
  "Timeout in seconds for the calibration update RPC.",
)


def create_robot_update_stub(
  address: str,
) -> tuple[grpc.Channel, robot_update_service_pb2_grpc.RobotUpdateServiceStub]:
  """Creates a gRPC channel and stub for RobotUpdateService."""
  channel = grpc.insecure_channel(address)
  stub = robot_update_service_pb2_grpc.RobotUpdateServiceStub(channel)
  return channel, stub


def check_kinematics_match(
  stub: robot_update_service_pb2_grpc.RobotUpdateServiceStub,
  resource_id: str,
  world_id: str,
  timeout: float = 10.0,
) -> bool:
  """Checks whether the world model kinematics match hardware calibration."""
  request = robot_update_service_pb2.CheckWorldMatchesHardwareKinematicsRequest(
    resource_id=resource_id,
    world_id=world_id,
  )
  try:
    response = stub.CheckWorldMatchesHardwareKinematics(
      request, timeout=timeout
    )
    return response.world_matches_hardware_kinematics
  except grpc.RpcError as e:
    logging.error("Failed to check kinematics match: %s", e)
    raise


def update_robot_kinematics(
  stub: robot_update_service_pb2_grpc.RobotUpdateServiceStub,
  resource_id: str,
  world_id: str,
  timeout: float = 60.0,
) -> None:
  """Requests the RobotUpdateService to fetch calib from hardware & update."""
  request = robot_update_service_pb2.RobotKinematicsUpdateRequest(
    resource_id=resource_id,
    world_id=world_id,
  )
  try:
    stub.UpdateRobotKinematics(request, timeout=timeout)
    logging.info(
      "Successfully updated robot kinematics in world '%s' for resource '%s'.",
      world_id,
      resource_id,
    )
  except grpc.RpcError as e:
    logging.error("Failed to update robot kinematics: %s", e)
    raise


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  address = _ADDRESS.value
  resource_id = _RESOURCE_ID.value
  world_id = _WORLD_ID.value
  check_only = _CHECK_ONLY.value
  timeout = _TIMEOUT.value

  logging.info("Connecting to RobotUpdateService at %s...", address)
  channel, stub = create_robot_update_stub(address)

  try:
    try:
      matches_before = check_kinematics_match(
        stub, resource_id, world_id, timeout=min(timeout, 10.0)
      )
    except grpc.RpcError as e:
      if hasattr(e, "code") and e.code() == grpc.StatusCode.FAILED_PRECONDITION:
        logging.warning(
          "Hardware module '%s' has no calibrated kinematics available "
          "(robot controller may be disconnected or unreachable). "
          "Skipping kinematics update.",
          resource_id,
        )
        return
      raise

    if matches_before:
      logging.info(
        "World '%s' kinematics ALREADY MATCH physical hardware calibration for"
        " resource '%s'.",
        world_id,
        resource_id,
      )
      if check_only:
        return
    else:
      logging.warning(
        "World '%s' kinematics DO NOT MATCH physical hardware calibration for"
        " resource '%s'. (Currently using nominal model)",
        world_id,
        resource_id,
      )

    if check_only:
      raise app.UsageError(
        f"World kinematics do not match hardware for {resource_id}."
      )

    logging.info(
      "Requesting hardware module '%s' calibration from controller...",
      resource_id,
    )
    try:
      update_robot_kinematics(stub, resource_id, world_id, timeout=timeout)
    except grpc.RpcError as e:
      if hasattr(e, "code") and e.code() == grpc.StatusCode.FAILED_PRECONDITION:
        logging.warning(
          "Failed to fetch calibrated kinematics for '%s' "
          "(robot controller may be disconnected or unreachable).",
          resource_id,
        )
        return
      raise

    matches_after = check_kinematics_match(
      stub, resource_id, world_id, timeout=min(timeout, 10.0)
    )
    if matches_after:
      logging.info(
        "Verification confirmed: World '%s' kinematics now match hardware"
        " calibration.",
        world_id,
      )
    else:
      logging.warning(
        "Update completed but verification check reported mismatch for '%s'.",
        resource_id,
      )
  finally:
    channel.close()


if __name__ == "__main__":
  app.run(main)
