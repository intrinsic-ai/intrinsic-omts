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

"""Hardware abstractions and adapters for the Open Machine Tending Solution."""

from src.hardware.gripper import (
  DioGripper,
  GripperInterface,
  MockGripper,
  RobotiqGripper,
  SideloadedGripperCmd,
)
from src.hardware.machine import (
  CncMachineInterface,
  DioCncMachine,
  MockCncMachine,
)
from src.hardware.robot import MockRobot, RobotInterface, UrRobot
from src.hardware.vision import MockVision, OrbbecVision, VisionInterface

__all__ = [
  "CncMachineInterface",
  "DioCncMachine",
  "DioGripper",
  "GripperInterface",
  "MockCncMachine",
  "MockGripper",
  "MockRobot",
  "MockVision",
  "OrbbecVision",
  "RobotInterface",
  "RobotiqGripper",
  "SideloadedGripperCmd",
  "UrRobot",
  "VisionInterface",
]
