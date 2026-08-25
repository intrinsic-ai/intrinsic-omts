"""Vision and 3D camera hardware interfaces and implementations."""

import abc
from typing import Any, Optional
from intrinsic.solutions import behavior_tree as bt
from src.core.types import Pose3D


class VisionInterface(abc.ABC):
  """Abstract interface for perception acquisition and pose estimation."""

  @abc.abstractmethod
  def build_capture_image_task(self, name: Optional[str] = None) -> bt.Node:
    """Builds a task to trigger camera image acquisition."""
    raise NotImplementedError


class OrbbecVision(VisionInterface):
  """Orbbec 3D camera perception adapter using SBL capture_images_skill."""

  def __init__(
      self,
      solution: Any,
      camera_name: str = "orbbec_camera",
  ) -> None:
    self._solution = solution
    self._camera_name = camera_name
    self._capture_images_skill = solution.skills.ai.intrinsic.capture_images

  def build_capture_image_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or f"Capture Image ({self._camera_name})"
    action = self._capture_images_skill(
        camera=getattr(self._solution.world, self._camera_name),
    )
    return bt.Task(action=action, name=task_name)


class MockVision(VisionInterface):
  """Mock vision sensor for offline testing."""

  def __init__(self, simulated_pose: Optional[Pose3D] = None) -> None:
    self.simulated_pose = simulated_pose or Pose3D(x=0.45, y=0.15, z=0.02)
    self.capture_count: int = 0

  def build_capture_image_task(self, name: Optional[str] = None) -> bt.Node:
    self.capture_count += 1
    return bt.Sequence([])
