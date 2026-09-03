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

"""Script to sample calibration poses and export them to a file."""

import datetime
import os
import sys
import termios
import threading
import time
import tty
from typing import Any, Optional

from absl import app
from absl import flags
from google.protobuf import text_format
import grpc

from intrinsic.executive.proto import run_metadata_pb2
from intrinsic.icon.proto import joint_space_pb2
from intrinsic.icon.proto.v1 import service_pb2_grpc
from intrinsic.icon.python import create_action_utils
from intrinsic.icon.python import errors as icon_errors
from intrinsic.icon.python import icon_api
from intrinsic.motion_planning.public.proto.v1 import geometric_constraints_pb2
from intrinsic.perception.public.proto.v1 import camera_to_robot_calibration_pb2 as calibration_type_pb2
from intrinsic.perception.skills.calibration import sample_calibration_poses_pb2
from intrinsic.skills.proto import skills_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments
from intrinsic.util.grpc import connection

# Command line input flags
_ADDRESS = flags.DEFINE_string(
    'address', 'localhost:17080', 'Solution address to connect to.'
)
_ROBOT = flags.DEFINE_string(
    'robot', 'icon', 'Robot / controller resource name in the workcell.'
)
_CAMERA = flags.DEFINE_string(
    'camera', 'orbbec_camera', 'Camera name in the workcell.'
)
_CALIBRATION_OBJECT = flags.DEFINE_string(
    'calibration_object',
    'charuco_9x14_20mm_15mm_dict_5x5',
    'Calibration pattern / object name.',
)
_MOVING_CAMERA = flags.DEFINE_bool(
    'moving_camera',
    True,
    'Whether it is a moving camera calibration. If false, it is stationary.',
)
_SAMPLE_BOX_HALFSIZE_X = flags.DEFINE_float(
    'sample_box_halfsize_x',
    0.15,
    'Sampling box half-size X dimension (meters).',
)
_SAMPLE_BOX_HALFSIZE_Y = flags.DEFINE_float(
    'sample_box_halfsize_y',
    0.15,
    'Sampling box half-size Y dimension (meters).',
)
_SAMPLE_BOX_HALFSIZE_Z = flags.DEFINE_float(
    'sample_box_halfsize_z',
    0.15,
    'Sampling box half-size Z dimension (meters).',
)
_RAND_ANGLE = flags.DEFINE_float(
    'rand_angle', 15.0, 'Randomization angle in degrees.'
)
_RAND_ROLL_ANGLE = flags.DEFINE_float(
    'rand_roll_angle', 45.0, 'Randomization roll angle in degrees.'
)
_NUM_SAMPLES = flags.DEFINE_integer(
    'num_samples', 25, 'Number of calibration samples.'
)
_MANUAL_WAYPOINTS = flags.DEFINE_bool(
    'manual_waypoints',
    True,
    'If True, collect waypoints manually instead of sampling automatically.',
)
_CAPTURE_IMAGES = flags.DEFINE_bool(
    'capture_images',
    True,
    'Whether to capture images from the camera when recording waypoints.',
)
_EXPORT_WAYPOINTS_FILE = flags.DEFINE_string(
    'export_waypoints_file',
    '',
    'If set, export the manual waypoints to this local file path.',
)
_ICON_PORT = flags.DEFINE_integer(
    'icon_port', 17080, 'Local port mapped to ICON service via port-forward.'
)
_STREAM_CAMERA = flags.DEFINE_bool(
    'stream_camera',
    True,
    'Whether to continuously trigger camera captures in the background to publish images to ROS topics during sampling.',
)
_STREAM_FPS = flags.DEFINE_float(
    'stream_fps',
    10.0,
    'Target frame rate (FPS) for background camera streaming to ROS topics.',
)


class CameraStreamer:
  """Background worker to continuously trigger camera captures and stream to ROS topics."""

  def __init__(
      self,
      camera: object | None = None,
      skills: Any | None = None,
      executive: Any | None = None,
      camera_ref: Any | None = None,
      fps: float = 10.0,
  ):
    self._camera = camera
    self._skills = skills
    self._executive = executive
    self._camera_ref = camera_ref
    self._interval = 1.0 / max(fps, 0.1)
    self._stop_event = threading.Event()
    self._thread: threading.Thread | None = None
    self._latest_capture: object | None = None
    self._lock = threading.Lock()
    self._capture_skill = None
    if self._skills is not None and self._camera_ref is not None:
      try:
        self._capture_skill = self._skills.ai.intrinsic.capture_images(
            camera=self._camera_ref
        )
      except Exception:
        self._capture_skill = None

  def start(self) -> None:
    if self._thread is not None and self._thread.is_alive():
      return
    self._stop_event.clear()
    self._thread = threading.Thread(
        target=self._stream_loop, daemon=True, name='camera_streamer'
    )
    self._thread.start()

  def stop(self) -> None:
    if self._thread is not None:
      self._stop_event.set()
      self._thread.join(timeout=2.0)
      self._thread = None

  def __enter__(self):
    self.start()
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    self.stop()

  def get_latest_capture(self) -> object | None:
    with self._lock:
      return self._latest_capture

  def trigger_capture(self) -> object | None:
    """Explicitly triggers a single capture."""
    try:
      if self._camera is not None and hasattr(self._camera, 'capture'):
        res = self._camera.capture()
        with self._lock:
          self._latest_capture = res
        return res
      elif self._capture_skill is not None and self._executive is not None:
        self._executive.run(self._capture_skill, silence_outputs=True)
    except Exception as e:
      print(f'Warning: Capture trigger failed: {e}')
    return None

  def _stream_loop(self) -> None:
    while not self._stop_event.is_set():
      start_time = time.time()
      try:
        if self._camera is not None and hasattr(self._camera, 'capture'):
          res = self._camera.capture()
          with self._lock:
            self._latest_capture = res
        elif self._capture_skill is not None and self._executive is not None:
          self._executive.run(self._capture_skill, silence_outputs=True)
      except Exception:
        # Suppress transient network/frame capture errors during background streaming
        pass

      elapsed = time.time() - start_time
      sleep_time = self._interval - elapsed
      if sleep_time > 0:
        self._stop_event.wait(sleep_time)


def get_key() -> str:
  """Reads a single character or escape sequence from standard input."""
  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)
  try:
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
    if ch == '\x1b':
      # It's an escape sequence
      ch += sys.stdin.read(2)
  finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  return ch


def read_input(prompt: str, choices: list[str]) -> str:
  """Reads characters from stdin until they uniquely match one of the choices."""
  sys.stdout.write(prompt)
  sys.stdout.flush()
  current_input = ''
  while True:
    ch = get_key()
    if ch in ('\x03', '\x04', ''):  # Ctrl+C, Ctrl+D, EOF
      raise KeyboardInterrupt
    if ch in ('\r', '\n'):
      if current_input in choices:
        sys.stdout.write('\n')
        sys.stdout.flush()
        return current_input
      continue
    # Handle backspace
    if ch in ('\x7f', '\x08'):
      if current_input:
        current_input = current_input[:-1]
        sys.stdout.write('\b \b')
        sys.stdout.flush()
      continue
    # Ignore escape sequences
    if len(ch) > 1:
      continue

    ch = ch.lower()
    next_input = current_input + ch
    matching_choices = [c for c in choices if c.startswith(next_input)]
    if not matching_choices:
      continue
    current_input = next_input
    sys.stdout.write(ch)
    sys.stdout.flush()
    if len(matching_choices) == 1 and matching_choices[0] == current_input:
      sys.stdout.write('\n')
      sys.stdout.flush()
      return current_input


def run_manual_waypoint_loop(
    world,
    robot_ref,
    manual_waypoints,
    session,
    ndof: int | None,
    part_name: str | None,
    icon_client,
    camera: object | None = None,
    streamer: CameraStreamer | None = None,
) -> None:
  """Runs the interactive loop to record waypoints or jog the robot."""
  print(
      'To record waypoints: jog the robot to a pose (either via Flowstate'
      " jogging panel, or by entering 'j' below to jog via terminal), and"
      " press 'r' to record the waypoint. Press 's' to stop recording"
      " waypoints. Close the Flowstate jogging panel before pressing 's'."
  )
  action_id_counter = 0
  while True:
    user_input = read_input(
        "\nEnter 'r' to record, 'j' to enter jogging mode, 's' to stop: ",
        ['r', 'j', 's'],
    )
    if user_input == 'r':
      try:
        kinematic_robot = world.get_kinematic_object(robot_ref.proto)
        joints = kinematic_robot.joint_positions
        waypoint = geometric_constraints_pb2.GeometricConstraint(
            joint_position=joint_space_pb2.JointVec(joints=joints)
        )
        manual_waypoints.append(waypoint)
        print(f'Recorded waypoint #{len(manual_waypoints)}: {joints}')
      except Exception as e:
        print(f'Error reading robot pose: {e}')

      if (camera or streamer) and _CAPTURE_IMAGES.value:
        try:
          capture_result = None
          if streamer is not None:
            capture_result = streamer.get_latest_capture()
            if capture_result is None:
              capture_result = streamer.trigger_capture()
          elif camera is not None and hasattr(camera, 'capture'):
            capture_result = camera.capture()

          if capture_result and hasattr(capture_result, 'sensor_images'):
            for name, sensor_image in capture_result.sensor_images.items():
              import numpy as np
              from PIL import Image

              array = sensor_image.array
              if array.dtype in (np.float32, np.float64):
                array = np.nan_to_num(array, nan=0.0, posinf=0.0, neginf=0.0)
                min_val = array.min()
                max_val = array.max()
                if max_val > min_val:
                  array = (array - min_val) / (max_val - min_val) * 255.0
                else:
                  array = np.zeros_like(array)
                array = array.astype(np.uint8)

              img = Image.fromarray(array)
              timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
              filename = f'captured_image_{name}_{timestamp}.png'

              workspace_dir = os.environ.get('BUILD_WORKSPACE_DIRECTORY')
              if workspace_dir:
                filepath = os.path.join(workspace_dir, filename)
              else:
                filepath = filename

              img.save(filepath)
        except Exception as e:
          print(f'Error capturing or saving image: {e}')
    elif user_input == 's':
      confirm = read_input(
          '\nConfirm robot poses are collected [y/n]: ', ['y', 'n']
      )
      if confirm == 'y':
        flowstate_confirmed = False
        while not flowstate_confirmed:
          confirm_jogging = read_input(
              '\nConfirm that, if you are using Flowstate, you have closed'
              " the robot's jogging panel [y/n]: ",
              ['y', 'n'],
          )
          if confirm_jogging == 'y':
            flowstate_confirmed = True
          elif confirm_jogging == 'n':
            pass  # loops and asks again

        if len(manual_waypoints) < 3:
          print(
              'Warning: Calibration typically requires at least 3 waypoints.'
              f' You have only recorded {len(manual_waypoints)}.'
          )
        break
      elif confirm == 'n':
        print('Continuing waypoint collection...')
    elif user_input == 'j':
      if not session or not ndof or not part_name or not icon_client:
        print(
            'Error: Jogging is unavailable because ICON initialization failed.'
        )
        continue

      active_joint = 0
      print(
          f'\n--- Jogging Mode (Active Joint: {active_joint}) ---\n'
          'Controls:\n'
          '  [Left/Right Arrow] : Jog active joint by 0.04 rad\n'
          f'  [0 to {ndof-1}]           : Change active joint\n'
          '  [q]                : Exit jogging mode\n'
      )

      while True:
        try:
          sys.stdout.write(
              f"\r\x1b[K[Joint {active_joint}] Press Arrow to jog, 0-{ndof-1} to switch, 'q' to quit: "
          )
          sys.stdout.flush()

          key = get_key()
          if key.lower() == 'q' or key == '\x03':
            print('\nExiting jogging mode...\n')
            break

          # Check if key is a digit to switch active joint
          if key.isdigit() and 0 <= int(key) < ndof:
            active_joint = int(key)
            continue

          # Check if key is Left/Right arrow to jog
          if key in ('\x1b[C', '\x1b[D'):
            delta_value = 0.04 if key == '\x1b[C' else -0.04

            # Fetch current status to perform relative rotation
            status = icon_client.get_status()
            part_status = status.part_status[part_name]

            # Extract current positions
            current_positions = [
                j.position_sensed for j in part_status.joint_states
            ]

            if not current_positions or len(current_positions) != ndof:
              print('\nError: Could not retrieve current joint positions from the robot.')
              continue

            # Calculate goal positions
            goal_position = list(current_positions)
            goal_position[active_joint] += delta_value
            goal_velocity = [0.0] * ndof

            # Send move action
            action = session.add_action(
                create_action_utils.create_point_to_point_move_action(
                    action_id=action_id_counter,
                    joint_position_part_name=part_name,
                    goal_position=goal_position,
                    goal_velocity=goal_velocity,
                )
            )
            action_id_counter += 1

            session.start_action(action_id=action.id)
            sys.stdout.write(
                f'\nJogged joint {active_joint} by {delta_value:+.2f} rad. Goal: {goal_position[active_joint]:.3f}\n'
            )
            sys.stdout.flush()

        except icon_errors.Session.ActionError as e:
          print(f'\nAction error: {e}')
        except grpc.RpcError as e:
          print(f'\nICON RPC error during jogging: {e}')
          break
        except Exception as e:
          print(f'\nUnexpected error during jogging: {e}')
          break
    else:
      print("Invalid input. Please enter 'r', 'j', or 's'.")


def main(argv) -> None:
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')

  solution = deployments.connect(address=_ADDRESS.value)

  executive = solution.executive

  if executive.has_operation:
    op_state = executive.operation.metadata.operation_state
    if op_state == run_metadata_pb2.RunMetadata.PREPARING:
      print("Operation is in PREPARING state. Canceling...")
      executive.cancel()

  skills = solution.skills
  world = solution.world

  try:
    robot_ref = solution.resources[_ROBOT.value]
  except KeyError:
    raise ValueError(
        f"Robot '{_ROBOT.value}' not found in resources. Available resources:"
        f' {dir(solution.resources)}'
    )

  try:
    camera_ref = solution.resources[_CAMERA.value]
  except KeyError:
    raise ValueError(
        f"Camera '{_CAMERA.value}' not found in resources. Available resources:"
        f' {dir(solution.resources)}'
    )

  camera = None

  calibration_object_ref = world.get_object(_CALIBRATION_OBJECT.value)
  if not calibration_object_ref:
    raise ValueError(
        f"Calibration object '{_CALIBRATION_OBJECT.value}' not found in the"
        ' world model.'
    )

  streamer = None
  try:
    waypoints = []
    if _MANUAL_WAYPOINTS.value:
      if _STREAM_CAMERA.value:
        streamer = CameraStreamer(
            camera=camera,
            skills=skills,
            executive=executive,
            camera_ref=camera_ref,
            fps=_STREAM_FPS.value,
        )
        streamer.start()

      print('\n=== Manual Waypoint Collection ===')
      # Initialize ICON client
      icon_client = None
      part_name = None
      ndof = None
      session_context = None

      try:
        print(f'\nConnecting to ICON on localhost:{_ICON_PORT.value}...')
        icon_client = icon_api.Client.connect_with_params(
            connection.ConnectionParams(f'localhost:{_ICON_PORT.value}', 'icon')
        )

        parts = icon_client.list_parts()
        if not parts:
          raise ValueError('No parts found on the ICON server.')

        for part in parts:
          if part == _ROBOT.value:
            part_name = part
            break
        if part_name is None:
          part_name = parts[1] if len(parts) > 1 else parts[0]
          print(
              f"Warning: Could not find part matching '{_ROBOT.value}'."
              f" Using fallback part '{part_name}'."
          )

        part_configs = icon_client.get_config().part_configs
        for config in part_configs:
          if config.name == part_name:
            if config.HasField('generic_config'):
              ndof = config.generic_config.joint_position_config.num_joints
              print(f"Part '{part_name}' has {ndof} DoFs.")
            break
        if ndof is None:
          raise ValueError(
              f"Could not retrieve configuration for part '{part_name}'."
          )

        print(
            f"Connected! Controlling part '{part_name}' with {ndof} joints.\n"
        )
        session_context = icon_client.start_session([part_name])
      except Exception as e:
        print(
            f"Warning: Failed to initialize ICON client: {e}. Jogging ('j')"
            ' will be unavailable.'
        )

      if session_context:
        with session_context as session:
          run_manual_waypoint_loop(
              world,
              robot_ref,
              waypoints,
              session,
              ndof,
              part_name,
              icon_client,
              camera,
              streamer,
          )
      else:
        run_manual_waypoint_loop(
            world,
            robot_ref,
            waypoints,
            None,
            None,
            None,
            None,
            camera,
            streamer,
        )

      print(
          'Completed manual waypoint collection. Total waypoints:'
          f' {len(waypoints)}'
      )
      print('==================================\n')
    else:
      # Use sample_calibration_poses_skill to sample automatically
      if _MOVING_CAMERA.value:
        calibration_type = (
            calibration_type_pb2.CAMERA_TO_ROBOT_CALIBRATION_TYPE_MOVING_CAMERA
        )
      else:
        calibration_type = (
            calibration_type_pb2.CAMERA_TO_ROBOT_CALIBRATION_TYPE_STATIONARY_CAMERA
        )

      sample_calibration_poses_skill = (
          skills.ai.intrinsic.sample_calibration_poses
      )
      rbp = sample_calibration_poses_skill.intrinsic_proto.skills.RandomizedBoxParams(
          num_samples=_NUM_SAMPLES.value,
          sample_box_halfsize=skills_pb2.VectorNdValue(
              value=[
                  _SAMPLE_BOX_HALFSIZE_X.value,
                  _SAMPLE_BOX_HALFSIZE_Y.value,
                  _SAMPLE_BOX_HALFSIZE_Z.value,
              ]
          ),
          rotation_randomization_angle_degrees=float(_RAND_ANGLE.value),
          rotation_randomization_roll_angle_degrees=float(_RAND_ROLL_ANGLE.value),
      )

      sample_calibration_poses = sample_calibration_poses_skill(
          calibration_type=calibration_type,
          calibration_object=calibration_object_ref,
          camera=camera_ref,
          robot=robot_ref,
          randomized_box_params=rbp,
      )

      print('Sampling calibration poses via executive...')
      try:
        executive.run(sample_calibration_poses)
        res_proto = executive.get_value(sample_calibration_poses.result)
        waypoints = list(res_proto.sample_calibration_poses_result)
        print(f'Successfully sampled {len(waypoints)} calibration poses.')
      except Exception as e:
        print(f'Failed to sample calibration poses: {e}')
        try:
          errors = executive.get_errors()
          if errors and errors.errors:
            print(f'{errors.summary}')
        except Exception as err_e:
          print(f'Could not fetch executive error details: {err_e}')
        return

    if waypoints:
      if _EXPORT_WAYPOINTS_FILE.value:
        export_path = _EXPORT_WAYPOINTS_FILE.value
      else:
        export_choice = read_input(
            '\nDo you want to export the waypoints to a file? [y/n]: ',
            ['y', 'n'],
        )
        if export_choice == 'y':
          default_filename = f'waypoints_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.pbtxt'
          export_path = input(
              f'Enter file path to export [default: {default_filename}]: '
          ).strip()
          if not export_path:
            export_path = default_filename
        else:
          export_path = None

      if export_path:
        try:
          workspace_dir = os.environ.get('BUILD_WORKSPACE_DIRECTORY')
          if workspace_dir and not os.path.isabs(export_path):
            export_path = os.path.normpath(
                os.path.join(workspace_dir, export_path)
            )

          result_proto = (
              sample_calibration_poses_pb2.SampleCalibrationPosesResult()
          )
          for wp in waypoints:
            result_proto.sample_calibration_poses_result.add().ParseFromString(
                wp.SerializeToString()
            )
          with open(export_path, 'w') as f:
            f.write(text_format.MessageToString(result_proto))
          print(f'Successfully exported waypoints to {export_path}')
        except Exception as e:
          print(f'Failed to export waypoints: {e}')
  finally:
    if streamer is not None:
      streamer.stop()


if __name__ == '__main__':
  app.run(main)
