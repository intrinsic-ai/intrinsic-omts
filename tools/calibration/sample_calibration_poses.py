"""Script to sample calibration poses and export them to a file."""

import datetime
import os
import sys
import termios
import tty

from absl import app
from absl import flags
from google.protobuf import text_format
import grpc

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
from intrinsic.solutions import perception
from intrinsic.util.grpc import connection

# Command line input flags
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
    0.20,
    'Sampling box half-size X dimension (meters).',
)
_SAMPLE_BOX_HALFSIZE_Y = flags.DEFINE_float(
    'sample_box_halfsize_y',
    0.20,
    'Sampling box half-size Y dimension (meters).',
)
_SAMPLE_BOX_HALFSIZE_Z = flags.DEFINE_float(
    'sample_box_halfsize_z',
    0.20,
    'Sampling box half-size Z dimension (meters).',
)
_RAND_ANGLE = flags.DEFINE_float(
    'rand_angle', 10.0, 'Randomization angle in degrees.'
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
    camera: perception.Camera | None = None,
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

      if camera and _CAPTURE_IMAGES.value:
        try:
          print('Capturing images...')
          capture_result = camera.capture()
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
            print(f'Saved captured image: {filepath}')
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

  solution = deployments.connect(address='localhost:17080')

  executive = solution.executive
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
  try:
    cameras = perception.Cameras.for_solution(solution)
    camera = cameras[_CAMERA.value]
  except Exception as e:
    print(
        f'Warning: Failed to initialize camera client: {e}. Image capture on'
        " 'r' will be unavailable."
    )

  calibration_object_ref = world.get_object(_CALIBRATION_OBJECT.value)
  if not calibration_object_ref:
    raise ValueError(
        f"Calibration object '{_CALIBRATION_OBJECT.value}' not found in the"
        ' world model.'
    )

  waypoints = []
  if _MANUAL_WAYPOINTS.value:
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

    sample_calibration_poses_skill = skills.ai.intrinsic.sample_calibration_poses
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
        result_proto.sample_calibration_poses_result.extend(waypoints)
        with open(export_path, 'w') as f:
          f.write(text_format.MessageToString(result_proto))
        print(f'Successfully exported waypoints to {export_path}')
      except Exception as e:
        print(f'Failed to export waypoints: {e}')


if __name__ == '__main__':
  app.run(main)
