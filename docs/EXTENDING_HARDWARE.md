# Extending OMTS Hardware & Real I/O Integration

This guide provides technical specifications and implementation details for interfacing physical hardware in OMTS:
1. **Robotic Grippers** (integrating `RobotiqGripper` via `gripper_cmd_skill` or `DioGripper` via digital I/O).
2. **CNC Machine & Pneumatic Vise Controls** (`DioCncMachine`, compliant seating, and the 3 cm pre-grasp linear retract).
3. **Perception & 3D Vision Tracking** (`estimate_pose_multi_view`, dynamic frame calculation via `bt.PythonScript`, short-side grasp alignment, and geodesic orientation optimization).
4. **Execution & CLI Configuration** (flags in [`src/main.py`](../src/main.py)).

---

## 1. Gripper Integration

OMTS defines an abstract [`GripperInterface`](../src/hardware/gripper.py):

```python
class GripperInterface(abc.ABC):
  @abc.abstractmethod
  def build_open_task(self, name: Optional[str] = None) -> bt.Node:
    raise NotImplementedError

  @abc.abstractmethod
  def build_close_task(self, name: Optional[str] = None) -> bt.Node:
    raise NotImplementedError
```

### Option A: Robotiq Smart Grippers (Hand-E / 2F)

Physical Robotiq grippers in IOC communicate via RS-485 Modbus RTU or UR tool communication and are controlled using the `ai.intrinsic.gripper_cmd_skill` SBL skill.

#### 1. Implementation in [`src/hardware/gripper.py`](../src/hardware/gripper.py)
[`RobotiqGripper`](../src/hardware/gripper.py) dispatches `gripper_cmd_skill` with standard ROS `JointState` messages specifying metric finger displacement:

```python
class RobotiqGripper(GripperInterface):
  """Robotiq adaptive gripper controlled via gripper_cmd_skill."""

  def __init__(
      self,
      solution: Any,
      joint_name: str = "robotiq_hande_left_finger_joint",
      open_position: float = 0.025,
      close_position: float = 0.0,
      action_name: Optional[str] = None,
  ) -> None:
    self._solution = solution
    self._joint_name = joint_name
    self._open_position = open_position
    self._close_position = close_position
    self._action_name = action_name
    self._gripper_cmd_skill = solution.skills.ai.intrinsic.gripper_cmd_skill

  def build_open_task(self, name: Optional[str] = None) -> bt.Node:
    command = self._gripper_cmd_skill.ai.intrinsic.JointState(
        name=[self._joint_name],
        position=[self._open_position],
    )
    kwargs = {"command": command}
    if self._action_name is not None:
      kwargs["action_name"] = self._action_name
    action = self._gripper_cmd_skill(**kwargs)
    return bt.Task(action=action, name=name or "Open Robotiq Gripper")

  def build_close_task(self, name: Optional[str] = None) -> bt.Node:
    command = self._gripper_cmd_skill.ai.intrinsic.JointState(
        name=[self._joint_name],
        position=[self._close_position],
    )
    kwargs = {"command": command}
    if self._action_name is not None:
      kwargs["action_name"] = self._action_name
    action = self._gripper_cmd_skill(**kwargs)
    return bt.Task(action=action, name=name or "Close Robotiq Gripper")
```

#### 2. Calibration & Position Parameters
* **Finger Joint:** Default joint name is `robotiq_hande_left_finger_joint`.
* **Fully Open:** `0.025` meters ($25\,\text{mm}$ per finger, yielding a total stroke of $50\,\text{mm}$).
* **Fully Closed:** `0.000` meters (fingers touching at centerline).
* **Action Namespace:** Default action endpoint is `/hande_gripper_controller/gripper_cmd` (or specified via `--gripper_action_name`).
* **Pre-Grasp Open Sequence:** Fingers must always be commanded open (`gripper.build_open_task`) **before** descending into `pre_grasp` to prevent colliding with stock or fixtures.

#### 3. Tool TCP Calibration Gotcha
When attaching the gripper to the UR flange in [`configs/ur_module.attachments.updates.pbtxt`](../configs/ur_module.attachments.updates.pbtxt):
* **Z-Rotation Offset:** Setting `gripper -> tool_frame` orientation to `x: 0, y: 0, z: 0.70710678, w: -0.70710678` ($270^\circ$ / $-90^\circ$ Z-rotation) aligns the moving TCP frame with the standard downward frame convention `orientation { x: 1, y: 0, z: 0, w: 0 }` ($0.00^\circ$ offset).

---

### Option B: Pneumatic or Relay Grippers via Digital I/O (`DioGripper`)

For pneumatic double-acting cylinders or open/close solenoid valves:
1. Map digital output pins into [`DioGripper`](../src/hardware/gripper.py):
   ```python
   gripper = DioGripper(
       solution=solution,
       open_pin=0,               # Digital output pin on controller/tool
       close_pin=1,              # Digital output pin
       device_name="ur_module",  # Device owning the I/O pins in solution
   )
   ```
2. `DioGripper.build_open_task` and `DioGripper.build_close_task` dispatch `skills.ai.intrinsic.dio_set_output`.

---

## 2. CNC Machine & Pneumatic Vise Integration

OMTS defines an abstract [`CncMachineInterface`](../src/hardware/machine.py):

```python
class CncMachineInterface(abc.ABC):
  @abc.abstractmethod
  def build_open_door_task(self, name: Optional[str] = None) -> bt.Node: ...
  @abc.abstractmethod
  def build_close_door_task(self, name: Optional[str] = None) -> bt.Node: ...
  @abc.abstractmethod
  def build_open_vise_task(self, name: Optional[str] = None) -> bt.Node: ...
  @abc.abstractmethod
  def build_close_vise_task(self, name: Optional[str] = None) -> bt.Node: ...
  @abc.abstractmethod
  def build_trigger_cycle_task(self, name: Optional[str] = None) -> bt.Node: ...
  @abc.abstractmethod
  def build_wait_cycle_complete_task(self, timeout_seconds: float = 30.0, name: Optional[str] = None) -> bt.Node: ...
```

### Digital I/O Wiring (`DioCncMachine`)

Real machine and vise signals are mapped in [`DioCncMachine`](../src/hardware/machine.py):

```python
machine = DioCncMachine(
    solution=solution,
    door_open_pin=2,          # Digital output -> Door open solenoid
    door_close_pin=3,         # Digital output -> Door close solenoid
    vise_open_pin=4,          # Digital output -> Pneumatic vise unclamp
    vise_close_pin=5,         # Digital output -> Pneumatic vise clamp
    cycle_start_pin=6,        # Digital output -> CNC cycle start pulse
    cycle_done_input_pin=0,   # Digital input <- M-code / cycle complete relay
    device_name="ur_module",
)
```

1. **Cycle Start Pulse:** Triggering CNC cycle start requires a pulse (high, then low), implemented as a sequence of `dio_set_output(pin=cycle_start_pin, state=True)` followed by `dio_set_output(pin=cycle_start_pin, state=False)`.
2. **Compliant Seating Before Clamping:**
   When loading raw stock into the vise jaws, never use rigid positioning. Always pair vise approach with [`create_compliant_touchdown_task`](../src/behaviors/motions.py) (`move_to_contact`) in $+Z$ tool direction (`contact_force_newtons=15.0`) to seat the stock flat against the parallels before commanding `close_vise`.

### 3 cm Linear Retract Before Gripper Closure

During both infeed pick and machine unload:
1. **Flange Contact:** The robot executes compliant touchdown towards the workpiece along $+Z_{\text{tool}}$. Mechanical contact is registered when the gripper body/flange touches the top face of the stock.
2. **Linear Retract:** Closing gripper fingers immediately at the touchdown depth causes the finger edges to pinch or collide with the top surface. Therefore, immediately following touchdown, the robot executes a $3\,\text{cm}$ ($0.03\,\text{m}$) relative `LINEAR` Cartesian motion along tool $-Z$:
   ```python
   create_relative_retract_task(
       robot=robot,
       relative_offset_z=-0.03,  # 3 cm linear retract along tool -Z
       name="Linear Retract 3cm",
   )
   ```
   This is implemented via `RelativePoseEquality(relative_pose=Pose(position=(0.0, 0.0, -0.03)))` relative to the moving `tool_frame`.
3. **Finger Closure:** Once elevated by $3\,\text{cm}$, the finger pads align squarely across the workpiece short sides. The gripper then commands `build_close_task`.

---

## 3. Perception & Dynamic World Frame Tracking

OMTS integrates 3D object pose estimation via IOC's `estimate_pose_multi_view` skill using FoundationPose.

### The 3-Step Perception Pipeline

In [`src/hardware/vision.py`](../src/hardware/vision.py), the `build_perception_and_spawn_task` method builds a sequential Behavior Tree pipeline:

1. **`capture_images`:** Captures synchronized RGB (sensor 1) and Depth (sensor 4) frames from the 3D camera.
2. **`estimate_pose_multi_view`:** Dispatches FoundationPose inference to `pose_estimator_service` using the registered estimator ID (`ai.intrinsic.raw_stock_2x3x5_estimator`).
3. **`bt.PythonScript` (Dynamic Frame Calculator):** Extracts the camera-relative estimate, resolves live robot kinematics, computes optimal grasp orientation, and updates target frames in SBL `ObjectWorld`.

### Clean Script Injection (`src/utils/script_utils.py`)

To maintain clean code structure without inline string scripts in behavior builders:
* Logic is maintained as a standard typed module in [`src/utils/dynamic_frame_calculator.py`](../src/utils/dynamic_frame_calculator.py).
* Injected into `bt.PythonScript` via [`src/utils/script_utils.py:load_python_script`](../src/utils/script_utils.py):
  ```python
  from src.utils.dynamic_frame_calculator import calculate_and_update_dynamic_frames
  from src.utils.script_utils import load_python_script

  update_frames_task = bt.Task(
      action=bt.PythonScript(
          function_body=load_python_script(calculate_and_update_dynamic_frames)
      ),
      name="Calculate and Update Dynamic Grasp Frames",
  )
  ```

### Resolving Dynamic Camera Extrinsics

`estimate_pose_multi_view` returns `root_t_target` expressed in the camera optical sensor coordinate frame ($T_{\text{camera} \to \text{target}}$).

The script dynamically queries the live camera-to-root transform from SBL `ObjectWorld`:
```python
root_t_camera = world.get_transform(parent_obj, camera_sensor_node)
root_t_target = root_t_camera * cam_pose
```
$$\mathbf{T}_{\text{root} \to \text{target}} = \mathbf{T}_{\text{root} \to \text{camera}} \cdot \mathbf{T}_{\text{camera} \to \text{target}}$$

> [!CRITICAL]
> **Camera Parentage Requirement:**
> On solution startup, `orbbec_camera` defaults to a child of `root` at $[0, 0, 0]$. You **must** apply [`configs/lab_bb_01_orbbec_gemini.updates.pbtxt`](../configs/lab_bb_01_orbbec_gemini.updates.pbtxt) via `bazel run //tools/world:apply_scene_updates`.
> Without this, `world.get_transform(root, camera.sensor)` returns identity, placing target frames at raw optical coordinates behind the robot base column and causing `move_robot:10301` IK failures.

### Short-Side Grasp Alignment

The detected workpiece coordinate frame from FoundationPose has its top-face normal along $+X_{\text{part}}$, with rectangular edges along $+Y_{\text{part}}$ and $+Z_{\text{part}}$.

To grasp the workpiece along its **short sides** (placing finger pads on the short ends with the closing stroke traversing the length):
1. Project part axes $+Y_{\text{part}}$ and $+Z_{\text{part}}$ into the world $XY$ plane:
   ```python
   vy = target_rot.rotate_point([0.0, 1.0, 0.0])
   vz = target_rot.rotate_point([0.0, 0.0, 1.0])
   ```
2. Determine the longest horizontal dimension:
   * If length along $Y >$ length along $Z$: $\mathbf{v}_{\text{longest}} = (v_{y,0}, v_{y,1})$
   * Otherwise: $\mathbf{v}_{\text{longest}} = (v_{z,0}, v_{z,1})$
3. Compute continuous yaw angle:
   $$\theta_{\text{longest}} = \text{atan2}(v_{\text{longest}, y}, v_{\text{longest}, x})$$
4. Set tool approach yaw $\psi = \theta_{\text{longest}}$ ($90^\circ$ relative to the long edge). The downward-pointing tool base orientation is:
   $$\mathbf{q}_{\text{base}} = [1.0, 0.0, 0.0, 0.0]$$
   $$\mathbf{q}_{\text{yaw}} = \left[0.0, 0.0, \sin\left(\frac{\psi}{2}\right), \cos\left(\frac{\psi}{2}\right)\right]$$
   $$\mathbf{q}_{\text{candidate}} = \mathbf{q}_{\text{yaw}} \cdot \mathbf{q}_{\text{base}}$$

### Geodesic Orientation Optimization

Parallel-jaw grippers have $180^\circ$ rotational symmetry around the tool $Z$ axis, and quaternions have double-cover ambiguity ($\mathbf{q} \equiv -\mathbf{q}$). Arbitrarily selecting an orientation candidate often requires large joint 6 rotations, leading to joint limit violations or protective stops.

[`dynamic_frame_calculator.py`](../src/utils/dynamic_frame_calculator.py) evaluates all 4 symmetric candidates:
1. $\mathbf{q}_0 = \mathbf{q}_{\text{candidate}}$
2. $\mathbf{q}_1 = -\mathbf{q}_{\text{candidate}}$
3. $\mathbf{q}_2 = \mathbf{q}_{\text{candidate}} \cdot \mathbf{R}_z(180^\circ)$
4. $\mathbf{q}_3 = -\mathbf{q}_{\text{candidate}} \cdot \mathbf{R}_z(180^\circ)$

The candidate maximizing the quaternion dot product with the current tool orientation is selected:
$$\mathbf{q}^* = \arg\max_i (\mathbf{q}_i \cdot \mathbf{q}_{\text{current}})$$
This guarantees the minimum geodesic angular distance in $\text{SO}(3)$ and prevents wrist joint wrapping.

### Updating Scene Frames on `root`

Target frames are updated directly on `root`:
* `root/grasp`: Centered on the top surface of the workpiece.
* `root/pre_grasp`: Offset vertically by standoff distance ($10\,\text{cm}$ along tool $-Z$).

```python
world.batch_update([
    create_or_update_frame_request("root", "pre_grasp", pregrasp_pos, best_quat),
    create_or_update_frame_request("root", "grasp", grasp_pos, best_quat),
])
```
This pattern eliminates `create_object:3` name collisions and `move_robot:10601` frame lookup failures.

---

## 4. Enabling Real Hardware in `main.py`

In [`src/main.py`](../src/main.py), configure flags to switch between simulation/mock and physical hardware:

```bash
# Execute with physical Robotiq Hand-E gripper and Orbbec camera:
bazel run //src:omts_app -- \
  --address=localhost:17080 \
  --infeed_mode=perception \
  --gripper_type=robotiq \
  --gripper_joint_name=robotiq_hande_left_finger_joint \
  --gripper_open_position=0.025 \
  --gripper_close_position=0.0

# Execute with Digital I/O pneumatic gripper:
bazel run //src:omts_app -- \
  --address=localhost:17080 \
  --infeed_mode=perception \
  --gripper_type=dio \
  --gripper_dio_open_pin=0 \
  --gripper_dio_close_pin=1

# Execute with deterministic grid pallet infeed:
bazel run //src:omts_app -- \
  --address=localhost:17080 \
  --infeed_mode=grid \
  --gripper_type=robotiq
```

### Pre-Flight Verification Checklist
Before running on physical hardware:
1. **Deploy Solution:** Ensure `//:omts_solution` is running on target cluster/port (`localhost:17080`).
2. **Apply Scene Updates:** Run `bazel run //tools/world:apply_scene_updates` to configure camera parentage and default frames.
3. **Inspect World:** Run `bazel run //tools/world:inspect_world` to verify `flange`, `tool_frame`, and `camera.sensor` frames are present.
4. **Move to View:** Run `bazel run //tools/jogging:move_to_frame -- --frame=view` to position the arm for clear perception before starting the application.
