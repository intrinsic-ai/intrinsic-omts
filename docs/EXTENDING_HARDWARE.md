# Extending OMTS Hardware & Real I/O Integration

This guide provides step-by-step instructions for transitioning OMTS from mock hardware to physical devices, specifically:
1. **Robotic Grippers** (replacing `MockGripper` with `DioGripper` or `RobotiqGripper`).
2. **CNC Machine & Pneumatic Vise Controls** (replacing `MockCncMachine` with `DioCncMachine`).
3. **Perception & 3D Vision Tracking** (best practices for FoundationPose, dynamic frame updates, and orientation alignment).

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

### Option A: Pneumatic or Relay Grippers via Digital I/O (`DioGripper`)
If using pneumatic double-acting cylinders or simple open/close relay valves:
1. Use [`DioGripper`](../src/hardware/gripper.py):
   ```python
   gripper = DioGripper(
       solution=solution,
       open_pin=0,       # Digital Output pin index on controller or tool
       close_pin=1,      # Digital Output pin index
       device_name="ur_module",  # Device owning the I/O pins in solution
   )
   ```
2. In `DioGripper`, `build_open_task` and `build_close_task` dispatch `skills.ai.intrinsic.dio_set_output`.

### Option B: Robotiq Smart Grippers (Hand-e / 2F-85)
For smart electric grippers communicating via RS-485 Modbus RTU or UR tool communication:
1. **Catalog Asset in `BUILD`:** Ensure the Robotiq resource asset is included in `omts_solution`:
   ```starlark
   assets = [
       "@ioc//google3/intrinsic/resources/catalog/resourcedata/gripper:robotiq_pinch_gripper_resource_type",
       ...
   ]
   ```
2. **Implement `RobotiqGripper` in [`src/hardware/gripper.py`](../src/hardware/gripper.py):**
   ```python
   class RobotiqGripper(GripperInterface):
     def __init__(
         self,
         solution: Any,
         gripper_resource_name: str = "gripper",
         stroke_mm: float = 50.0,
         force_percentage: float = 50.0,
     ) -> None:
       self._solution = solution
       self._gripper_resource = solution.resources[gripper_resource_name]
       self._grip_skill = solution.skills.ai.intrinsic.robotiq_hande_grip

     def build_open_task(self, name: Optional[str] = None) -> bt.Node:
       action = self._grip_skill(
           gripper=self._gripper_resource,
           position_mm=self._stroke_mm,
           force_percentage=self._force_percentage,
       )
       return bt.Task(action=action, name=name or "Open Robotiq Gripper")

     def build_close_task(self, name: Optional[str] = None) -> bt.Node:
       action = self._grip_skill(
           gripper=self._gripper_resource,
           position_mm=0.0,
           force_percentage=self._force_percentage,
       )
       return bt.Task(action=action, name=name or "Close Robotiq Gripper")
   ```

### Gripper TCP Calibration Gotcha
When attaching the gripper to the UR flange in [`configs/ur_module.attachments.updates.pbtxt`](../configs/ur_module.attachments.updates.pbtxt):
* **Z-Rotation Offset:** Setting `gripper -> tool_frame` orientation to `x: 0, y: 0, z: 0.70710678, w: -0.70710678` (270° / -90° Z-rotation) aligns the moving TCP frame with the standard downward frame convention `orientation { x: 1, y: 0, z: 0, w: 0 }` (0.00° offset).

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
To wire real machine inputs and outputs:
1. Map the electrical pins from the CNC controller or PLC into [`DioCncMachine`](../src/hardware/machine.py):
   ```python
   machine = DioCncMachine(
       solution=solution,
       door_open_pin=2,          # Digital output -> Door open solenoid
       door_close_pin=3,         # Digital output -> Door close solenoid
       vise_open_pin=4,          # Digital output -> Pneumatic vise unclamp
       vise_close_pin=5,         # Digital output -> Pneumatic vise clamp
       cycle_start_pin=6,        # Digital output -> CNC cycle start push
       cycle_done_input_pin=0,   # Digital input <- M-code / cycle complete relay
       device_name="ur_module",
   )
   ```
2. **Cycle Start Pulse:** Triggering CNC cycle start requires a pulse (e.g. 500ms high, then low), which is implemented as a sequence of `dio_set_output(pin=cycle_start_pin, state=True)` followed by `dio_set_output(pin=cycle_start_pin, state=False)`.
3. **Compliant Seating Before Clamping:**
   When loading raw stock into the vise jaws, never use rigid positioning. Always pair vise approach with [`create_compliant_touchdown_task`](../src/behaviors/motions.py) (`move_to_contact`) in $+Z$ tool direction to seat the stock flat against the parallels before commanding `close_vise`.

---

## 3. Perception & Dynamic World Frame Tracking

OMTS uses FoundationPose via IOC's `estimate_pose_multi_view` skill. When integrating 3D camera pose estimation:

### Why Target Scene Frames on `root` Rather Than Spawned Objects
1. **Frame Resolution:** Objects created dynamically by `create_object` do not expose child transform frames (like link names) in the SBL transform tree. Calling `move_robot` with `TransformNodeReferenceByName(object=...)` causes `move_robot:10601` frame lookup failures.
2. **Name Collisions:** Spawning objects with static names (`object_names=["detected_workpiece"]`) causes `create_object:3` name collisions on subsequent cycles.
3. **Correct Pattern:** Pre-define `root/pre_grasp` and `root/grasp` in [`configs/scene.updates.pbtxt`](../configs/scene.updates.pbtxt) and dynamically update their transforms via `update_world`.

### Indirect Transform Updates (Zero Hardcoded Extrinsics)
In [`src/hardware/vision.py`](../src/hardware/vision.py), configure `update_world` with indirect transform requests:
* `node_a`: `orbbec_camera`
* `node_b`: `root/pre_grasp` (or `root/grasp`)
* `node_to_update`: `root/pre_grasp` (or `root/grasp`)

SBL's world service automatically computes the world transform using the live robot kinematic tree:
$$T_{\text{root}\to\text{pre\_grasp}} = T_{\text{root}\to\text{orbbec\_camera}} \cdot T_{\text{orbbec\_camera}\to\text{pre\_grasp}}$$
No camera calibration matrices (`t_cam`, `r_mat`) are hardcoded in application code.

### Tool Grasp Orientation Alignment in CEL
The detected workpiece coordinate frame from FoundationPose has its top-face normal along $+X_{\text{part}}$, with edges along $+Y_{\text{part}}$ and $+Z_{\text{part}}$.

To point the tool approach axis ($+Z_{\text{tool}}$) vertically downward into the table normal ($-Z_{\text{root}}$) while aligning the parallel jaws ($+X_{\text{tool}}$) with the workpiece horizontal edges:
$$\mathbf{q}_{\text{tool}} = \mathbf{q}_{\text{part}} \cdot [0.5, 0.5, 0.5, 0.5]$$

This product is expressed linearly in CEL:
```python
tool_orientation = uw_proto.Quaternion(
    x=cel.CelExpression(f"0.5 * ({qw} + {qx} + {qy} - {qz})"),
    y=cel.CelExpression(f"0.5 * ({qw} - {qx} + {qy} + {qz})"),
    z=cel.CelExpression(f"0.5 * ({qw} + {qx} - {qy} + {qz})"),
    w=cel.CelExpression(f"0.5 * ({qw} - {qx} - {qy} - {qz})"),
)
```

### Pre-Grasp Standoff
Standoff distance is applied along the optical depth axis ($Z$ in camera coordinates):
```python
z = cel.CelExpression(f"{detected_pos.z} - {approach_offset_z}")
```
For top-down camera views, setting `approach_offset_z = 0.10` creates a $100\,\text{mm}$ clearance directly above the raw stock.

---

## 4. Enabling Real Hardware in `main.py`

In [`src/main.py`](../src/main.py), configure CLI flags to toggle between mocks and real hardware:

```python
_GRIPPER_TYPE = flags.DEFINE_enum(
    "gripper_type", "mock", ["mock", "dio", "robotiq"], "Gripper backend type."
)
_MACHINE_TYPE = flags.DEFINE_enum(
    "machine_type", "mock", ["mock", "dio"], "CNC machine & vise backend type."
)
```

Then instantiate the selected adapter:
```python
if _GRIPPER_TYPE.value == "dio":
  gripper = DioGripper(solution=solution, open_pin=0, close_pin=1)
elif _GRIPPER_TYPE.value == "robotiq":
  gripper = RobotiqGripper(solution=solution)
else:
  gripper = MockGripper()

if _MACHINE_TYPE.value == "dio":
  machine = DioCncMachine(solution=solution)
else:
  machine = MockCncMachine()
```
