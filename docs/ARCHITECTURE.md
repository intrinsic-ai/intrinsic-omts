# OMTS Architecture & System Design

The Open Machine Tending Solution (OMTS) is an open-source reference application for CNC machine tending, press braking, and fixture loading built on **Intrinsic Open Core (IOC)** using the **Solution Building Library (SBL)** Python SDK.

---

## 1. System Overview & Lifecycle

OMTS separates deployment packaging from application logic:

* **Solution Package (`//:omts_solution`):** Declares hardware devices (Universal Robots arm, Robotiq gripper, Orbbec camera), resources, skills, and configuration files deployed as a microservice cluster or local container.
* **Application Binary (`//src:omts_app`):** Connects to the solution deployment via gRPC (`deployments.connect(address=...)`), instantiates hardware adapters, configures infeed strategies, builds a composable Behavior Tree (BT), and executes the machine tending cycle.

```mermaid
flowchart TD
    subgraph SolutionCluster["SBL Solution Deployment (:omts_solution)"]
        W[World Model & Kinematics Tree]
        R[Robot Service / ICON]
        P[Perception / Pose Estimator]
        IO[ADIO / Digital I/O]
        G[Gripper Service]
    end

    subgraph OMTSApplication["OMTS Application (//src:omts_app)"]
        MAIN[main.py] --> ADAPTERS[Hardware Adapters]
        ADAPTERS --> ROBOT_ADAPT[UrRobot]
        ADAPTERS --> VISION_ADAPT[OrbbecVision]
        ADAPTERS --> GRIP_ADAPT[DioGripper / MockGripper]
        ADAPTERS --> CNC_ADAPT[DioCncMachine / MockCncMachine]
        
        MAIN --> STRATEGY[Infeed Strategy]
        STRATEGY -.-> PERCEP[PerceptionInfeedStrategy]
        STRATEGY -.-> GRID[GridInfeedStrategy]
        
        MAIN --> BT_BUILDER[Behavior Tree Builder]
        BT_BUILDER --> SUB_PICK[Pick Subtree]
        BT_BUILDER --> SUB_LOAD[Machine Load Subtree]
        BT_BUILDER --> SUB_MACHINE[Machining Subtree]
        BT_BUILDER --> SUB_UNLOAD[Unload Subtree]
        BT_BUILDER --> SUB_OUTFEED[Outfeed Return Subtree]
    end

    OMTSApplication -- gRPC (port 17080) --> SolutionCluster
```

---

## 2. Hardware Abstraction Layer (HAL)

All hardware interactions are mediated by abstract interfaces in [`src/hardware/`](../src/hardware/):

| Interface | Implementations | Key Responsibilities |
| :--- | :--- | :--- |
| [`RobotInterface`](../src/hardware/robot.py) | `UrRobot`, `MockRobot` | Joint motions (`build_move_joint_task`), Cartesian motions (`build_move_cartesian_task`), and compliant contact (`build_move_to_contact_task`). |
| [`GripperInterface`](../src/hardware/gripper.py) | `DioGripper`, `MockGripper` | Gripper open/close tasks and grasp confirmation. |
| [`CncMachineInterface`](../src/hardware/machine.py) | `DioCncMachine`, `MockCncMachine` | Door actuation, pneumatic vise clamping, cycle start pulsing, and cycle complete waiting. |
| [`VisionInterface`](../src/hardware/vision.py) | `OrbbecVision`, `MockVision` | RGB-D image acquisition, 6D pose estimation, and dynamic world frame updates. |

This abstraction ensures that high-level process behavior trees remain decoupled from the underlying hardware interfaces, facilitating offline unit testing without real hardware or simulators.

---

## 3. Infeed Strategies (Strategy Pattern)

Infeed handling is decoupled into interchangeable strategy classes in [`src/core/infeed.py`](../src/core/infeed.py):

### A. Vision-Guided Infeed (`PerceptionInfeedStrategy`)
* Used for parts placed randomly on the infeed table or tray.
* Moves robot to a calibrated `view` frame.
* Triggers Orbbec RGB-D capture and FoundationPose multi-view inference.
* Dynamically updates `root/pre_grasp` and `root/grasp` via indirect transform updates in `update_world`.
* Moves to dynamic pre-grasp, executes compliant touchdown (`move_to_contact`), grasps the part, and retracts linearly.

### B. Blind Grid Pallet Infeed (`GridInfeedStrategy`)
* Used for structured part pallets, blister packs, or fixtures.
* Computes deterministic slot coordinates:
  $$\mathbf{p}_{\text{slot}}(i) = \mathbf{p}_{\text{origin}} + (\text{row} \cdot \Delta_y) + (\text{col} \cdot \Delta_x)$$
* Iterates sequentially across available slots without requiring perception.

---

## 4. Master Machine Tending Cycle

The master Behavior Tree assembled in [`src/behaviors/machine_tending_bt.py`](../src/behaviors/machine_tending_bt.py) executes the following 13-step sequence:

```mermaid
sequenceDiagram
    autonumber
    participant Robot as UR Robot
    participant Vision as Orbbec Camera
    participant CNC as CNC Machine & Vise
    participant World as World Model

    Note over Robot,World: Phase 1: Infeed Acquisition
    Robot->>Robot: Move to view frame
    Vision->>Vision: Capture RGB-D & run FoundationPose
    Vision->>World: update_world (root/pre_grasp, root/grasp)
    Robot->>Robot: Move to root/pre_grasp (0.10m standoff)
    Robot->>Robot: Compliant Touchdown (+Z tool contact)
    Robot->>Robot: Close Gripper (Grasp Part)
    Robot->>Robot: Linear Retract to root/pre_grasp

    Note over Robot,World: Phase 2: Machine Loading
    CNC->>CNC: Open Door & Open Vise (DIO)
    Robot->>Robot: Move to machine_approach entry frame
    Robot->>Robot: Move to pre_place_vise insertion frame
    Robot->>Robot: Compliant Seating into Vise (+Z tool contact)
    CNC->>CNC: Clamp Vise (DIO)
    Robot->>Robot: Open Gripper
    Robot->>Robot: Linear Retract to pre_place_vise
    Robot->>Robot: Retract to machine_approach

    Note over Robot,World: Phase 3: Machining & Extraction
    CNC->>CNC: Close Door & Pulse Cycle Start (DIO)
    CNC->>CNC: Wait for Machining Cycle Complete
    CNC->>CNC: Open Door & Open Vise (DIO)
    Robot->>Robot: Approach Vise & Grasp Finished Part
    Robot->>Robot: Retract to Outfeed / Return
```
