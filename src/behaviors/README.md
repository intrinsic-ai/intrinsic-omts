# Behavior Trees & Motion Subtrees (`src/behaviors/`)

Composable SBL Behavior Tree builders and motion primitives orchestrating the
end-to-end machine tending cycle.

## Modules

| Module | Entrypoint Function | Role in Machine Tending Cycle |
| :--- | :--- | :--- |
| [`machine_tending_bt.py`](machine_tending_bt.py) | `build_machine_tending_behavior_tree` | Assembles all 5 subtrees into a single `bt.Sequence` and wraps it in `bt.Loop(max_times=...)` for multi-cycle (`> 1`) or continuous (`<= 0`) execution. |
| [`motions.py`](motions.py) | `create_move_to_frame_task`, `create_compliant_touchdown_task`, `create_relative_retract_task` | Reusable building blocks for Cartesian frame alignment, force-controlled contact (`move_to_contact`), and linear `-Z` tool retracts (`RelativePoseEquality`). |
| [`pick.py`](pick.py) | `build_pick_from_infeed_subtree` | **Subtree 1**: Optional CNC door/vise prep, view pose alignment, 3D vision pose estimation & dynamic grasp frame update, compliant touchdown, 3 cm `-Z` retract, grasp, world attach, and linear lift. |
| [`load_machine.py`](load_machine.py) | `build_load_machine_subtree` | **Subtree 2**: Door/vise open verification, blended/direct approach to CNC entry & vise, compliant seating into jaws, vise clamp, release, world detach, and linear exit. |
| [`machining.py`](machining.py) | `build_machining_handshake_subtree` | **Subtree 3**: Robot standby outside enclosure, CNC door close, 0.5 s cycle-start DIO pulse, and cycle-complete wait/timeout. |
| [`unload_machine.py`](unload_machine.py) | `build_unload_machine_subtree` | **Subtree 4**: Door/vise open, machine & vise approach, compliant touchdown to machined part, 3 cm `-Z` retract, grasp, world attach, 3 cm `-Z` lift clear of vise jaws, and linear extraction. |
| [`return_infeed.py`](return_infeed.py) | `build_return_to_infeed_subtree` | **Subtree 5**: Blended/direct approach to infeed placement frame, compliant table touchdown, release, world detach, linear retract, and return to `view` frame. |

## Key Orchestration Rules

* **Single Executive Submission**: All subtrees are composed into one
  `bt.BehaviorTree` executed via a single `solution.executive.run(tree)` call.
* **Optional CNC Hardware (`machine=None`)**: When `config.machine` is `None`
  (e.g., `lab_bb_01`), door, vise, and cycle handshake nodes are automatically
  omitted without branching at runtime.
* **Segment-Scoped Collision Rules**: Contact and close-proximity segments attach
  targeted `CollisionRule` exclusions (e.g., excluding `[gripper, workpiece]`
  during `-Z` linear retracts or `[gripper, vise]` during vise approach) rather
  than disabling collision checking globally.
