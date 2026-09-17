# Third-Party Integrations

Optional integrations with software that lives outside OMTS. Everything in this
directory is **opt-in**: OMTS builds, deploys and runs to completion without any
of it, and nothing under [`src/`](../src/) or [`tools/`](../tools/) may depend on
it.

| Integration | Upstream | What it adds |
| :--- | :--- | :--- |
| [`intrinsic_moveit/`](./intrinsic_moveit/) | [`intrinsic-ai/intrinsic-moveit`](https://github.com/intrinsic-ai/intrinsic-moveit) | Model-based grasp planning via MoveIt Task Constructor, plus a verification CLI. |

## Ground rules

> [!IMPORTANT]
> The dependency edge points **one way only**: `third_party/` may depend on
> `//src/...`, never the reverse. Each integration's `BUILD` sets
> `default_visibility` to its own `__subpackages__` rather than
> `//visibility:public`, so Bazel rejects a first-party dependency at analysis
> time instead of leaving it to review to catch.

* **Prerequisites are the user's job.** OMTS does not deploy, bundle or
  configure an integration's services and skills. Each integration's `README.md`
  states what must already be running before its tools do anything, and its CLIs
  repeat that in `--help`.
* **Names are explicit.** Modules, Bazel targets and public symbols carry the
  integration's name — `moveit_plan_grasp_and_move`, `MoveItGraspPlanner` — so they stay
  distinguishable from the first-party equivalents that may later occupy the
  generic names.
* **Tests live with the code.** `bazel test //tests/unit:all` covers first-party
  OMTS only. Each integration ships its own test package.
