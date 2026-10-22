# AGENTS.md: Agent Guidelines & Architecture for OMTS (Intrinsic Core)

## 1. Executive Summary & Core Mental Model

The Open Machine Tending Solution (OMTS) is an open-source reference application built on [Intrinsic Core](https://github.com/intrinsic-ai/intrinsic-core), a hardware-agnostic robotics platform providing deterministic real-time control, collision-free motion planning, 6-DOF perception, automated camera calibration, and local physics simulation, coupled natively to ROS 2.

Robotic applications in Intrinsic Core are structured around four fundamental primitives:

### The 4 Core Primitives
1. **Assets (`intrinsic_asset_instance`)**: Self-contained, deployable packages representing hardware devices (manipulators, grippers, cameras), persistent compute services (neural network inference, pose estimators), simulators (Gazebo), or physical data/CAD meshes (SceneObjects).
2. **Skills (`skill_interface.Skill`)**: Discrete, parameterized, strongly-typed units of executable behavior invoked by the Behavior Tree Executive (e.g., `move_robot`, `capture_images`, `calibrate_camera_to_robot`, `adder`).
3. **World / Scene**: The spatial knowledge engine maintaining coordinate transforms (`/tf`), kinematic trees, CAD geometry, and collision environments. Modified dynamically via `ObjectWorldUpdates`.
4. **Executive**: The orchestration engine executing Behavior Trees (BT) to govern sequencing, concurrency, fallback handling, blackboard data transfer, and error recovery.

### The Runtime Architecture
* **Local Cluster**: Intrinsic Core executes on a local single-node Kubernetes (`k3s`) cluster running directly on the workstation or Industrial PC (IPC).
* **Cluster Gateway**: All CLI tools (`inctl`), Python Solution Building Language (SBL) scripts, and client libraries connect to the cluster gateway at `localhost:17080`.
* **Zenoh Middleware Router**: High-throughput DDS/ROS 2 bridge and telemetry exchange operates on port `7447`.
* **Execution Modes**: Solutions run in either `sim` (Gazebo physics & simulated sensors) or `real` (physical hardware modules & real-time `PREEMPT_RT` kernel).

---

## 2. Core Agent Principles & Architecture Rules

When authoring, refactoring, or running Intrinsic Core solutions, adhere strictly to these engineering tenets:

### Rule 1: Skills Are Strictly Stateless
> [!IMPORTANT]
> The Intrinsic Skill runtime creates a **new instance of your Skill class for every single interaction** (`get_footprint()`, `preview()`, and `execute()`).
* **Never** load heavy neural network weights, open persistent network sockets, or store mutable state inside a Skill's `__init__()`. Doing so causes the runtime to re-read files from disk and reconstruct inference sessions on every invocation, causing severe latency spikes and memory churn.
* **Always** offload state, continuous background processes, and large ML inference models to a **persistent Service Asset** (such as a containerized gRPC microservice). Author your custom Skill as a thin, lightweight client wrapper whose sole responsibility is to forward inputs and receive predictions from that background Service.

### Rule 2: Strongly-Typed Protobuf Contracts
* Every Skill and Service parameter and result **must** be declared using Protocol Buffers (`.proto`).
* Never pass arbitrary untyped JSON blobs or dictionaries across the Executive boundary. Protobuf schemas ensure compile-time safety, cross-language compatibility (Python/C++), and serialization validation.

### Rule 3: Progressive Asset Discovery (Reuse Before Re-inventing)
* Before writing a custom asset from scratch, check existing capabilities in:
  - Core catalog: [https://github.com/intrinsic-ai/intrinsic-core](https://github.com/intrinsic-ai/intrinsic-core) (`intrinsic/resources/catalog/`)
  - Core skills: [`.agents/skills/`](.agents/skills/)
  - Installed cluster assets: `inctl asset list --address localhost:17080`
* Only create a new custom Asset or Skill if no existing primitive meets the task requirements.

### Rule 4: Simulation-First Verification & Robotic Safety
> [!CAUTION]
> **Physical Robot Safety**: A physical robot or real hardware workcell may be connected to the cluster. **Never command physical motion or execute actions on real hardware (`--operation_mode=real`) without explicit consent and confirmation from the user.** Unintended physical motion poses severe safety risks to personnel and equipment.
* **Simulate First**: Always develop, test, and validate changes hermetically against Gazebo simulation (`--operation_mode=sim`) before considering physical hardware execution.
* **Inspect Spatial Alignment**: Inspect the world state before and after execution to verify that robot base poses, tool frames, collision geometries, and target frames match expectations prior to execution.

### Rule 5: Code Style & Formatting (Google Style)
The repository strictly adheres to **Google Code Style** across all languages, configuration files, and build definitions. Formatters run via `./tools/format.sh` and are verified in CI via `./tools/lint.sh`:
* **Bazel (`BUILD`, `BUILD.bazel`, `.bzl`)**: Formatted and linted using `buildifier` (`buildifier -r .` and `buildifier --lint=fix -r .`).
* **Python (`.py`)**: Formatted and linted using `ruff` (`ruff check --fix .` and `ruff format .`, configured in `pyproject.toml` with `line-length = 80` and `indent-width = 2`).
* **C++ & Protocol Buffers (`.cc`, `.h`, `.proto`)**: Formatted using `clang-format -i <file>` (`BasedOnStyle: Google`, `SortIncludes: true`).

---

## 3. Available Skills

Domain-specific guidance is packaged as modular skills located in `.agents/skills/`. Modern agent tools load skill frontmatter (name and description) automatically:

* [`intrinsic-core-api-overview`](.agents/skills/intrinsic-core-api-overview/SKILL.md)
* [`intrinsic-core-bazel`](.agents/skills/intrinsic-core-bazel/SKILL.md)
* [`intrinsic-core-concepts`](.agents/skills/intrinsic-core-concepts/SKILL.md)
* [`intrinsic-core-debugging`](.agents/skills/intrinsic-core-debugging/SKILL.md)
* [`intrinsic-core-robot-motion`](.agents/skills/intrinsic-core-robot-motion/SKILL.md)
* [`intrinsic-core-service-authoring`](.agents/skills/intrinsic-core-service-authoring/SKILL.md)
* [`intrinsic-core-skill-authoring`](.agents/skills/intrinsic-core-skill-authoring/SKILL.md)
* [`intrinsic-core-solution-building`](.agents/skills/intrinsic-core-solution-building/SKILL.md)
* [`intrinsic-core-solutions`](.agents/skills/intrinsic-core-solutions/SKILL.md)

---

## 4. Connecting and Running Solutions

Connect to the local cluster gateway and compose Behavior Trees using the Solution Building Language (SBL):

```python
from intrinsic.solutions import deployments

# Connect to the local Intrinsic Core cluster deployment
solution = deployments.connect(address="localhost:17080")

# Instantiate and execute skills via the Executive
adder_node = solution.skills.ai.intrinsic.adder(
  calculator=solution.resources.calculator_service,
  x=10,
  y=20,
)
solution.executive.run([adder_node])
assert solution.executive.get_value(adder_node.result).sum == 30
```

For advanced Behavior Tree composition, blackboard variables, and world mutations, see [`.agents/skills/intrinsic-core-solution-building`](.agents/skills/intrinsic-core-solution-building/SKILL.md).

---

## 5. Authoring Custom Assets, Skills, and Solutions

When developing new capabilities, consult the dedicated authoring skills:
* **Custom Skills**: Define Protobuf contracts, manifests, Python logic, and `py_skill` Bazel targets in [`.agents/skills/intrinsic-core-skill-authoring`](.agents/skills/intrinsic-core-skill-authoring/SKILL.md).
* **Custom Services**: Build containerized gRPC microservices with `intrinsic_service` in [`.agents/skills/intrinsic-core-service-authoring`](.agents/skills/intrinsic-core-service-authoring/SKILL.md).
* **Solutions**: Compose assets, simulators, hardware modules, and Behavior Trees using `intrinsic_solution` in [`.agents/skills/intrinsic-core-solution-building`](.agents/skills/intrinsic-core-solution-building/SKILL.md) and [`.agents/skills/intrinsic-core-solutions`](.agents/skills/intrinsic-core-solutions/SKILL.md).

---

## 6. Agent Operational Checklist

When tasked with creating, modifying, or executing an Intrinsic Core application, follow this cycle:

1. **Investigate Environment**:
   * Confirm cluster gateway accessibility (`curl -s localhost:17080/healthz` or `inctl doctor`).
   * Check currently installed assets (`inctl asset list --address localhost:17080`).
2. **Build and Bundle**:
   * Validate target analysis (`bazel build --nobuild //...`) or build targeted skills, services, or apps (e.g., `bazel build //src:omts_app`).
   * Identify generated bundle archives in `bazel-bin/`.
3. **Deploy & Instantiate**:
   * Install asset bundles using `inctl asset install <bundle.tar> --address localhost:17080`.
   * If introducing a service, add its instance (`inctl service add <id> --name=<name> --address localhost:17080`).
4. **Run Solution & Behavior Tree**:
   * Deploy or launch the solution via Bazel (`bazel run //:omts_solution -- --operation_mode=sim`) or run SBL Python execution scripts (`bazel run //src:omts_app`).
5. **Inspect & Extract Telemetry**:
   * Tail text logs (`inctl logs --follow ...`).
   * Check world geometry (`bazel --quiet run -c opt //tools/world:inspect_world -- --address localhost:17080`).
   * Extract high-resolution recordings or ROS 2 topics if debugging is required.
6. **Verify, Format & Clean Up**:
   * Run automated validation (`bazel test //tests/...`; avoid bare `bazel test //...` without `--build_tests_only`, which also builds the full `//:omts_solution` container image layers).
   * Format and lint changed files (`./tools/format.sh` and `./tools/lint.sh`).
   * Reset world state if required (`inctl world reset --address localhost:17080`).

For detailed workflows and CLI command syntax, refer to the relevant skill in [`.agents/skills/`](.agents/skills/).

---

## 7. Contribution Process & Pull Requests

OMTS accepts contributions and patches from the community. Before submitting contributions, complete the prerequisites below.

### Contributor License Agreement (CLA)
Contributions to this project must be accompanied by a [Contributor License Agreement](https://cla.developers.google.com/about) (CLA). You (or your employer) retain copyright to your contribution; the agreement grants permission to use and redistribute your contributions as part of the project.

* If you or your employer have already signed the Google CLA for another project, you do not need to sign it again.
* Visit [cla.developers.google.com](https://cla.developers.google.com/) to view existing agreements or sign a new one.
* AI Agents should remind developers about this requirement when preparing contributions to Intrinsic packages.

### GitHub Issues Process
* Search existing GitHub issues and Discourse community discussions ([intrinsic.discourse.group](https://intrinsic.discourse.group/)) before creating new reports to avoid duplicates and check for existing solutions or workarounds.
* Include reproduction steps, environment context, and relevant `inctl logs`.
* Tag reports with the appropriate component and priority labels.
* Never file security vulnerabilities as public GitHub issues. Use the private security reporting process in [SECURITY.md](SECURITY.md) instead.

### Pull Request (PR) Workflow
All submissions require review through GitHub pull requests:

1. Fork the repository on GitHub (required for community contributors without write access) and create a feature branch from `main` using descriptive naming (`feature/<name>`, `fix/<issue>`).
2. Verify changes locally using Bazel:
   ```bash
   bazel build --nobuild //...
   bazel test //tests/...
   ```
3. Format and lint all modified files according to Google Code Style (`./tools/format.sh` and `./tools/lint.sh`):
   * Bazel / Starlark files: `buildifier -r .` and `buildifier --lint=fix -r .`
   * Python files: `ruff check --fix .` and `ruff format .`
   * C++ and Protocol Buffers: `clang-format -i <file>`
4. Commit your changes with a descriptive commit message following Conventional Commits conventions.
5. Push the branch to your fork (or upstream repository) and submit a pull request against `main`. Link any resolved issues (for example, `Fixes #123`) in the PR description.
6. Address reviewer feedback and maintain clean commit history before merging.

---

## 8. Community Guidelines & Code of Conduct

OMTS follows [Google's Open Source Community Guidelines](https://opensource.google/conduct/). Full contribution instructions are found in [CONTRIBUTING.md](CONTRIBUTING.md).

### Inclusivity and Respect
Every Intrinsic open-source project and community provides an inclusive environment based on treating all individuals respectfully, regardless of gender identity and expression, sexual orientation, disabilities, neurodiversity, physical appearance, body size, ethnicity, nationality, race, age, religion, or similar personal characteristic.

Diverse technical opinions are valued, but respectful behavior is required:
* Be considerate, kind, constructive, and helpful in all interactions.
* Do not engage in demeaning, discriminatory, harassing, hateful, sexualized, or physically threatening speech, behavior, or imagery.
* Focus discussions on technical merits and project goals.

---

## 9. Security Policy & Vulnerability Reporting

The Intrinsic team treats software security and the physical safety of robotic systems with the highest priority. Vulnerabilities in robotics software can directly impact physical equipment and personnel safety.

> [!CAUTION]
> **Responsible Security Disclosure**: Never report security vulnerabilities through public GitHub issues, public discussions, or pull requests. Public exposure before remediation creates immediate risk for operational robotic deployments.

For instructions on reporting vulnerabilities and details on the intake process, refer to [SECURITY.md](SECURITY.md).
