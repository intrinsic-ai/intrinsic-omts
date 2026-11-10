# OMTS on NVIDIA Jetson AGX Orin (linux/arm64)

This guide cross-compiles `//:omts_solution` for `linux/arm64` (Orin GPU,
`sm_87`) on an x86_64 workstation and deploys it in simulation mode to a
Jetson AGX Orin running the Intrinsic Core runtime.

## 1. Prerequisites

### Target (Jetson AGX Orin)

- JetPack 7.2 (L4T R39.2, Ubuntu 24.04) with the preinstalled NVIDIA L4T
  driver and `nvidia-container-toolkit`. Do not install the desktop
  `nvidia-driver-*` packages on L4T.
- `git`, `git-lfs`, and `util-linux-extra`:

  ```bash
  sudo apt-get update
  sudo apt-get install -y git git-lfs util-linux-extra
  git lfs install
  ```

- K3s and GPU support, installed with the arm64- and Tegra-aware
  `intrinsic_runtime/setup_k3s.sh` and `intrinsic_runtime/setup_nvidia.sh`
  from a clone of the same intrinsic-core revision used for the build:

  ```bash
  ./intrinsic_runtime/setup_k3s.sh
  newgrp containerd
  sudo ./intrinsic_runtime/setup_nvidia.sh
  kubectl describe node | grep nvidia.com/gpu   # expect a non-zero count
  ```

- Native `linux/arm64` images for components that are published for amd64
  only, imported into the K3s containerd (`sudo k3s ctr images import -`):
  `chart-assignment-controller` (built from
  `github.com/googlecloudrobotics/core`, `src/go/cmd/chart-assignment-controller`)
  and `code-execution-jupyter-server`.
- `intrinsic-base` and `inctl` built for `linux/arm64` from intrinsic-core
  (no arm64 release artifacts are published):

  ```bash
  # In the intrinsic-core checkout, on the workstation:
  bazel build --platforms=//bazel:linux_arm64 \
    //intrinsic_runtime:intrinsic_base \
    //intrinsic_sdk/intrinsic/tools/inctl:inctl_external
  ```

  Copy both to the target, install `inctl_external` as `/usr/local/bin/inctl`,
  then extract `intrinsic_base.tar` and run `./intrinsic-base`. Verify with
  `kubectl get pods -A` that all pods reach `Running`.

### Workstation (x86_64)

- The standard OMTS development setup from the [README](../README.md)
  (Bazelisk, `git-lfs`, CUDA-capable toolchain prerequisites).
- An intrinsic-core checkout that contains insrc#59422 (linux/arm64 platform
  and runtime support). Until that change is part of an intrinsic-core
  release, OMTS must be built against the checkout with `--override_module`.
- The pose estimator model files staged as described in
  `intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator/README.md`
  of intrinsic-core.

## 2. Build

From the OMTS checkout, with `CORE` set to the intrinsic-core checkout:

```bash
CORE=<path-to-intrinsic-core-checkout>
P=@intrinsic-core//intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator

bazel build \
  --platforms=@intrinsic-core//bazel:linux_arm64 \
  --override_module=intrinsic-core="${CORE}" \
  --override_module=intrinsic_apis="${CORE}/intrinsic_apis" \
  --repo_env=NVIDIA_POSE_ESTIMATOR_MODELS_DIR=<path-to-staged-models> \
  //:omts_solution \
  "${P}:foundationpose_model_data_asset" \
  "${P}:foundationstereo_model_data_asset" \
  "${P}:sam3_model_data_asset" \
  @intrinsic-core//intrinsic/assets/build_defs:solution_update
```

`@intrinsic-core//bazel:linux_arm64` selects `--cpu=aarch64` and
`--@rules_cuda//cuda:archs=sm_87`; no additional flags are required. Outputs
are written to `bazel-out/aarch64-opt/bin/`. All container images in
`omts_solution.dir` are `linux/arm64`.

The x86_64 build is unchanged: `bazel build //:omts_solution`.

## 3. Deploy

Copy the build outputs to the target:

```bash
OUT=bazel-out/aarch64-opt/bin
MODELS=${OUT}/external/intrinsic-core+/intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator
rsync -az --delete --copy-links "${OUT}/omts_solution.dir/" <target>:~/omts_solution.dir/
rsync -az --copy-links \
  "${OUT}/external/intrinsic-core+/intrinsic/assets/build_defs/solution_update" \
  "${MODELS}/foundationpose_model_data_asset.bundle.tar" \
  "${MODELS}/foundationstereo_model_data_asset.bundle.tar" \
  "${MODELS}/sam3_model_data_asset.bundle.tar" \
  <target>:~/
```

On the target, deploy the solution in simulation mode, then install the model
data assets. `inctl asset install` requires a running solution.

```bash
~/solution_update ~/omts_solution.dir/solution_manifest.txtpb \
  --address=localhost:17080 --operation_mode=sim
for a in foundationpose foundationstereo sam3; do
  inctl asset install ~/${a}_model_data_asset.bundle.tar --address=localhost:17080
done
# The pose estimator starts before the data assets exist; restart it once.
kubectl delete pod -n app-resources rs-nvidia-pose-estimator-service-0
```

Optionally run `sudo jetson_clocks` to reduce TensorRT engine build and
inference time.

## 4. Run

Build the client tools for x86_64 on the workstation as described in the
[README](../README.md) and point them at the target through an SSH tunnel:

```bash
ssh -N -L 27080:localhost:17080 <target> &
bazel run //tools/world:apply_scene_updates -- --address=localhost:27080 --setup=omts
bazel run //src:omts_app -- --address=localhost:27080 \
  --config=configs/omts/app_config.yaml --num_cycles=1
```

Use a local port other than an existing tunnel's port (here `27080`) so the
tools do not connect to a different cluster.

## 5. Known limitations

- **Simulation only.** The prebuilt amd64-only bundles are omitted from the
  arm64 solution (`@platforms//cpu:aarch64` selects in [`BUILD`](../BUILD)):
  Hand-E gripper service and `gripper_cmd_skill`, `moveit_plan_grasp_skill`,
  Flowstate ROS bridge, and the Orbbec Gemini driver. Physical gripper and
  camera hardware cannot be driven, and `--grasp_planner=moveit` is
  unavailable.
- **Emulated gripper.** `configs/omts/app_config.yaml` sets
  `gripper.allow_world_joint_fallback: true`, so `RobotiqGripper` sets the
  gripper finger joints in the belief world through `update_world` when
  `gripper_cmd_skill` is missing. The fallback is refused for non-simulated
  solutions and logs a warning when active. It defaults to `false`; the
  `lab_bb_01` setup therefore does not run on arm64.
- **First start is slow.** The first FoundationPose warmup on Orin builds the
  TensorRT engines and takes about 12 minutes. Engines are cached in the
  pose estimator service volume for subsequent starts.
- **Module overrides.** The `--override_module` flags are required until
  insrc#59422 is included in an intrinsic-core release that OMTS depends on.
