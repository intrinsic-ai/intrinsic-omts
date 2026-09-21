# Instructions to build and sideload nvidia_pose_estimator_service to OMTS

## Prerequisites

* The target **scene object asset must already be installed on the OMTS cluster**. Step 3
  looks it up with `GetInstalledAsset`; if it is missing, the tool prints
  `Failed to fetch scene object ...` and **exits with status 0**, so it is easy to mistake
  for success. Check first:
  ```bash
  inctl_external asset list --asset_types=scene_object --address=localhost:17080
  ```
* `$ORG` is set (see `~/.flowstate.env`).

## Step 1. Free up the GPU

The GPU does not have enough memory to run both `inference_service` and
`nvidia_pose_estimator_service`, so the existing inference service has to go first.

Record what is installed before deleting, so you can put it back:

```bash
inctl_external asset list --asset_types=service --address=localhost:17080 --output_type=id_version
inctl_external service delete inference_service --address=localhost:17080
```

> **Undo.** To restore the original setup, delete the pose estimator instance and re-add
> the inference service using the id_version you recorded above:
>
> ```bash
> inctl_external service delete nvidia_pose_estimator_service --address=localhost:17080
> inctl_external service add <inference_service_id_version> \
>   --name=inference_service \
>   --address=localhost:17080
> ```
>
> Anything else on the cluster that depends on `inference_service` stays broken until you
> do this.

## Step 2. Build the asset bundle and transfer it to the OMTS machine

Follow the setup in the [README.md](README.md):

1. Manually download SAM3 (sam3.pt) and FoundationStereo (deployable_foundation_stereo_s_dynamic_v2.0.onnx)
   models to `src/nvidia_foundationpose/checkpoints/`.
2. Run `./src/nvidia_foundationpose/scripts/build.sh` to build external libraries and bindings
3. Build the bazel bundle

  * Assuming [insrc PR](https://github.com/intrinsic-ai/insrc/pull/55558) has been merged,

    ```bash
    bazel build @intrinsic-core//intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator:nvidia_pose_estimator_service_asset
    ```
  * If PR has not been merged,

    First manually export the IOC build using the `run_ioc.sh` script, e.g. to ~/ioc_export, then run
    
    ```bash
    bazel build \
          --override_module=intrinsic-core=~/ioc_export \
          --override_module=intrinsic_apis=~/ioc_export/intrinsic_apis \
          @intrinsic-core//intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator:nvidia_pose_estimator_service_asset
    ```

Transfer the built asset (bazel-bin/external/intrinsic-core+/intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator/nvidia_pose_estimator_service_asset.bundle.tar) to the OMTS machine, e.g. using rsync:

## Step 3. Register the pose estimator for the target scene object

Use the `ioc_train_service` tool to register a pose estimator, including the text prompt
used for SAM 3. We find the prompt `"metallic block"` to work well.

> **These instructions assume the cuboid asset names below**
> (scene object `ai.intrinsic.cuboid_2_5x5x10cm`, pose estimator `ai.intrinsic.mycuboid`).
> If you are deploying a different part, substitute your own ids consistently here and in
> Step 6.

```bash
bazel run \
  //incode/intrinsic_perception/intrinsic/perception/service/ioc_train_service/tools:register_using_ioc_train_service -- \
  --address=localhost:17080 \
  --org=$ORG \
  --scene_object_id="ai.intrinsic.cuboid_2_5x5x10cm" \
  --pose_estimator_id="ai.intrinsic.mycuboid" \
  --refinement_iters=2 \
  --confidence_threshold=0.6 \
  --visibility_threshold=0.6 \
  --text_prompt="metallic block"
```

`--refinement_iters=2` is below the tool default of 3, trading a little pose accuracy for
latency.

> This tool instantiates `IocTrainService` **in-process**, so there is no train service to
> deploy on the cluster -- the only cluster dependency is the installed-assets service.
> The tool's "are you talking to the right train service?" warning can be ignored.

## Step 4. Install the asset bundle

Run from the directory you transferred the bundle into:

```bash
inctl_external asset install nvidia_pose_estimator_service_asset.bundle.tar \
  --address=localhost:17080
```

## Step 5. Create the service instance

Installing the asset does not start anything. Create an instance of it in the solution:

```bash
inctl_external service add ai.intrinsic.nvidia_pose_estimator_service \
  --name=nvidia_pose_estimator_service \
  --address=localhost:17080
```

`--name` is technically optional, but pass it explicitly: this is the resource name OMTS
resolves in Step 7, and it has to match. No `--config` is needed -- the bundle ships
`configs/default_config.textproto` as its default configuration.

## Step 6. Verify the service is up

**Do not start OMTS until this step passes.** The service is not ready the moment the pod
exists, and `main.py` resolves the perception resource at startup.

> **First startup takes roughly 10-20 minutes.** The service compiles its TensorRT engines
> on the target GPU before it starts serving, and the logs look idle while it does.
> Subsequent restarts hit the engine cache and come up quickly.

```bash
# Asset is installed
inctl_external asset list --asset_types=service --address=localhost:17080

# Instance is running
kubectl get pods -n app-resources --context=flowstate | grep nvidia-pose

# Warmup progress
kubectl logs -n app-resources <pod-name> -c rs-nvidia-pose-estimator-service --context=flowstate -f
```

The service is ready once the pod is `Running`/`Ready` **and** the logs show:

```text
[Warmup Resident] Staged resident warmup completed successfully. All models loaded and ready for low-latency inference.
-- Server listening at port : [::]:50051 ...
```

## Step 7. Point OMTS at the new service

Only once Step 6 passes. No source edit is required; `main.py` already exposes the
relevant flags. Using the **same asset ids as Step 3**:

```bash
--perception_service_name=nvidia_pose_estimator_service \
--pose_estimator_id=ai.intrinsic.mycuboid \
--scene_object_id=ai.intrinsic.cuboid_2_5x5x10cm
```

The last two matter: they default to `ai.intrinsic.raw_stock_2x3x5_estimator` /
`ai.intrinsic.raw_stock_2x3x5`, so omitting them runs perception against the wrong part.