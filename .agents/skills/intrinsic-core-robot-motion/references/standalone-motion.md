# Standalone motion script execution and template

## Execution harness and hermetic Bazel targets

The Intrinsic SDK libraries (`intrinsic.icon.python` and `intrinsic.world.python`) are provided hermetically through Bazel module dependencies, not in host system site-packages. Standalone motion scripts must execute via hermetic Bazel targets rather than unmanaged host Python (`python3 script.py`), which fails with missing module errors.

To execute a standalone motion script:
1. **Initialize workspace if required**:
   ```bash
   inctl bazel init
   ```
   Ensure external SDK dependencies are declared in `MODULE.bazel` as detailed in [intrinsic-core-bazel](../../intrinsic-core-bazel/SKILL.md).
2. **Define a hermetic `py_binary` target**:
   In your package `BUILD` file:
   ```python
   load("@rules_python//python:defs.bzl", "py_binary")

   py_binary(
       name = "jog_flange",
       srcs = ["jog_flange.py"],
       deps = [
           "@ai_intrinsic_sdks//intrinsic/icon/python:icon_api",
           "@ai_intrinsic_sdks//intrinsic/world/python:object_world_client",
           "@ai_intrinsic_sdks//intrinsic/world/proto:object_world_service_py_pb2_grpc",
       ],
   )
   ```
3. **Execute via Bazel**:
   ```bash
   bazel run //:jog_flange -- --address=localhost:17080
   ```

## Standalone motion script implementation template

A minimal standalone script querying the robot initial flange transform and calculating a safe jog displacement within the 10 mm envelope:

```python
"""Sample standalone robot micro-jog script executed hermetically via Bazel."""

import argparse
import grpc
from intrinsic.world.proto import object_world_service_pb2_grpc
from intrinsic.world.python import object_world_client

CANONICAL_P0 = (-0.093094, -0.339947, 0.385646)


def main() -> None:
  parser = argparse.ArgumentParser(description="Hermetic motion script.")
  parser.add_argument(
      "--address", default="localhost:17080", help="Cell address"
  )
  args = parser.parse_args()

  channel = grpc.insecure_channel(args.address)
  stub = object_world_service_pb2_grpc.ObjectWorldServiceStub(channel)
  client = object_world_client.ObjectWorldClient(world_id="world", stub=stub)

  ur = client.get_kinematic_object("ur_module")
  tf = client.get_transform(client.root, ur.flange)
  curr_pos = tuple(float(x) for x in tf.translation[:3])
  print(f"Initial flange position: {curr_pos}")

  # Commanded micro-jog displacement (e.g. 4 mm along X-axis, strictly within 1 mm - 8 mm)
  delta_x, delta_y, delta_z = 0.004, 0.0, 0.0
  target_pos = (
      curr_pos[0] + delta_x,
      curr_pos[1] + delta_y,
      curr_pos[2] + delta_z,
  )

  # Verify target position remains strictly <= 10 mm from canonical P0
  disp_from_p0 = (
      (target_pos[0] - CANONICAL_P0[0]) ** 2
      + (target_pos[1] - CANONICAL_P0[1]) ** 2
      + (target_pos[2] - CANONICAL_P0[2]) ** 2
  ) ** 0.5
  if disp_from_p0 > 0.01:
    raise ValueError(
        f"Target pose exceeds 10 mm limit from canonical P0: {disp_from_p0:.4f} m"
    )


if __name__ == "__main__":
  main()
```
