---
name: intrinsic-core-service-authoring
description: >-
  Intrinsic Core microservice authoring, ServiceManifest definitions (real_spec vs sim_spec), RuntimeContext port bindings (gRPC port 1 vs HTTP port 7), SIGTERM lifecycle handling, and .binpb sideloading.
  Triggers: developing, packaging, configuring, or sideloading custom Intrinsic microservices.
  Target subsystems: SERVICES, KUBERNETES, INGRESS, RUNTIMECONTEXT.
  Disqualifying anti-keywords: behavior tree leaf actions (use intrinsic-core-skill-authoring), general Bazel dependencies (use intrinsic-core-bazel), motion planning (use intrinsic-core-robot-motion).
---

# Authoring Intrinsic Core services

Prerequisite: read the [intrinsic-core-bazel skill](../intrinsic-core-bazel/SKILL.md).

## Service manifest and container specifications

Custom services are defined by `intrinsic_proto.services.ServiceManifest`. The manifest configures identity, configuration descriptors, ingress routes, and Kubernetes pod execution specs (`real_spec` for physical robots, `sim_spec` for simulation), built via `python_oci_image` and `intrinsic_service`. Scaffolding can be generated via `inctl service create <asset_id> --language python` inside a Bazel workspace.

```textproto
metadata {
  id { package: "com.example" name: "telemetry_service" }
  vendor { display_name: "Example Organization" }
  display_name: "Telemetry Service"
}
service_def {
  config_message_full_name: "com.example.TelemetryConfig"
  service_proto_prefixes: "/com.example.TelemetryService/"
  http_config: {}
  real_spec {
    image {
      archive_filename: "telemetry_service_image.tar"
      settings {
        args: ["--mode=real"]
      }
    }
  }
  sim_spec {
    image {
      archive_filename: "telemetry_service_image.tar"
      settings {
        args: ["--mode=sim"]
      }
    }
  }
}
```

## How to communicate with an Intrinsic asset vs. an Intrinsic Core platform service

At startup, the platform mounts `intrinsic_proto.config.RuntimeContext` at `/etc/intrinsic/runtime_config.pb` (`INTRINSIC_RUNTIME_CONFIG`). Ports are allocated based on manifest declarations:

| Target protocol | Manifest requirement in `service_def` | `RuntimeContext` port binding | Ingress routing mechanism |
| :--- | :--- | :--- | :--- |
| **gRPC microservice** | `service_proto_prefixes: ["/<pkg>.<Service>/"]` | Bind server to `context.port` (field 1). | Envoy routes asset calls via URI prefix + `x-resource-instance-name: <name>` header, unlike platform services (e.g. `ObjectWorldService`). |
| **HTTP ingress server** | `http_config: {}` (mandatory empty message) | Bind HTTP server to `context.http_port` (field 7). | Envoy exposes endpoint at `/ext/services/<instance_name>/`. Omitting `http_config` sets `http_port` to 0. |

## Resilient service entrypoint and lifecycle management

Kubernetes sends `SIGTERM` to PID 1 and waits 60 seconds before issuing `SIGKILL`. Services must trap `SIGTERM` and `SIGINT`, stop listeners cleanly, and call `sys.exit(0)`. Restrict filesystem inspection to workspace paths; do not scan root `/` or system mounts (`/proc`, `/sys`).

```python
import http.server
import os
from pathlib import Path
import signal
import sys
import threading
from google.protobuf import any_pb2
from google.protobuf import message
import grpc
from intrinsic.resources.proto import runtime_context_pb2


def load_runtime_context() -> runtime_context_pb2.RuntimeContext:
  """Loads RuntimeContext from INTRINSIC_RUNTIME_CONFIG."""
  config_path = Path(os.environ.get("INTRINSIC_RUNTIME_CONFIG", "/etc/intrinsic/runtime_config.pb"))
  context = runtime_context_pb2.RuntimeContext()
  if config_path.is_file():
    context.ParseFromString(config_path.read_bytes())
  return context


def unpack_config(context: runtime_context_pb2.RuntimeContext, config: message.Message) -> None:
  """Unpacks context.config, raising on type_url mismatch or missing envelope."""
  if not context.config.Unpack(config):
    raise ValueError(f"Failed to unpack config: {context.config.type_url}")


def run_service(grpc_server: grpc.Server | None = None, httpd: http.server.HTTPServer | None = None) -> None:
  """Runs servers and stops cleanly upon SIGTERM or SIGINT."""
  stop_event = threading.Event()
  def handle_stop(signum: int, frame: object) -> None:
    del signum, frame
    if grpc_server:
      grpc_server.stop(grace=1.0)
    if httpd:
      threading.Thread(target=httpd.shutdown, daemon=True).start()
    stop_event.set()
  signal.signal(signal.SIGTERM, handle_stop)
  signal.signal(signal.SIGINT, handle_stop)
  if grpc_server:
    grpc_server.start()
  if httpd:
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
  stop_event.wait()
  sys.exit(0)
```

## Hermetic `bazel` build targets

Define microservice packages using standard `rules_python` rules (`py_library`, `py_binary`) in `BUILD`. Declare dependencies on generated protobuf targets and SDK libraries via `@ai_intrinsic_sdks`. See [intrinsic-core-bazel](../intrinsic-core-bazel/SKILL.md) for canonical dependency mappings, `MODULE.bazel` configuration, and hermetic build targets.

## Sideloading workflow and binary configuration

Deploy custom services into the workcell using the two-step catalog and instantiation workflow:
1. **Install asset bundle**: `inctl asset install /path/to/bundle.tar --address=localhost:17080`
   > [!IMPORTANT]
   > **Backend containerd socket circuit breaker**: If `inctl asset install` fails with containerd socket connection refused (`/run/containerd/containerd.sock` or `localhost:17127`), stop after <= 2 attempts. The socket belongs in backend `k3s` on the cluster host, NOT in the local client sandbox. Do NOT debug local `/run` sockets or proxy unix sockets; verify the service hermetically via Bazel (`bazel build //...`, `bazel test //...`).
2. **Instantiate service**: Pass binary `.binpb` protobufs to `inctl service add --config`; do not supply `.textproto` or JSON files (which fail with wire-format unmarshaling errors):
   ```bash
   inctl service add "<package.service_name>" --name=<instance_name> --config=/path/to/my_config.binpb --address=localhost:17080
   ```
   Generate `.binpb` by packing the compiled config into `google.protobuf.Any`:
   ```python
   any_msg = any_pb2.Any()
   any_msg.Pack(my_service_config)
   Path("/path/to/my_config.binpb").write_bytes(any_msg.SerializeToString())
   ```
3. **Verify running state**: `inctl service state list --address=localhost:17080` or `inctl asset instance list --address=localhost:17080`.

## Paired safety guardrails and anti-patterns

1. **Zero-fallback policy**: Import and parse authoritative protobuf definitions directly; do not implement synthetic fallbacks, mock contexts, or dummy ports that mask environment failures.
2. **Binary configuration format**: Pass serialized binary `google.protobuf.Any` (`.binpb`) to `inctl service add --config`; do not supply `.textproto` or JSON files.
3. **Workspace-scoped filesystem exploration**: Restrict tool exploration to local workspace directories; do not scan root `/` or virtual system mounts (`/proc`, `/sys`).
4. **Backend socket circuit breaker**: If sideloading fails with containerd socket connection refused, halt after <= 2 attempts; verify hermetically via Bazel rather than debugging local `/run` sockets.

## System 2 reflection and anti-thrashing circuit breaker

- **Pre-execution reflection checkpoint**: Before building bundles or instantiating services, verify: (1) `metadata.vendor.display_name` is present; (2) `service_proto_prefixes` uses `"/<pkg>.<Service>/"` format; (3) container images use `archive_filename: "<name>.tar"`; (4) CLI args reside in `image.settings.args`; and (5) `--config` references a binary `.binpb`.
- **Anti-thrashing circuit breaker**: Enforce a <= 2 retry cap on deployment failures. If `inctl service add` fails or enters `Faulted`, inspect `inctl service state list --address=localhost:17080` and verify `context.config.Unpack()`. If containerd fails with connection refused (`localhost:17127`), halt after <= 2 attempts and verify hermetically via Bazel.

## Completion criteria

- [ ] **Manifest schema**: Defines `vendor.display_name`, `archive_filename`, `settings.args`, `service_proto_prefixes: ["/<pkg>.<Service>/"]`, and `http_config: {}`.
- [ ] **Port bindings**: gRPC server binds to `context.port` (field 1); HTTP server binds to `context.http_port` (field 7).
- [ ] **Lifecycle handling**: Registers `SIGTERM` and `SIGINT` handlers invoking `sys.exit(0)`.
- [ ] **Configuration format**: Sideloading configuration packaged as binary `google.protobuf.Any` (`.binpb`).
- [ ] **Verification**: `inctl service state list` reports `State: Enabled`, or hermetic local build and test via Bazel (`bazel build //...`, `bazel test //...`) succeed on backend socket failures.
