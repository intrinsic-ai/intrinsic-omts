# Intrinsic SDK dependencies and `Python` imports

When developing Intrinsic Core skills and services, all first-party Intrinsic capabilities must be declared under the `@ai_intrinsic_sdks` repository in `BUILD` files and imported from the corresponding `intrinsic.<domain>` modules in Python.

## Fundamental import contract

Every Intrinsic SDK capability requires two aligned declarations:
1. **In `BUILD` (or `BUILD.bazel`)**: Declare the target dependency under `@ai_intrinsic_sdks//...`.
2. **In Python source**: Import the corresponding module under `from intrinsic.<domain>... import ...`.

```
[ BUILD target dependency ]                             [ Python import ]
@ai_intrinsic_sdks//intrinsic/foo:bar            ===>   from intrinsic.foo import bar
@ai_intrinsic_sdks//intrinsic/foo/bar:baz        ===>   from intrinsic.foo.bar import baz
@ai_intrinsic_sdks//intrinsic/skills/proto:skills_py_pb2 ===> from intrinsic.skills.proto import skills_pb2
```

## Target naming conventions

In the Intrinsic SDK (`@ai_intrinsic_sdks`), target labels follow consistent naming conventions:
- **Protobuf messages**: Named `<name>_py_pb2`. Compiles `<name>.proto` into `<name>_pb2.py` and `<name>_pb2.pyi`.
  - *Example*: `@ai_intrinsic_sdks//intrinsic/skills/proto:skills_py_pb2`
  - *Anti-pattern*: Do not use `<name>_py_proto` (this legacy suffix is not used in the SDK).
- **gRPC service client and servicer stubs**: Named `<name>_py_pb2_grpc`. Compiles service definitions into `<name>_pb2_grpc.py`.
  - *Example*: `@ai_intrinsic_sdks//intrinsic/skills/proto:skill_service_py_pb2_grpc`
- **Pure Python libraries**: Named by module utility name without `_pb2`.
  - *Example*: `@ai_intrinsic_sdks//intrinsic/skills/python:skill_interface`

## Canonical SDK target mapping table

The following table provides verified mappings between platform capabilities, Bazel dependencies, and Python imports across core Intrinsic domains:

| Domain and capability | Bazel dependency label (`BUILD`) | Python import syntax |
| :--- | :--- | :--- |
| **Skill interface** | `@ai_intrinsic_sdks//intrinsic/skills/python:skill_interface` | `from intrinsic.skills.python import skill_interface` |
| **Skill proto** | `@ai_intrinsic_sdks//intrinsic/skills/proto:skills_py_pb2` | `from intrinsic.skills.proto import skills_pb2` |
| **Skill footprint** | `@ai_intrinsic_sdks//intrinsic/skills/proto:footprint_py_pb2` | `from intrinsic.skills.proto import footprint_pb2` |
| **Skill manifest** | `@ai_intrinsic_sdks//intrinsic/skills/proto:skill_manifest_py_pb2` | `from intrinsic.skills.proto import skill_manifest_pb2` |
| **Skill service gRPC** | `@ai_intrinsic_sdks//intrinsic/skills/proto:skill_service_py_pb2_grpc` | `from intrinsic.skills.proto import skill_service_pb2_grpc` |
| **Runtime context** | `@ai_intrinsic_sdks//intrinsic/resources/proto:runtime_context_py_pb2` | `from intrinsic.resources.proto import runtime_context_pb2` |
| **Resource handle** | `@ai_intrinsic_sdks//intrinsic/resources/proto:resource_handle_py_pb2` | `from intrinsic.resources.proto import resource_handle_pb2` |
| **Resource client** | `@ai_intrinsic_sdks//intrinsic/resources/client:resource_client` | `from intrinsic.resources.client import resource_client` |
| **Object world client** | `@ai_intrinsic_sdks//intrinsic/world/python:object_world_client` | `from intrinsic.world.python import object_world_client` |
| **Object world service** | `@ai_intrinsic_sdks//intrinsic/world/proto:object_world_service_py_pb2` | `from intrinsic.world.proto import object_world_service_pb2` |
| **Object world gRPC** | `@ai_intrinsic_sdks//intrinsic/world/proto:object_world_service_py_pb2_grpc` | `from intrinsic.world.proto import object_world_service_pb2_grpc` |
| **Object world updates** | `@ai_intrinsic_sdks//intrinsic/world/proto:object_world_updates_py_pb2` | `from intrinsic.world.proto import object_world_updates_pb2` |
| **Geometry types** | `@ai_intrinsic_sdks//intrinsic/geometry/proto:geometry_py_pb2` | `from intrinsic.geometry.proto import geometry_pb2` |
| **Geometry service** | `@ai_intrinsic_sdks//intrinsic/geometry/proto:geometry_service_py_pb2` | `from intrinsic.geometry.proto import geometry_service_pb2` |
| **Point cloud proto** | `@ai_intrinsic_sdks//intrinsic/geometry/proto:point_cloud_py_pb2` | `from intrinsic.geometry.proto import point_cloud_pb2` |
| **Triangle mesh proto** | `@ai_intrinsic_sdks//intrinsic/geometry/proto:mesh_py_pb2` | `from intrinsic.geometry.proto import mesh_pb2` |
| **Math pose proto** | `@ai_intrinsic_sdks//intrinsic/math/proto:pose_py_pb2` | `from intrinsic.math.proto import pose_pb2` |
| **Math matrix proto** | `@ai_intrinsic_sdks//intrinsic/math/proto:matrix_py_pb2` | `from intrinsic.math.proto import matrix_pb2` |
| **Eigenmath library** | `@ai_intrinsic_sdks//intrinsic/eigenmath/python:eigenmath` | `from intrinsic.eigenmath.python import eigenmath` |
| **Motion planning client** | `@ai_intrinsic_sdks//intrinsic/motion_planning/python:motion_planning_client` | `from intrinsic.motion_planning.python import motion_planning_client` |

## Discovering SDK `APIs` and schemas

Bazel extracts the Intrinsic SDK archive into its external output base under `$(bazel info output_base)/external/` (typically directory `ai_intrinsic_sdks+`).

### Resolving SDK filesystem location

Resolve and verify the SDK path in a single command:
```bash
SDK_DIR=$(ls -d "$(bazel info output_base)/external/"*ai_intrinsic_sdks* 2>/dev/null | head -n 1)
test -n "${SDK_DIR}" && test -d "${SDK_DIR}" && echo "Resolved SDK to: ${SDK_DIR}"
```

If `${SDK_DIR}` is empty, trigger lazy archive extraction by requesting a lightweight target build:
```bash
bazel build @ai_intrinsic_sdks//intrinsic/skills/proto:skills_py_pb2
```

### Targeted domain navigation

List available functional domains safely without recursive traversal:
```bash
ls -d "${SDK_DIR}/intrinsic"/*/ | sed -e 's#^.*/intrinsic/##' -e 's#/$##' | sort
```

Key functional domains include:
- `skills/`: Skill interfaces, manifests, and execution contexts.
- `resources/`: Service runtime contexts and resource management.
- `world/`: Object world client, models, and world updates.
- `geometry/`: Meshes, point clouds, collision shapes, and coordinate frames.
- `eigenmath/` and `math/`: Poses, rotations, transformations, and vectors.
- `motion_planning/`: Trajectory planners, kinematic solvers, and motion constraints.

### Scoped query and inspection patterns

Avoid searching the entire Bazel output base with unconstrained `grep -rn`. Always scope searches to specific leaf domains with result limits:

```bash
# Search for Python class or function definitions
grep -rn -m 20 --include="*.py" -E "^(class|def) " "${SDK_DIR}/intrinsic/skills/python/"

# Search for protobuf message schemas
grep -rn -m 20 --include="*.proto" "^message " "${SDK_DIR}/intrinsic/resources/proto/"

# Inspect specific proto fields
grep -n -A 15 "message RuntimeContext" "${SDK_DIR}/intrinsic/resources/proto/runtime_context.proto"
```

Use `bazel query` to inspect target labels directly from Bazel's dependency graph without compiling:

```bash
# List all targets in a proto package
bazel query '@ai_intrinsic_sdks//intrinsic/skills/proto:all'

# List all Python rules in a domain
bazel query 'kind("py_.*", @ai_intrinsic_sdks//intrinsic/skills/...)'

# Inspect direct dependencies of a target
bazel query 'deps(@ai_intrinsic_sdks//intrinsic/skills/python:skill_interface, 1)'
```
