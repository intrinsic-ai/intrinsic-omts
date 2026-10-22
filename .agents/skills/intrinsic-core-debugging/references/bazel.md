# Bazel dependency and compilation debugging

This guide details the anti-thrashing circuit breakers and diagnostic decision tree for resolving Bazel build, test, and external dependency failures in Intrinsic Core.

## Constitutional 0-0-2 circuit breaker rules

To prevent exploratory rabbit holes, action thrashing, and plan churn, every agent must adhere to the **0-0-2 circuit breaker rules**:
1. **0 binary scraping attempts**: Never execute `strings`, regular expressions, or Python scrapers on `/usr/local/bin/inctl` or compiled binaries. The schema and API definitions exist in the Intrinsic SDK (`${SDK_DIR}/intrinsic/`).
2. **0 manifest guessing attempts**: Never guess tarball manifest formats, binary protobuf headers, or directory layouts for `inctl asset install` or `inctl skill install`. If `inctl asset install` or sideloading fails with `failed to dial "/run/containerd/containerd.sock": connect: connection refused` or connection refused on `localhost:17127`, stop after at most 2 attempts. The failure is on the backend server side (the containerd socket and its mount belong inside `k3s` on the cluster host, not in the client sandbox). Do not attempt to debug local `/run` sockets, search for sockets with `find /run`, or attempt to proxy `/run/containerd/containerd.sock`. Local development requires building hermetic Bazel targets (`py_binary`, `py_test`) and running verification tests locally.
3. **2-attempt retry cap**: If `bazel run //:requirements.update` or `bazel build //...` fails twice consecutively after editing `requirements.in` or `BUILD`, halt modifications immediately. Execute the diagnostic decision tree below before making any further edits.

## Input-aware diagnostic decision tree

Select diagnostic and repair actions based on the observed error signature:

```
[Build or dependency error occurs]
├── Error: "no such package '@@[unknown repo 'ai_intrinsic_sdks'...]'"
│   └── Cause: MODULE.bazel lacks @ai_intrinsic_sdks declaration.
│       └── Fix: Add bazel_dep(name = "ai_intrinsic_sdks") and archive_override to MODULE.bazel.
│
├── Error: "no such package '@@[unknown repo 'pypi_deps'...]'"
│   └── Cause: hub_name mismatch or use_repo missing in MODULE.bazel.
│       └── Fix: Confirm pip.parse(hub_name = "pypi_deps") and use_repo(pip, "pypi_deps") are declared.
│
├── Error: "no such package '@@rules_python++pip+pypi_deps//<pkg>'"
│   └── Cause: Missing transitive PyPI dependency in requirements_lock.txt.
│       └── Fix: Add package name to requirements.in, re-run bazel run //:requirements.update, and reference in deps.
│
├── Error: "target '<name>_py_proto' not found in package..."
│   └── Cause: Incorrect proto rule naming convention.
│       └── Fix: Change target label from _py_proto to _py_pb2 (or _py_pb2_grpc for services).
│
├── Error: "error reading file '//:requirements_lock.txt': No such file"
│   └── Cause: requirements_lock.txt does not exist on filesystem.
│       └── Fix: Run touch requirements_lock.txt and re-run bazel run //:requirements.update.
│
├── Error: "The requirements_lock.txt file is out of date"
│   └── Cause: requirements.in was modified without re-compiling the lockfile.
│       └── Fix: Run bazel run //:requirements.update to regenerate lockfile.
│
├── Error: "SDK_DIR is empty / external directory not found"
│   └── Cause: Bazel fetches external archives lazily upon first build reference.
│       └── Fix: Run bazel build @ai_intrinsic_sdks//intrinsic/skills/proto:skills_py_pb2 to trigger fetch.
│
├── Error: "Config value '...' is not defined..."
│   └── Cause: Internal monorepo config flag passed in standalone workspace.
│       └── Fix: Omit monorepo config flags and invoke standard bazel build //... or bazel test //....
│
└── Error: "ModuleNotFoundError: No module named 'intrinsic...'"
    └── Cause: Missing BUILD dependency or in-tree fake package shadowing SDK.
        ├── Check 1: Does a local directory named intrinsic/ exist in the workspace?
        │   └── Fix: Run rm -rf ./intrinsic/ to remove the shadowing stub package.
        └── Check 2: Does the consumer BUILD target declare the SDK dependency in deps?
            └── Fix: Add "@ai_intrinsic_sdks//intrinsic/<domain>:..." to deps attribute.
```

## Resolving dynamic linker and interpreter `ABI` mismatches

When building Python C-extensions or running tests under Bazel:
- **Symptom**: `ImportError: ...: undefined symbol: PyModule_Create2` or interpreter version discrepancies.
- **Cause**: The execution environment injected host Python directories into `PYTHONPATH` or bound a host `py_runtime_pair`.
- **Remediation**:
  1. Unset `PYTHONPATH` before executing Bazel commands (`unset PYTHONPATH`).
  2. Verify that `.bazelrc` does not set `--repo_env=PYTHONPATH` or point to `/usr/lib/python3/dist-packages`.
  3. Ensure `MODULE.bazel` declares the hermetic toolchain:
     ```python
     python = use_extension("@rules_python//python/extensions:python.bzl", "python")
     python.toolchain(is_default = True, python_version = "3.11")
     ```
