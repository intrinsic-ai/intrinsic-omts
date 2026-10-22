# Third-party `PyPI` dependencies and rules_python

When developing Intrinsic Core skills, services, and applications, third-party Python packages must be managed hermetically through `rules_python` and pinned in an immutable lockfile (`requirements_lock.txt`).

## Hermetic toolchain vs host interpreter alignment

The workspace dependency graph is governed strictly by declarative Bazel configuration:
- **Single source of truth**: `MODULE.bazel` declares all external module dependencies (`bazel_dep`) and module extension rules (`use_extension`).
- **No hybrid execution**: Never rely on host system Python (`/usr/bin/python3`), host site-packages (`/usr/lib/python3/dist-packages`), or manual `pip install`. Mixing host interpreter state with Bazel sandbox builds causes dynamic linker crashes (`PyModule_Create2` ABI mismatch), module shadowing, and non-reproducible evaluation failures.
- **Hermetic toolchain registration**: The workspace relies exclusively on hermetic Python toolchains registered via `rules_python`:
  ```python
  bazel_dep(name = "rules_python", version = "0.31.0")

  python = use_extension("@rules_python//python/extensions:python.bzl", "python")
  python.toolchain(
      is_default = True,
      python_version = "3.11",
  )
  ```
  This guarantees an identical, isolated Python 3.11 runtime across development and evaluation environments.

## Prohibition on host cache scraping and environment pollution

In Intrinsic Core, the following host-coupled patterns are strictly prohibited:
- **No wheel scraping**: Never search `/var/cache/pip_cache` for pre-existing wheels (`.whl`). Scraping host caches circumvents Bazel's hermetic sandbox and introduces unversioned binary artifacts.
- **No `PIP_FIND_LINKS` injection**: Do not set `PIP_FIND_LINKS=/var/cache/pip_cache` or configure pip flags to point to host directories. Bazel's `rules_python` manages its own isolated artifact downloads.
- **No `PYTHONPATH` leaks**: Never set `PYTHONPATH` to point to `/usr/lib/python3/dist-packages` or any host system directory. Injecting `dist-packages` into the runtime path causes ABI mismatches, crashes the C-extension loader, and shadows hermetic Bazel packages.
- **No in-tree mock packages**: Never create local directories named `intrinsic/` containing stub `__init__.py` files. Python resolves local directories before external runfiles, causing local stubs to shadow genuine SDK packages.

## Four-stage `PyPI` dependency lifecycle

Third-party Python packages are managed through a deterministic four-stage lifecycle:

```
[ Stage 1: Declare abstract intent ]
  Add package names to requirements.in
                |
                v
[ Stage 2: Define compiler rule ]
  Define compile_pip_requirements in root BUILD
                |
                v
[ Stage 3: Compile deterministic lockfile ]
  Run: bazel run //:requirements.update
                |
                v
[ Stage 4: Consume in target graph ]
  Reference @pypi_deps//<pkg>:pkg in BUILD deps
```

### Stage 1: declaring abstract requirements in `requirements.in`

Create or edit `requirements.in` at the workspace root (adjacent to `MODULE.bazel`). Declare direct, abstract package dependencies without pinning internal sub-dependencies:

```text
numpy>=1.24.0,<2.0.0
opencv-python-headless
flask>=3.0.0
requests>=2.31.0
scipy>=1.11.0
```

*Rule*: Never add Intrinsic SDK packages (`intrinsic`, `ai_intrinsic_sdks`) to `requirements.in`. The Intrinsic SDK is a Bazel repository, not a PyPI distribution.

### Stage 2: configuring `compile_pip_requirements` in `BUILD`

Ensure the root `BUILD` (or `BUILD.bazel`) contains the `compile_pip_requirements` target:

```python
load("@rules_python//python:pip.bzl", "compile_pip_requirements")

compile_pip_requirements(
    name = "requirements",
    src = ":requirements.in",
    requirements_txt = ":requirements_lock.txt",
)
```

*File existence prerequisite*: `requirements_lock.txt` must exist on the filesystem before Bazel runs. If missing, create an empty file first:
```bash
touch requirements_lock.txt
```

### Stage 3: compiling the deterministic lockfile

Update the lockfile by executing the generated update target:

```bash
bazel run //:requirements.update
```

This hermetically invokes `pip-compile`, resolves all transitive dependencies, computes cryptographic wheel hashes, and writes a fully resolved `requirements_lock.txt`. Lockfile deduplication is handled entirely automatically by `rules_python`.

*Rule*: Never edit `requirements_lock.txt` manually. Always update it using `bazel run //:requirements.update`.

### Stage 4: consuming `PyPI` packages in target `BUILD` files

Reference external packages in target `deps` using the canonical hub prefix:
```python
@pypi_deps//<package_name>:pkg
```

#### Package name normalization

PyPI package names with hyphens normalize to underscores in Bazel labels:
- `opencv-python-headless` in `requirements.in` -> `@pypi_deps//opencv_python_headless:pkg` in `BUILD`
- `scikit-learn` in `requirements.in` -> `@pypi_deps//scikit_learn:pkg` in `BUILD`

#### Example `BUILD` configuration

```python
load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "image_processor",
    srcs = ["image_processor.py"],
    deps = [
        "@pypi_deps//numpy:pkg",
        "@pypi_deps//opencv_python_headless:pkg",
    ],
)

py_test(
    name = "image_processor_test",
    srcs = ["image_processor_test.py"],
    deps = [
        ":image_processor",
        "@pypi_deps//numpy:pkg",
    ],
)
```

In `image_processor.py`, use standard Python imports:
```python
import cv2
import numpy as np
```

## Intra-workspace code sharing

When sharing code between multiple packages within the same workspace, reuse code through explicit Bazel packages and visibility declarations rather than copying files or modifying `sys.path`:

1. **Create a library package**: Create a directory (e.g. `shared_utils`) adjacent to `MODULE.bazel` containing a `BUILD` file and implementation files (e.g. `shared_utils/math_helpers.py`).
2. **Define `py_library` target**: In `shared_utils/BUILD`, declare public visibility:
   ```python
   load("@rules_python//python:defs.bzl", "py_library")

   py_library(
       name = "math_helpers",
       srcs = ["math_helpers.py"],
       visibility = ["//visibility:public"],
   )
   ```
3. **Consume library in dependent targets**: In the consumer's `BUILD` file (e.g. `my_skill/BUILD`), add `//shared_utils:math_helpers` to `deps`:
   ```python
   py_library(
       name = "my_skill_py",
       srcs = ["my_skill.py"],
       deps = [
           "//shared_utils:math_helpers",
       ],
   )
   ```
4. **Import in Python**:
   ```python
   from shared_utils.math_helpers import compute_offset
   ```
