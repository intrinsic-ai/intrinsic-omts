#!/usr/bin/env python3
# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Builds a relocatable Triton Python 3.12 execution environment archive (env.tar.gz)."""

import argparse
import os
import shutil
import stat
import subprocess
import tarfile
import tempfile
import zipfile

RPATH_STR = (
  "$ORIGIN:$ORIGIN/..:$ORIGIN/../..:$ORIGIN/../../..:$ORIGIN/../../../..:$ORIGIN/../../../../..:"
  "$ORIGIN/numpy.libs:$ORIGIN/scipy.libs:$ORIGIN/opencv_python_headless.libs:$ORIGIN/pillow.libs:"
  "$ORIGIN/shapely.libs:$ORIGIN/h5py.libs:$ORIGIN/pyzmq.libs:$ORIGIN/simsimd.libs:"
  "$ORIGIN/../numpy.libs:$ORIGIN/../scipy.libs:$ORIGIN/../opencv_python_headless.libs:$ORIGIN/../pillow.libs:"
  "$ORIGIN/../shapely.libs:$ORIGIN/../pyzmq.libs:$ORIGIN/../simsimd.libs:"
  "$ORIGIN/../../numpy.libs:$ORIGIN/../../scipy.libs:$ORIGIN/../../opencv_python_headless.libs:$ORIGIN/../../pillow.libs:"
  "$ORIGIN/../../shapely.libs:$ORIGIN/../../pyzmq.libs:$ORIGIN/../../simsimd.libs:"
  "$ORIGIN/../../../numpy.libs:$ORIGIN/../../../scipy.libs:$ORIGIN/../../../opencv_python_headless.libs:$ORIGIN/../../../pillow.libs:"
  "$ORIGIN/../../../shapely.libs:$ORIGIN/../../../pyzmq.libs:$ORIGIN/../../../simsimd.libs:"
  "$ORIGIN/../../../../numpy.libs:$ORIGIN/../../../../scipy.libs:$ORIGIN/../../../../opencv_python_headless.libs:$ORIGIN/../../../../pillow.libs:"
  "$ORIGIN/../../../../shapely.libs:$ORIGIN/../../../../pyzmq.libs:$ORIGIN/../../../../simsimd.libs:"
  "$ORIGIN/../../../../../numpy.libs:$ORIGIN/../../../../../scipy.libs:$ORIGIN/../../../../../opencv_python_headless.libs:$ORIGIN/../../../../../pillow.libs:"
  "$ORIGIN/../../../../../shapely.libs:$ORIGIN/../../../../../pyzmq.libs:$ORIGIN/../../../../../simsimd.libs"
)

ACTIVATE_CONTENT = """# This file must be used with "source bin/activate" *from bash*
# You cannot run it directly

deactivate () {
    if [ -n "${_OLD_VIRTUAL_PATH:-}" ] ; then
        PATH="${_OLD_VIRTUAL_PATH:-}"
        export PATH
        unset _OLD_VIRTUAL_PATH
    fi
    if [ -n "${_OLD_VIRTUAL_PYTHONHOME:-}" ] ; then
        PYTHONHOME="${_OLD_VIRTUAL_PYTHONHOME:-}"
        export PYTHONHOME
        unset _OLD_VIRTUAL_PYTHONHOME
    fi
    hash -r 2> /dev/null
    if [ -n "${_OLD_VIRTUAL_PS1:-}" ] ; then
        PS1="${_OLD_VIRTUAL_PS1:-}"
        export PS1
        unset _OLD_VIRTUAL_PS1
    fi
    unset VIRTUAL_ENV
    unset VIRTUAL_ENV_PROMPT
    if [ ! "${1:-}" = "nondestructive" ] ; then
        unset -f deactivate
    fi
}

deactivate nondestructive

_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export VIRTUAL_ENV="${_ENV_DIR}"

_OLD_VIRTUAL_PATH="$PATH"
PATH="$VIRTUAL_ENV/bin:$PATH"
export PATH

if [ -n "${PYTHONHOME:-}" ] ; then
    _OLD_VIRTUAL_PYTHONHOME="${PYTHONHOME:-}"
    unset PYTHONHOME
fi

if [ -z "${VIRTUAL_ENV_DISABLE_PROMPT:-}" ] ; then
    _OLD_VIRTUAL_PS1="${PS1:-}"
    PS1='(env_312) '"${PS1:-}"
    export PS1
    VIRTUAL_ENV_PROMPT='(env_312) '
    export VIRTUAL_ENV_PROMPT
fi

hash -r 2> /dev/null
export PYTHONPATH="${_ENV_DIR}/lib/python3.12/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${_ENV_DIR}/lib:${_ENV_DIR}/lib/python3.12/site-packages:${_ENV_DIR}/lib/python3.12/site-packages/numpy.libs:${_ENV_DIR}/lib/python3.12/site-packages/scipy.libs:${_ENV_DIR}/lib/python3.12/site-packages/opencv_python_headless.libs:${_ENV_DIR}/lib/python3.12/site-packages/pillow.libs:${LD_LIBRARY_PATH:-}"
"""

PYVENV_CFG_CONTENT = """home = /usr/bin
include-system-site-packages = false
version = 3.12.3
executable = /usr/bin/python3.12
command = /usr/bin/python3 -m venv /tmp/env_312
"""


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--output", required=True, help="Output env.tar.gz path")
  parser.add_argument(
    "--patchelf", required=True, help="Path to patchelf binary"
  )
  parser.add_argument(
    "--so-file", required=True, help="Path to foundationpose_cpp.so"
  )
  parser.add_argument(
    "--wheel", action="append", default=[], help="Wheel file paths"
  )
  args = parser.parse_args()

  patchelf_path = os.path.abspath(args.patchelf)
  output_path = os.path.abspath(args.output)
  so_path = os.path.abspath(args.so_file)

  with tempfile.TemporaryDirectory() as stage_dir:
    bin_dir = os.path.join(stage_dir, "bin")
    include_dir = os.path.join(stage_dir, "include", "python3.12")
    site_packages_dir = os.path.join(
      stage_dir, "lib", "python3.12", "site-packages"
    )
    os.makedirs(bin_dir, exist_ok=True)
    os.makedirs(include_dir, exist_ok=True)
    os.makedirs(site_packages_dir, exist_ok=True)

    # Create lib64 -> lib symlink
    os.symlink("lib", os.path.join(stage_dir, "lib64"))

    # Write pyvenv.cfg
    with open(os.path.join(stage_dir, "pyvenv.cfg"), "w") as f:
      f.write(PYVENV_CFG_CONTENT)

    # Write bin/activate and python symlinks
    activate_path = os.path.join(bin_dir, "activate")
    with open(activate_path, "w") as f:
      f.write(ACTIVATE_CONTENT)
    os.chmod(activate_path, 0o644)

    os.symlink("/usr/bin/python3", os.path.join(bin_dir, "python3"))
    os.symlink("python3", os.path.join(bin_dir, "python"))
    os.symlink("python3", os.path.join(bin_dir, "python3.12"))

    # Extract all wheels into site-packages
    for whl in args.wheel:
      with zipfile.ZipFile(whl, "r") as zf:
        zf.extractall(site_packages_dir)

    # Copy foundationpose_cpp.so into site-packages
    target_so = os.path.join(site_packages_dir, "foundationpose_cpp.so")
    shutil.copyfile(so_path, target_so)
    os.chmod(target_so, 0o755)

    # Apply RPATH using patchelf to all shared objects in site-packages
    for root, _, files in os.walk(site_packages_dir):
      for fname in files:
        if ".so" in fname:
          fpath = os.path.join(root, fname)
          if os.path.islink(fpath):
            continue
          os.chmod(fpath, os.stat(fpath).st_mode | stat.S_IWUSR | stat.S_IXUSR)
          subprocess.run(
            [patchelf_path, "--set-rpath", RPATH_STR, fpath],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
          )

    # Package into deterministic tar.gz
    def reset_tarinfo(tarinfo):
      tarinfo.uid = 0
      tarinfo.gid = 0
      tarinfo.uname = "root"
      tarinfo.gname = "root"
      tarinfo.mtime = 1704067200  # 2024-01-01 00:00:00 UTC
      return tarinfo

    with tarfile.open(output_path, "w:gz", format=tarfile.GNU_FORMAT) as tf:
      for item in sorted(os.listdir(stage_dir)):
        tf.add(
          os.path.join(stage_dir, item), arcname=item, filter=reset_tarinfo
        )


if __name__ == "__main__":
  main()
