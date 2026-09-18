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
import gzip
import os
import shutil
import tarfile
import tempfile
import zipfile

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
command = /usr/bin/python3.12 -m venv /tmp/env_312
"""


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--output", required=True, help="Output env.tar.gz path")
  parser.add_argument(
    "--so-file", required=True, help="Path to foundationpose_cpp.so"
  )
  parser.add_argument(
    "--wheel", action="append", default=[], help="Wheel file paths"
  )
  args = parser.parse_args()

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
    os.chmod(activate_path, 0o755)

    os.symlink("/usr/bin/python3.12", os.path.join(bin_dir, "python3.12"))
    os.symlink("python3.12", os.path.join(bin_dir, "python3"))
    os.symlink("python3.12", os.path.join(bin_dir, "python"))

    # Extract all wheels into site-packages
    for whl in args.wheel:
      with zipfile.ZipFile(whl, "r") as zf:
        zf.extractall(site_packages_dir)

    # Copy foundationpose_cpp.so into site-packages
    target_so = os.path.join(site_packages_dir, "foundationpose_cpp.so")
    shutil.copyfile(so_path, target_so)
    os.chmod(target_so, 0o755)

    # Package into deterministic tar.gz
    def reset_tarinfo(tarinfo):
      tarinfo.uid = 0
      tarinfo.gid = 0
      tarinfo.uname = "root"
      tarinfo.gname = "root"
      tarinfo.mtime = 1704067200  # 2024-01-01 00:00:00 UTC
      if (
        tarinfo.isdir()
        or tarinfo.name.endswith(".so")
        or ".so." in tarinfo.name
        or tarinfo.name.startswith("bin/")
      ):
        tarinfo.mode = 0o755
      else:
        tarinfo.mode = 0o644
      return tarinfo

    with open(output_path, "wb") as f_out:
      with gzip.GzipFile(filename="", mode="wb", fileobj=f_out, mtime=0) as gz:
        with tarfile.open(
          fileobj=gz, mode="w", format=tarfile.GNU_FORMAT
        ) as tf:
          for item in sorted(os.listdir(stage_dir)):
            tf.add(
              os.path.join(stage_dir, item),
              arcname=item,
              filter=reset_tarinfo,
            )


if __name__ == "__main__":
  main()
