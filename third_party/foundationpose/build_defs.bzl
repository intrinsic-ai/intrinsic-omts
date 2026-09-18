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

"""Custom build rules for FoundationPose Python 3.12 C++ extension and Triton environment."""

def _py312_transition_impl(_settings, _attr):
    return {"@rules_python//python/config_settings:python_version": "3.12"}

py312_transition = transition(
    implementation = _py312_transition_impl,
    inputs = [],
    outputs = ["@rules_python//python/config_settings:python_version"],
)

def _py312_file_impl(ctx):
    out = ctx.actions.declare_file(ctx.attr.out)
    src_file = ctx.files.src[0]
    ctx.actions.symlink(
        output = out,
        target_file = src_file,
    )
    return [DefaultInfo(files = depset([out]))]

py312_file = rule(
    implementation = _py312_file_impl,
    attrs = {
        "out": attr.string(mandatory = True),
        "src": attr.label(
            mandatory = True,
            allow_files = True,
            cfg = py312_transition,
        ),
        "_allowlist_function_transition": attr.label(
            default = "@bazel_tools//tools/allowlists/function_transition_allowlist",
        ),
    },
)

def _triton_python_env_tar_impl(ctx):
    out = ctx.actions.declare_file(ctx.attr.out)
    args = ctx.actions.args()
    args.add("--output", out)
    args.add("--patchelf", ctx.file.patchelf)
    args.add("--so-file", ctx.file.so_file)
    for whl in ctx.files.wheels:
        args.add("--wheel", whl)

    ctx.actions.run(
        inputs = [ctx.file.patchelf, ctx.file.so_file] + ctx.files.wheels,
        outputs = [out],
        executable = ctx.executable._builder,
        arguments = [args],
        mnemonic = "TritonPythonEnvTar",
        progress_message = "Building Triton Python 3.12 environment %s" % out.short_path,
    )
    return [DefaultInfo(files = depset([out]))]

triton_python_env_tar = rule(
    implementation = _triton_python_env_tar_impl,
    attrs = {
        "out": attr.string(mandatory = True),
        "patchelf": attr.label(
            mandatory = True,
            allow_single_file = True,
            cfg = "exec",
        ),
        "so_file": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
        "wheels": attr.label_list(
            mandatory = True,
            allow_files = [".whl"],
        ),
        "_builder": attr.label(
            default = "//third_party/foundationpose:build_triton_env",
            executable = True,
            cfg = "exec",
        ),
    },
)
