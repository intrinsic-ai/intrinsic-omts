"""Bazel rule for wrapping a binary with environment variables and runfiles.

Bazel's built-in mechanism to specify "args" and "env" is limited. They is only taken into account
when running via "bazel run" or "bazel test" but not when running the binary directly (e.g. via
`bazel-bin/...`) or calling it from other rules.

This implementation wraps the target binary with a shell script that takes care of correctly setting
the environment variables and runfiles. This ensures that "args" and "env" are also correctly
handled correctly when running the binary directly (e.g. via `bazel-bin/...`) or calling it from
other rules.

Furthermore, this rule allows to specify additional "arguments", "env" and "data" dependencies.
This can be useful when using a single test binary for multiple test targets with different
arguments, environment, or data dependencies.
"""

load("@bazel_lib//lib:paths.bzl", "BASH_RLOCATION_FUNCTION", "to_rlocation_path")

visibility(["//..."])

WRAPPER_HEADER = """#!/bin/bash
set -euo pipefail

{rlocation_function}

runfiles_export_envvars
""".format(rlocation_function = BASH_RLOCATION_FUNCTION)

# Workaround for https://github.com/bazelbuild/bazel/issues/26750.
_ArgsInfo = provider(
    doc = "Collects arguments from the target executable.",
    fields = {
        "args": "A list of arguments.",
        "deps": "A list of dependencies.",
    },
)

def _args_aspect_impl(_, ctx):
    args = []
    if hasattr(ctx.rule.attr, "args"):
        args = ctx.rule.attr.args

    deps = []
    if hasattr(ctx.rule.attr, "srcs"):
        deps += ctx.rule.attr.srcs
    if hasattr(ctx.rule.attr, "data"):
        deps += ctx.rule.attr.data
    if hasattr(ctx.rule.attr, "deps"):
        deps += ctx.rule.attr.deps

    return [
        _ArgsInfo(args = args, deps = deps),
    ]

_args_aspect = aspect(
    implementation = _args_aspect_impl,
)

def _wrapped_executable_impl(ctx):
    args = []
    if _ArgsInfo in ctx.attr.target:
        args += [
            ctx.expand_make_variables(
                "target",
                ctx.expand_location(arg, targets = ctx.attr.target[_ArgsInfo].deps),
                {},
            )
            for arg in ctx.attr.target[_ArgsInfo].args
        ]
    args += [
        ctx.expand_make_variables(
            "data",
            ctx.expand_location(arg, targets = ctx.attr.data),
            {},
        )
        for arg in ctx.attr.arguments
    ]

    env = ctx.attr.target[RunEnvironmentInfo].environment | {
        key: ctx.expand_make_variables(
            "env",
            ctx.expand_location(value, ctx.attr.data),
            {},
        )
        for key, value in ctx.attr.env.items()
    }

    wrapper = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(
        wrapper,
        content = """{header}
{exports}

exec "$(rlocation "{target}")" {args} "$@"
""".format(
            header = WRAPPER_HEADER,
            exports = "\n".join(['export %s="%s"' % (key, value) for key, value in env.items()]),
            target = to_rlocation_path(ctx, ctx.attr.target[DefaultInfo].files_to_run.executable),
            args = " ".join(['"%s"' % arg for arg in args]),
        ),
        is_executable = True,
    )

    default_info = DefaultInfo(
        executable = wrapper,
        files = depset(
            direct = [wrapper],
            transitive = [
                ctx.attr.target[DefaultInfo].files,
            ],
        ),
        runfiles = ctx.runfiles(
            files = [wrapper] + ctx.files.data,
        ).merge_all([
            ctx.attr._runfiles_dep[DefaultInfo].default_runfiles,
            ctx.attr.target[DefaultInfo].default_runfiles,
        ] + [
            target[DefaultInfo].default_runfiles
            for target in ctx.attr.data
        ]),
    )

    run_environment_info = RunEnvironmentInfo(
        environment = {},  # Added in the wrapper script.
        inherited_environment = ctx.attr.target[RunEnvironmentInfo].inherited_environment + ctx.attr.env_inherit,
    )

    return [
        default_info,
        run_environment_info,
    ]

def make_wrapped_executable_rule(extra_attrs = {}, **kwargs):
    return rule(
        _wrapped_executable_impl,
        attrs = {
            # NOTE: Unforunately, we can't override the built-in "args" attribute.
            "arguments": attr.string_list(),
            "data": attr.label_list(
                allow_files = True,
            ),
            "env": attr.string_dict(),
            "env_inherit": attr.string_list(),
            "target": attr.label(
                executable = True,
                cfg = "target",
                mandatory = True,
                aspects = [_args_aspect],
            ),
            "_runfiles_dep": attr.label(
                default = Label("@rules_shell//shell/runfiles"),
            ),
        } | extra_attrs,
        **kwargs
    )

wrapped_binary = make_wrapped_executable_rule(
    executable = True,
    doc = "A binary that is wrapped with environment variables and runfiles.",
)

wrapped_test = make_wrapped_executable_rule(
    test = True,
    doc = "A test that is wrapped with environment variables and runfiles.",
)
