"""Macro for running a test against an onprem VM leased from a VM pool.

The macro leases a VM (which comes with the Intrinsic platform pre-installed),
logs in with `inctl`, runs the given test against it, and returns the VM. It:

  * uses whatever platform version the leased VM comes with;
  * expects an API key in the INTRINSIC_API_KEY environment variable.
"""

load("@rules_shell//shell:sh_test.bzl", "sh_test")
load("//private:wrapped_executable.bzl", "wrapped_test")

def _intrinsic_onprem_e2e_test_impl(
        name,
        visibility,
        test,
        inctl,
        org,
        pool,
        pool_type,
        timeout_minutes,
        env_inherit,
        _executor_script,
        _jq_toolchain):
    # Embed "args" and "env" to ensure they are passed to the wrapped test.
    wrapped_test(
        name = name + "_wrapped",
        target = test,
        tags = ["manual"],
        visibility = ["//visibility:private"],
    )
    wrapped = ":" + name + "_wrapped"

    sh_test(
        name = name,
        srcs = [_executor_script],
        data = [
            inctl,
            _jq_toolchain,
            wrapped,
        ],
        env = {
            "BINARY": "$(rlocationpath %s)" % wrapped,
            "E2E_ORG": org,
            "E2E_POOL": pool,
            "E2E_POOL_TYPE": pool_type,
            "INCTL": "$(rlocationpath %s)" % inctl,
            "JQ": "$(JQ_BIN)",
            "TIMEOUT": str(timeout_minutes * 60),
        },
        env_inherit = [
            "INTRINSIC_API_KEY",
            "INTRINSIC_ORG",
            "INTRINSIC_VMPOOL",
        ] + env_inherit,
        tags = [
            "manual",
            "requires-network",  # Overrides --sandbox_default_allow_network=false
        ],
        timeout = "eternal",
        size = "large",
        use_bash_launcher = True,
        visibility = visibility,
        toolchains = [_jq_toolchain],
    )

intrinsic_onprem_e2e_test = macro(
    doc = """Leases an onprem VM with the Intrinsic platform and runs a test against it.

    The following environment variables are read from the host:

      * `INTRINSIC_API_KEY` (required): API key used to log in with `inctl`.
      * `INTRINSIC_ORG`: organization (`org@project`) to lease the VM in, unless
        the `org` attribute is set.
      * `INTRINSIC_VMPOOL`: VM pool to lease from, unless the `pool` attribute is
        set. Defaults to `default`.

    The test can use the following environment variables:

      * `CLUSTER_NAME`: the name of the leased cluster.
      * `INTRINSIC_ORG`: the organization (`org@project`) to use.

    The test runs with an `inctl` login (written to `$HOME/.config/intrinsic`).

    Run manually as follows:

    ```
    INTRINSIC_API_KEY=... INTRINSIC_ORG=... bazel test --test_output=streamed //path/to:my_test
    ```
    """,
    implementation = _intrinsic_onprem_e2e_test_impl,
    attrs = {
        "env_inherit": attr.string_list(
            default = [],
            doc = "List of environment variables from the host to pass to the test.",
            configurable = False,
        ),
        "inctl": attr.label(
            mandatory = True,
            doc = "The inctl binary to use. Needs to support `auth login`, `vm lease` and `vm return`.",
            configurable = False,
        ),
        "org": attr.string(
            default = "",
            doc = "The organization (`org@project`) to lease the VM in. If empty, INTRINSIC_ORG from the host is used.",
            configurable = False,
        ),
        "pool": attr.string(
            default = "",
            doc = "The VM pool to lease from. If empty, INTRINSIC_VMPOOL from the host or `default` is used.",
            configurable = False,
        ),
        "pool_type": attr.string(
            default = "regular",
            values = ["regular", "gpu"],
            doc = "Indicates the type of VM underpinning the pool. `gpu` appends `-gpu` to the pool name.",
            configurable = False,
        ),
        "test": attr.label(
            mandatory = True,
            doc = "A _test rule.",
            configurable = False,
        ),
        "timeout_minutes": attr.int(
            default = 15,
            doc = "Timeout for running the test binary.",
            configurable = False,
        ),
        "_executor_script": attr.label(
            default = Label("//:vmpool_test_wrapper.sh"),
            configurable = False,
        ),
        "_jq_toolchain": attr.label(
            default = Label("@jq_toolchains//:resolved_toolchain"),
            configurable = False,
        ),
    },
)
