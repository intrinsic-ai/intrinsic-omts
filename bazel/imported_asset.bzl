"""Bazel rule to import a pre-packaged asset bundle into Flowstate solutions."""

load("@ioc//intrinsic/assets/build_defs:asset.bzl", "AssetInfo", "AssetLocalInfo")

def _imported_asset_bundle_impl(ctx):
    asset_info_output = ctx.actions.declare_file(ctx.label.name + ".asset_info.binpb")
    bundle = ctx.file.bundle

    local_info_args = ctx.actions.args().add(
        "--manifest",
        ctx.file.manifest,
    ).add(
        "--asset_type",
        ctx.attr.asset_type,
    ).add(
        "--output_asset_info",
        asset_info_output,
    )

    ctx.actions.run(
        inputs = [ctx.file.manifest],
        outputs = [asset_info_output],
        executable = ctx.executable._assetlocalinfogen,
        arguments = [local_info_args],
        mnemonic = "AssetLocalInfo",
        progress_message = "Writing asset info %{output} for %{label}",
    )

    return [
        DefaultInfo(
            files = depset([bundle]),
            runfiles = ctx.runfiles(files = [bundle]),
        ),
        AssetInfo(
            asset_info = asset_info_output,
            transitive_descriptor_sets = depset(),
        ),
        AssetLocalInfo(
            bundle_path = bundle,
        ),
    ]

imported_asset_bundle = rule(
    implementation = _imported_asset_bundle_impl,
    attrs = {
        "bundle": attr.label(
            allow_single_file = [".bundle.tar", ".tar"],
            mandatory = True,
            doc = "The pre-built bundle tarball file.",
        ),
        "manifest": attr.label(
            allow_single_file = [".textproto"],
            mandatory = True,
            doc = "Service/Asset manifest textproto matching the asset in the bundle.",
        ),
        "asset_type": attr.string(
            default = "ASSET_TYPE_SERVICE",
            doc = "Asset type string (e.g. ASSET_TYPE_SERVICE, ASSET_TYPE_HARDWARE_DEVICE, ASSET_TYPE_SKILL).",
        ),
        "_assetlocalinfogen": attr.label(
            default = Label("@ioc//intrinsic/assets/build_defs:assetlocalinfogen"),
            cfg = "exec",
            executable = True,
        ),
    },
    provides = [AssetInfo, AssetLocalInfo],
)
