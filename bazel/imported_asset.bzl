"""Bazel rule to import a pre-packaged asset bundle into Flowstate solutions."""

load("@intrinsic-core//intrinsic/assets/build_defs:asset.bzl", "AssetInfo", "AssetLocalInfo")

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

    inputs = [ctx.file.manifest]
    if ctx.file.file_descriptor_set:
        local_info_args.add(
            "--file_descriptor_set",
            ctx.file.file_descriptor_set,
        )
        inputs.append(ctx.file.file_descriptor_set)

    ctx.actions.run(
        inputs = inputs,
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
        "asset_type": attr.string(
            default = "ASSET_TYPE_SERVICE",
            doc = "Asset type string (e.g. ASSET_TYPE_SERVICE, ASSET_TYPE_HARDWARE_DEVICE, ASSET_TYPE_SKILL).",
        ),
        "bundle": attr.label(
            allow_single_file = [".bundle.tar", ".tar"],
            mandatory = True,
            doc = "The pre-built bundle tarball file.",
        ),
        "file_descriptor_set": attr.label(
            allow_single_file = [".binpb", ".pbbin", ".bin"],
            mandatory = False,
            doc = "Optional binary FileDescriptorSet proto for the asset.",
        ),
        "manifest": attr.label(
            allow_single_file = [".textproto", ".binpb"],
            mandatory = True,
            doc = "Manifest matching the asset in the bundle. assetlocalinfogen " +
                  "parses Service, HardwareDevice, SceneObject and Data manifests " +
                  "as textproto, but Skill and Process manifests as binary proto, " +
                  "so the latter must be passed as a .binpb (for example extracted " +
                  "from the bundle itself).",
        ),
        "_assetlocalinfogen": attr.label(
            default = Label("@intrinsic-core//intrinsic/assets/build_defs:assetlocalinfogen"),
            cfg = "exec",
            executable = True,
        ),
    },
    provides = [AssetInfo, AssetLocalInfo],
)
