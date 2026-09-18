load("@ioc//incode/intrinsic_sdk/intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@ioc//incode/intrinsic_sdk/intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "schunk_egp_64nnb_filegroup",
    srcs = [
        "base_visual.glb",
        "jaw_l_visual.glb",
        "jaw_r_visual.glb",
    ],
)

sdf_scene_object(
    name = "schunk_egp_64nnb_scene_object",
    src = "schunk_egp_64nnb.sdf",
    sdf_assets = [":schunk_egp_64nnb_filegroup"],
)

intrinsic_scene_object(
    name = "schunk_egp_64nnb",
    manifest = ":schunk_egp_64nnb_manifest.textproto",
    scene_object = ":schunk_egp_64nnb_scene_object",
)
