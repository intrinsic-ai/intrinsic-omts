load("@intrinsic-core//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")
load("@intrinsic-core//intrinsic_sdk/intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "banana_filegroup",
    srcs = ["banana.glb"],
)

sdf_scene_object(
    name = "banana_scene_object",
    src = "banana.sdf",
    sdf_assets = [":banana_filegroup"],
)

intrinsic_scene_object(
    name = "banana",
    manifest = ":banana_manifest.textproto",
    scene_object = ":banana_scene_object",
)
