load("@intrinsic-core//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")
load("@intrinsic-core//intrinsic_sdk/intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "bracket_filegroup",
    srcs = ["bracket.glb"],
)

sdf_scene_object(
    name = "bracket_scene_object",
    src = "bracket.sdf",
    sdf_assets = [":bracket_filegroup"],
)

intrinsic_scene_object(
    name = "bracket",
    manifest = ":bracket_manifest.textproto",
    scene_object = ":bracket_scene_object",
)
