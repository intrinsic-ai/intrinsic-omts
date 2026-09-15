load("@ioc//google3/intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@ioc//google3/intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "raw_stock_2x3x5_filegroup",
    srcs = ["base_visual.glb"],
)

sdf_scene_object(
    name = "raw_stock_2x3x5_scene_object",
    src = "raw_stock_2x3x5.sdf",
    sdf_assets = [":raw_stock_2x3x5_filegroup"],
)

intrinsic_scene_object(
    name = "raw_stock_2x3x5",
    manifest = ":raw_stock_2x3x5_manifest.textproto",
    scene_object = ":raw_stock_2x3x5_scene_object",
)
