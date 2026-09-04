load("@ioc//google3/intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@ioc//google3/intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "raw_stock_50x50x75_filegroup",
    srcs = ["raw_stock_50x50x75.glb"],
)

sdf_scene_object(
    name = "raw_stock_50x50x75_scene_object",
    src = "raw_stock_50x50x75.sdf",
    sdf_assets = [":raw_stock_50x50x75_filegroup"],
)

intrinsic_scene_object(
    name = "raw_stock_50x50x75",
    manifest = ":raw_stock_50x50x75_manifest.textproto",
    scene_object = ":raw_stock_50x50x75_scene_object",
)
