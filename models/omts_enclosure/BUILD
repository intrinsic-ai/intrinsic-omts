load("@ioc//incode/intrinsic_sdk/intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@ioc//incode/intrinsic_sdk/intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "omts_enclosure_filegroup",
    srcs = ["omts_enclosure.glb"],
)

sdf_scene_object(
    name = "omts_enclosure_scene_object",
    src = "omts_enclosure.sdf",
    sdf_assets = [":omts_enclosure_filegroup"],
)

intrinsic_scene_object(
    name = "omts_enclosure",
    manifest = ":omts_enclosure_manifest.textproto",
    scene_object = ":omts_enclosure_scene_object",
)
