load("@ioc//incode/intrinsic_sdk/intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@ioc//incode/intrinsic_sdk/intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "camera_mount_filegroup",
    srcs = ["camera_mount.glb"],
)

sdf_scene_object(
    name = "camera_mount_scene_object",
    src = "camera_mount.sdf",
    sdf_assets = [":camera_mount_filegroup"],
)

intrinsic_scene_object(
    name = "camera_mount",
    manifest = ":camera_mount_manifest.textproto",
    scene_object = ":camera_mount_scene_object",
)
