load("@rules_license//rules:license.bzl", "license")
load("@rules_license//rules:package_info.bzl", "package_info")

package(
    default_applicable_licenses = [":license"],
    default_visibility = ["//visibility:public"],
)

license(
    name = "license",
    license_kinds = [
        "@rules_license//licenses/spdx:Apache-2.0",
    ],
    license_text = "LICENSE",
)

package_info(
    name = "package_info",
    package_name = "FoundationPose",
    package_url = "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation",
    package_version = "release-3.2",
)

exports_files([
    "config.pbtxt",
    "model.py",
])
