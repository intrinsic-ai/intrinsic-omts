# Patches mirrored from @intrinsic-core because Bzlmod only evaluates
# `single_version_override` / `archive_override` in the root module.
# See MODULE.bazel for the upstream BCR tracking issues and removal conditions.
exports_files([
    "gloop.patch",
    "llvm_header_search_paths.patch",
    # Remove once the upstream insrc PR switching default_config.textproto to
    # DEPTH_SOURCE_HARDWARE is merged and INTRINSIC_CORE_COMMIT is bumped.
    "nvidia_pose_estimator_depth_source.patch",
    "robotiq_hande_finger_offset.patch",
    "toybox_dynamic_link.patch",
])
