load("@bazel_skylib//rules:common_settings.bzl", "string_flag")
load("@bazel_skylib//rules:copy_file.bzl", "copy_file")
load("@ioc//google3/intrinsic/assets/build_defs:asset.bzl", "intrinsic_asset_instance")
load("@ioc//google3/intrinsic/config:def.bzl", "intrinsic_solution")
load("@ioc//incode/intrinsic_inference/assets/inference_service/bazel:intrinsic_mlmodel.bzl", "intrinsic_mlmodel")
load("//bazel:imported_asset.bzl", "imported_asset_bundle")

package(default_visibility = ["//visibility:public"])

# Flag to parameterize hardware setup (omts vs lab_bb_01)
string_flag(
    name = "setup",
    build_setting_default = "omts",
    values = [
        "omts",
        "lab_bb_01",
    ],
)

config_setting(
    name = "is_lab_bb_01",
    flag_values = {
        ":setup": "lab_bb_01",
    },
)

# Solution deployment definition
intrinsic_solution(
    name = "omts_solution",
    add_compose_world_test = False,
    assets = [
        "@ioc//google3/intrinsic/resources/catalog/resourcedata/gripper:robotiq_pinch_gripper_resource_type",
        "@ioc//google3/intrinsic/resources/catalog/resourcedata/product/building_block",
        "@ioc//google3/intrinsic/resources/catalog/resourcedata/product/imts:raw_stock_2x3x5",
        "@ioc//google3/intrinsic/simulation/gazebo/asset:gazebo_simulator_type",
        "@ioc//google3/intrinsic/perception/calibration/charuco_boards:charuco_9x14_20mm_15mm_dict_5x5",
        "@ioc//google3/intrinsic/perception/calibration/charuco_boards:charuco_9x14_20mm_15mm_dict_5x5_estimator",
        "@ioc//google3/intrinsic/perception/service/v1:calibration_service",
        "@ioc//google3/intrinsic/perception/skills:capture_images_skill",
        "@ioc//google3/intrinsic/perception/skills/calibration:calibrate_camera_to_robot_skill",
        "@ioc//google3/intrinsic/perception/skills/calibration:sample_calibration_poses_skill",
        "@ioc//google3/intrinsic/perception/skills/calibration:collect_calibration_data_skill",
        "@ioc//google3/intrinsic/perception/skills/calibration:initialize_calibration_skill",
        "@ioc//incode/intrinsic_perception/intrinsic/perception/cameras/hardware_devices:orbbec_gemini_335le_hardware_device",
        "@ioc//incode/intrinsic_motion_planning/intrinsic/motion_planning/service:motion_planner_service_asset",
        "@ioc//incode/motion_planning/skills:clear_motion_planner_service_cache_skill",
        "@ioc//incode/motion_planning/skills:preplan_motion_skill",
        "@ioc//incode/motion_planning/skills:move_robot_skill",
        "@ioc//google3/intrinsic/icon/machines/common:generic_icon_mainloop_type",
        "@ioc//google3/intrinsic/manipulation/skills/force:move_to_contact_skill",
        "@ioc//google3/intrinsic/icon/skills:dio_read_input_skill",
        "@ioc//google3/intrinsic/icon/skills:dio_set_output_skill",
        "@ioc//incode/intrinsic_inference/assets/inference_service:inference_service_asset",
        "@ioc//incode/perception/ioc_pose_estimator:ioc_pose_estimator_service_asset",
        "@ioc//incode/perception/ioc_train_service:ioc_train_service_asset",
        "@ioc//google3/intrinsic/perception/skills/multi_view:estimate_pose_multi_view_skill",
        "@ioc//google3/intrinsic/world/skills/create_object:create_object_skill",
        "@ioc//google3/intrinsic/skills/apps:update_world_skill",
        "@ioc//google3/intrinsic/skills/apps:attach_object_to_robot_skill",
        "@ioc//google3/intrinsic/skills/apps:detach_object_skill",
        ":flowstate_ros_bridge_asset",
        ":orbbec_gemini_driver_asset",
        ":foundationpose_mlmodel",
        ":rfdetr_mlmodel",
        "//models/raw_stock_50x50x75",
    ] + select({
        ":is_lab_bb_01": [
            "@ioc//google3/intrinsic/apps/bluebird_caw/resources:caw_enclosure",
            "@ioc//google3/intrinsic/icon/hardware_modules/universal_robots:ur3e_hardware_module_ioc",
        ],
        "//conditions:default": [
            "@ioc//google3/intrinsic/icon/hardware_modules/universal_robots:ur5e_hardware_module_ioc",
            "//models/camera_mount",
            "//models/cnc_enclosure",
            "//models/omts_enclosure",
            "//models/schunk_egp_64nnb",
        ],
    }),
    default_operation_mode = "real",
    instances = [
        ":enclosure",
        ":robotiq_pinch_gripper",
        ":building_block",
        ":raw_stock_2x3x5",
        ":raw_stock_50x50x75",
        ":gazebo_simulator",
        ":charuco_9x14_20mm_15mm_dict_5x5",
        ":calibration_service_instance",
        ":icon",
        ":ur_module",
        ":orbbec_camera",
        ":orbbec_gemini_driver",
        ":motion_planner_service",
        ":inference_service",
        ":pose_estimator_service",
        ":train_service",
        ":flowstate_ros_bridge",
    ] + select({
        ":is_lab_bb_01": [
        ],
        "//conditions:default": [
            ":camera_mount",
            ":cnc_enclosure",
            ":schunk_egp_64nnb",
        ],
    }),
    object_world_updates = [
        "//configs:ur_module.attachments.updates.pbtxt",
        "//configs:scene.updates.pbtxt",
        "//configs:align_robot.updates.pbtxt",
    ] + select({
        ":is_lab_bb_01": [
            "//configs:lab_bb_01_orbbec_gemini.updates.pbtxt",
        ],
        "//conditions:default": [
            "//configs:cnc_enclosure.updates.pbtxt",
            "//configs:omts_camera_mount.updates.pbtxt",
            "//configs:schunk.updates.pbtxt",
        ],
    }),
)

# Default aliases
alias(
    name = "omts",
    actual = ":omts_solution",
)

alias(
    name = "omts_app",
    actual = "//src:omts_app",
)

intrinsic_asset_instance(
    name = "enclosure",
    asset = select({
        ":is_lab_bb_01": "ai.intrinsic.caw_enclosure",
        "//conditions:default": "ai.intrinsic.omts_enclosure",
    }),
    instance_name = "enclosure",
)

intrinsic_asset_instance(
    name = "cnc_enclosure",
    asset = "ai.intrinsic.cnc_enclosure",
)

intrinsic_asset_instance(
    name = "camera_mount",
    asset = "ai.intrinsic.camera_mount",
)

intrinsic_asset_instance(
    name = "robotiq_pinch_gripper",
    asset = "ai.intrinsic.robotiq_gripper",
    instance_name = "gripper",
)

intrinsic_asset_instance(
    name = "building_block",
    asset = "ai.intrinsic.building_block",
)

intrinsic_asset_instance(
    name = "raw_stock_2x3x5",
    asset = "ai.intrinsic.raw_stock_2x3x5",
)

intrinsic_asset_instance(
    name = "raw_stock_50x50x75",
    asset = "ai.intrinsic.raw_stock_50x50x75",
)

intrinsic_asset_instance(
    name = "charuco_9x14_20mm_15mm_dict_5x5",
    asset = "ai.intrinsic.charuco_9x14_20mm_15mm_dict_5x5",
)

intrinsic_asset_instance(
    name = "gazebo_simulator",
    asset = "ai.intrinsic.gazebo_simulator",
)

intrinsic_asset_instance(
    name = "calibration_service_instance",
    asset = "ai.intrinsic.calibration_service",
    instance_name = "calibration_service",
)

intrinsic_asset_instance(
    name = "icon",
    asset = "ai.intrinsic.generic_realtime_control_service",
    config = "//configs:icon_config.textproto",
    instance_name = "icon",
)

intrinsic_asset_instance(
    name = "ur_module",
    asset = select({
        ":is_lab_bb_01": "ai.intrinsic.ur3e_hardware_module_ioc",
        "//conditions:default": "ai.intrinsic.ur5e_hardware_module_ioc",
    }),
    config = "//configs:ur_module_config.textproto",
    instance_name = "ur_module",
)

intrinsic_asset_instance(
    name = "orbbec_camera",
    asset = "ai.intrinsic.orbbec_gemini_335le",
    config = select({
        ":is_lab_bb_01": "//configs:lab_bb_01_gemini_device_config.textproto",
        "//conditions:default": "//configs:omts_gemini_device_config.textproto",
    }),
    instance_name = "orbbec_camera",
)

intrinsic_asset_instance(
    name = "inference_service",
    asset = "ai.intrinsic.inference_service",
    instance_name = "inference_service",
)

intrinsic_asset_instance(
    name = "motion_planner_service",
    asset = "ai.intrinsic.motion_planner_service",
    instance_name = "motion_planner_service",
)

intrinsic_asset_instance(
    name = "pose_estimator_service",
    asset = "ai.intrinsic.ioc_pose_estimator_service",
    config = "//configs:pose_estimator_config.textproto",
    instance_name = "pose_estimator_service",
)

intrinsic_asset_instance(
    name = "train_service",
    asset = "ai.intrinsic.ioc_train_service",
    instance_name = "train_service",
)

imported_asset_bundle(
    name = "flowstate_ros_bridge_asset",
    bundle = "@flowstate_ros_bridge_bundle//:flowstate_ros_bridge.bundle.tar",
    manifest = "//configs:flowstate_ros_bridge_manifest.textproto",
)

intrinsic_asset_instance(
    name = "flowstate_ros_bridge",
    asset = "ai.intrinsic.flowstate_ros_bridge",
    instance_name = "flowstate_ros_bridge",
)

imported_asset_bundle(
    name = "orbbec_gemini_driver_asset",
    bundle = "@orbbec_gemini_driver_bundle//:orbbec_gemini_driver.bundle.tar",
    manifest = "//configs:orbbec_gemini_driver_manifest.textproto",
)

intrinsic_asset_instance(
    name = "orbbec_gemini_driver",
    asset = "ai.intrinsic.orbbec_gemini_driver",
    instance_name = "orbbec_gemini_driver",
)

intrinsic_asset_instance(
    name = "schunk_egp_64nnb",
    asset = "ai.intrinsic.schunk_egp_64nnb",
    instance_name = "schunk_egp_64nnb",
)

copy_file(
    name = "rfdetr_segmentation_config_pbtxt",
    src = "@segmentation_weights_bundle//:config.pbtxt",
    out = "rfdetr_segmentation_config.pbtxt",
    allow_symlink = True,
)

copy_file(
    name = "rfdetr_segmentation_onnx_file",
    src = "@segmentation_weights_bundle//:segmentation.onnx",
    out = "segmentation.onnx",
    allow_symlink = True,
)

intrinsic_mlmodel(
    name = "rfdetr_mlmodel",
    backend = "triton",
    config = ":rfdetr_segmentation_config_pbtxt",
    description = "RF-DETR segmentation model for object detection and segmentation.",
    display_name = "RF-DETR Segmentation Model",
    id = "ai.intrinsic.ioc_pose_estimation.segmentor.rfdetr",
    model_files = {
        ":rfdetr_segmentation_onnx_file": "1/segmentation.onnx",
    },
)

copy_file(
    name = "foundationpose_refine_onnx_file",
    src = "@foundationpose_refine_onnx//file",
    out = "foundationpose_refine.onnx",
    allow_symlink = True,
)

copy_file(
    name = "foundationpose_score_onnx_file",
    src = "@foundationpose_score_onnx//file",
    out = "foundationpose_score.onnx",
    allow_symlink = True,
)

copy_file(
    name = "foundationpose_py312_model_py",
    src = "@foundationpose_py312_bundle//:model.py",
    out = "foundationpose_py312_model.py",
    allow_symlink = True,
)

copy_file(
    name = "foundationpose_py312_cpp_so",
    src = "@foundationpose_py312_bundle//:foundationpose_cpp.so",
    out = "foundationpose_py312_foundationpose_cpp.so",
    allow_symlink = True,
)

copy_file(
    name = "foundationpose_py312_env_tar_gz",
    src = "@foundationpose_py312_bundle//:env.tar.gz",
    out = "foundationpose_py312_env.tar.gz",
    allow_symlink = True,
)

copy_file(
    name = "foundationpose_py312_config_pbtxt",
    src = "@foundationpose_py312_bundle//:config.pbtxt",
    out = "foundationpose_py312_config.pbtxt",
    allow_symlink = True,
)

intrinsic_mlmodel(
    name = "foundationpose_mlmodel",
    backend = "triton",
    config = ":foundationpose_py312_config_pbtxt",
    description = "FoundationPose 6D object pose estimation model.",
    display_name = "FoundationPose model weights",
    id = "ai.intrinsic.ioc_pose_estimation.pose_estimator.foundationpose",
    model_files = {
        ":foundationpose_refine_onnx_file": "1/foundationpose_refine.onnx",
        ":foundationpose_score_onnx_file": "1/foundationpose_score.onnx",
        ":foundationpose_py312_model_py": "1/model.py",
        ":foundationpose_py312_cpp_so": "1/foundationpose_cpp.so",
        ":foundationpose_py312_env_tar_gz": "env.tar.gz",
    },
)
