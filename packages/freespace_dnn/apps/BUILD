load("//engine/build:isaac.bzl", "isaac_app", "isaac_subgraph")

isaac_app(
    name = "autonomous_freespace_annotation",
    data = [
        "//apps/assets/maps",
        "//apps/carter:2d_carter_subgraph",
        "//apps/carter/robots:robots",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/superpixels/apps:superpixels_subgraph",
    ],
    modules = [
        "imu",
        "navigation",
        "perception",
        "planner",
        "realsense",
        "rgbd_processing",
        "segway",
        "sensors:joystick",
        "superpixels",
        "velodyne_lidar",
        "viewers",
    ],
    script = "select_json.sh",
)

isaac_app(
    name = "path_segmentation_inference",
    data = [
        "//packages/freespace_dnn/apps:segmentation_subgraph",
        "@path_segmentation_pretrained_models",
    ],
    modules = [
        "freespace_dnn",
        "ml",
        "navigation",
        "segway",
        "sensors:joystick",
        "zed",
        "viewers",
    ],
)

isaac_subgraph(
    name = "segmentation_subgraph",
    modules = [
        "ml",
        "ml:tensorflow",
    ],
    subgraph = "path_segmentation_inference.subgraph.json",
    visibility = ["//visibility:public"],
)