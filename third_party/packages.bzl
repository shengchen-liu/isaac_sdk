"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_git_repository", "isaac_new_git_repository")
load("//engine/build:isaac.bzl", "isaac_http_archive", "isaac_new_http_archive")

def clean_dep(dep):
    return str(Label(dep))

# loads dependencies for various modules
def isaac_packages_workspace():
    isaac_new_http_archive(
        name = "robotis",
        build_file = clean_dep("//third_party:dynamixel.BUILD"),
        sha256 = "1233525218b59ee9b923124ca688feab7014362c1c9c7ad4a844927f8ec3dba5",
        url = "https://developer.nvidia.com/isaac/download/third_party/robotis_dynamixel_sdk-3-6-2-tar-gz",
        type = "tar.gz",
        strip_prefix = "DynamixelSDK-3.6.2",
        licenses = ["@robotis//:LICENSE"],
    )

    isaac_new_http_archive(
        name = "assimp",
        build_file = clean_dep("//third_party:assimp.BUILD"),
        sha256 = "60080d8ab4daaab309f65b3cffd99f19eb1af8d05623fff469b9b652818e286e",
        url = "https://developer.nvidia.com/isaac/download/third_party/assimp-4-0-1-tar-gz",
        type = "tar.gz",
        strip_prefix = "assimp-4.0.1",
        licenses = ["@assimp//:LICENSE"],
    )

    isaac_new_http_archive(
        name = "nvstereomatcher",
        build_file = clean_dep("//third_party:nvstereomatcher.BUILD"),
        sha256 = "c4db7e3641d32f370394570181c5f85fc33667a195c5de7f6bef8d4194e315af",
        url = "https://developer.nvidia.com/isaac/download/third_party/libnvstereomatcher_v5-tar-gz",
        type = "tar.gz",
        strip_prefix = "libnvstereomatcher_v5",
        licenses = ["//:LICENSE"],
    )

    isaac_new_http_archive(
        name = "apriltags",
        build_file = clean_dep("//third_party:apriltags.BUILD"),
        sha256 = "5cfc4bc73fccc4d737685e345060e98e2ce067ac41026f4a1e212b402e6c33c0",
        url = "https://developer.nvidia.com/isaac/download/third_party/april_tag_v5-tar-gz",
        type = "tar.gz",
        strip_prefix = "libapriltagging_v5",
        licenses = ["//:LICENSE"],
    )

    isaac_git_repository(
        name = "gmapping_repo",
        remote = "https://github.com/lullabee/openslam_gmapping.git",
        commit = "574e40a7b9a2cd529e8c9ba4802c9f849923251d",
        licenses = ["https://openslam-org.github.io/gmapping.html"],
    )

    native.bind(
        name = "gmapping",
        actual = "@gmapping_repo//:gmapping",
    )

    isaac_http_archive(
        name = "audio_assets",
        sha256 = "3915240ad6c5fe50f50a84204b6d9b602505f2558fad4c14b27187266f458b25",
        url = "https://developer.nvidia.com/isaac/download/third_party/alsa_audio_assets-v2-tar-gz",
        type = "tar.gz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "torch_inference_test_assets",
        build_file = clean_dep("//third_party:torch_inference_test_assets.BUILD"),
        sha256 = "24e10fbb2aae938b9dcfbaa8ceb1fc65b31de33733159c23a1e1d3c545cb8322",
        url = "https://developer.nvidia.com/isaac/download/third_party/torch_inference_test_assets-v2-tar-gz",
        type = "tar.gz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "ball_segmentation_model",
        build_file = clean_dep("//third_party:ball_segmentation_model.BUILD"),
        sha256 = "b0b01e06a0b02f316748d664c9b07b1dbd0ea70dc692262d9df2ac1fdbd22ddd",
        url = "https://developer.nvidia.com/isaac/download/third_party/ball_segmentation_toy_model-20190626-tar-xz",
        type = "tar.xz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_new_http_archive(
        name = "voice_command_detection_model_carter",
        build_file = clean_dep("//third_party:voice_command_detection_model_carter.BUILD"),
        sha256 = "57e1b0f70136f7008b467d02eb97d8f09da45e85ca6a8cb442aca9ea2f3d7b55",
        url = "https://developer.nvidia.com/isaac/download/third_party/vcd_model_carter_v1-tar-gz",
        type = "tar.gz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_new_http_archive(
        name = "voice_command_detection_model_kaya",
        build_file = clean_dep("//third_party:voice_command_detection_model_kaya.BUILD"),
        sha256 = "80a8251c81735c88573e17933f553da2aead04771fea2dee76348eddc85d426d",
        url = "https://developer.nvidia.com/isaac/download/third_party/vcd_model_kaya_v1-tar-gz",
        type = "tar.gz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_new_http_archive(
        name = "alsa",
        build_file = clean_dep("//third_party:alsa.BUILD"),
        sha256 = "938832b91e5ac8c4aee9847561f680814d199ba5ad9fb795c5a699075a19fd61",
        url = "https://developer.nvidia.com/isaac/download/third_party/alsa-x86_64-tar-xz",
        type = "tar.xz",
        licenses = ["https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html"],
    )

    # Used for both TX2 and Xavier
    isaac_new_http_archive(
        name = "alsa_aarch64",
        build_file = clean_dep("//third_party:alsa.BUILD"),
        sha256 = "8b0b1f65bc7fbdf45c30389457c530c423518dd12b32cdddca704bfd0daf0ec9",
        url = "https://developer.nvidia.com/isaac/download/third_party/alsa-aarch64-tar-xz",
        type = "tar.xz",
        licenses = ["https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html"],
    )

    isaac_new_http_archive(
        name = "libi2c_aarch64",
        build_file = clean_dep("//third_party:libi2c.BUILD"),
        sha256 = "0371eb3a1f60a5515f5571e5fc07711eb5d82f575060096fae09dcd0821b7d39",
        url = "https://developer.nvidia.com/isaac/download/third_party/libi2c-0-aarch64_xavier-tar-xz",
        type = "tar.xz",
        strip_prefix = "libi2c",
        licenses = ["https://raw.githubusercontent.com/amaork/libi2c/master/LICENSE"],
    )

    isaac_new_http_archive(
        name = "vrworks_warp360",
        build_file = clean_dep("//third_party:warp360.BUILD"),
        sha256 = "48225cc6bae5a50f342998cd7bde5015f3402f7371d3c3c8deda23921171d532",
        url = "https://developer.nvidia.com/isaac/download/third_party/vrworks_warp360-3-tar-gz",
        type = "tar.gz",
        licenses = ["//:LICENSE"],
    )

    isaac_new_http_archive(
        name = "libargus",
        build_file = clean_dep("//third_party:libargus.BUILD"),
        sha256 = "8db8df094efb31a6c945e69001d6f9de3ccdbe8752d4069ef05c79c14ec0af5b",
        url = "https://developer.nvidia.com/isaac/download/third_party/libargus-2019-02-tar-gz",
        type = "tar.gz",
        strip_prefix = "libargus",
        licenses = ["https://raw.githubusercontent.com/pauldotknopf/JetsonTX1Drivers/master/nv_tegra/LICENSE.libargus"],
    )

    isaac_new_http_archive(
        name = "vicon_datastream",
        build_file = clean_dep("//third_party:vicon_datastream.BUILD"),
        sha256 = "f8e0d88ad53a99e3ef4de21891781c664fb333a7e656967fd1d4230d7538371e",
        url = "https://developer.nvidia.com/isaac/download/third_party/vicon-datastream-sdk-tar-gz",
        type = "tar.gz",
        licenses = ["https://www.vicon.com/products/software/datastream-sdk"],
    )

    isaac_new_http_archive(
        name = "elbrus_vo",
        build_file = clean_dep("//third_party:elbrus_vo.BUILD"),
        sha256 = "4db528fb11f796a45a76e089aca025d62cc790b22cb9d6004e7dacc6a31cd83e",
        url = "https://developer.nvidia.com/isaac/download/third_party/elbrus_v4_5-tar-xz",
        type = "tar.xz",
        strip_prefix = "elbrus",
        licenses = ["//:LICENSE"],
    )

    isaac_new_http_archive(
        name = "kinova_jaco",
        build_file = clean_dep("//third_party:kinova_jaco.BUILD"),
        sha256 = "a8fa1a09ec98a69ab508176c35e582ed41abb63da430c73b8371940c68739fdd",
        url = "https://developer.nvidia.com/isaac/download/third_party/kinova_ros-1-2-1-tar-gz",
        type = "tar.gz",
        strip_prefix = "kinova-ros-1.2.1",
        licenses = ["https://raw.githubusercontent.com/Kinovarobotics/kinova-ros/master/LICENSE"],
    )

    isaac_new_http_archive(
        name = "realsense",
        build_file = clean_dep("//third_party:realsense.BUILD"),
        sha256 = "fe8fb031d6a2b45cb563a44b142cbf5d669cd229a325567a0f1350ccfea89e3c",
        url = "https://developer.nvidia.com/isaac/download/third_party/librealsense-2-17-0-tar-gz",
        type = "tar.gz",
        strip_prefix = "librealsense-2.17.0",
        licenses = ["@realsense//:LICENSE"],
    )

    isaac_new_http_archive(
        name = "opencv_x86_64",
        build_file = clean_dep("//third_party:opencv.BUILD"),
        sha256 = "364b6004167c9ac614fc4051a768777cad3fbaf71cbd486a31540d8195db6a98",
        url = "https://developer.nvidia.com/isaac/download/third_party/opencv-3-3-1-x86_64-18-20190327c-tgz",
        type = "tgz",
        licenses = ["https://opencv.org/license.html"],
    )

    isaac_new_http_archive(
        name = "opencv_aarch64_jetpack42",
        build_file = clean_dep("//third_party:opencv_jetpack42.BUILD"),
        sha256 = "f50be71f870a5e064b859fd110a73eb075b08b6365847d2fc19c59c2bdebef91",
        url = "https://developer.nvidia.com/isaac/download/third_party/opencv_jetpack_4_2_b150_aarch64_nano-tar-xz",
        type = "tar.xz",
        licenses = ["https://opencv.org/license.html"],
    )

    isaac_new_http_archive(
        name = "libtensorflow_x86_64",
        build_file = clean_dep("//third_party:libtensorflow_x86_64.BUILD"),
        sha256 = "6caa54f6f12b6de8b95dfe0689989e07791e8a5cae2e595bd95cd3c3ab42566e",
        url = "https://developer.nvidia.com/isaac/download/third_party/libtensorflow_1_13_1_cuda_10_0_cudnn_7_4_2_x86_64_20190326-tar-gz",
        type = "tar.gz",
        licenses = ["https://raw.githubusercontent.com/tensorflow/tensorflow/master/LICENSE"],
    )

    isaac_new_http_archive(
        name = "libtensorflow_aarch64_jetpack42",
        build_file = clean_dep("//third_party:libtensorflow_aarch64_jetpack42.BUILD"),
        sha256 = "8652447d82da4aae4c8fc77c4b75c5189577a11c220013d0cee4239ef2f0336e",
        url = "https://developer.nvidia.com/isaac/download/third_party/libtensorflow_1_13_1_aarch64_jetpack42_20190327-tar-gz",
        type = "tar.gz",
        licenses = ["https://raw.githubusercontent.com/tensorflow/tensorflow/master/LICENSE"],
    )

    # libtorch for x86_64
    isaac_new_http_archive(
        name = "libtorch_x86_64",
        build_file = clean_dep("//third_party:libtorch_x86_64.BUILD"),
        sha256 = "203ffa86773e5e061ff249345012a23c8acc3feb3f68b93bd2aecbd9ba41c4ae",
        url = "https://developer.nvidia.com/isaac/download/third_party/libtorch_x86_64_1-1-0-v2-tar-xz",
        type = "tar.xz",
        licenses = ["https://github.com/pytorch/pytorch/blob/master/LICENSE"],
    )

    # libtorch for aarch64_jetpack42
    isaac_new_http_archive(
        name = "libtorch_aarch64_jetpack42",
        build_file = clean_dep("//third_party:libtorch_aarch64_jetpack42.BUILD"),
        sha256 = "3a66d995cd0b7254e82549edc1c09d2a5562f0fe186bb69c5855e3da7ab9f7d0",
        url = "https://developer.nvidia.com/isaac/download/third_party/libtorch_aarch64_jetpack42_1-1-0-v0-tar-gz",
        type = "tar.gz",
        licenses = ["https://github.com/pytorch/pytorch/blob/master/LICENSE"],
    )

    isaac_new_http_archive(
        name = "mobilenetv2",
        build_file = clean_dep("//third_party:mobilenetv2.BUILD"),
        sha256 = "36bb48816a00d123299685382dc9e57ef743092b11211e3b0dd5fb47a0710fcd",
        url = "https://developer.nvidia.com/isaac/download/third_party/mobilenetv2-1-4-224-v1-tgz",
        type = "tgz",
        licenses = ["https://raw.githubusercontent.com/tensorflow/models/master/research/slim/nets/mobilenet/mobilenet.py"],
    )

    isaac_new_http_archive(
        name = "mobilenetv2_onnx",
        build_file = clean_dep("//third_party:mobilenetv2_onnx.BUILD"),
        sha256 = "8ce2930074b6025c141fcfee9e2c63bb7183f5f19e27695931ce763956cab098",
        url = "https://rdk-public.s3.amazonaws.com/test_data/mobilenetv2-1_0_onnx.tar.xz",
        type = "tar.xz",
        licenses = ["https://raw.githubusercontent.com/onnx/models/master/models/image_classification/mobilenet/README.md"],
    )

    isaac_new_http_archive(
        name = "ml_test_data",
        build_file = clean_dep("//third_party:ml_test_data.BUILD"),
        sha256 = "2916fe0330ed1c2392148fe1ba8f8353ae3b694aa1c50d28d8f3df8f318ad57e",
        url = "https://developer.nvidia.com/isaac/download/third_party/ml_test_data_1_3-tar-xz",
        type = "tar.xz",
        licenses = ["//:LICENSE"],
    )

    # As of now the public TensorRT version for x64 is 4.0.1.6. Jetpack 3.3 comes with 4.1.3, which
    # is not yet available for x64
    isaac_new_http_archive(
        name = "tensorrt_x86_64",
        build_file = clean_dep("//third_party:tensorrt_x86_64.BUILD"),
        sha256 = "7b8e803e296858f396e21b59f236bfdf9ba9de5f3f0d0af55f3a1dab2a324e81",
        url = "https://developer.nvidia.com/isaac/download/third_party/tensorrt_5-0-2_x86_64-tar-xz",
        strip_prefix = "tensorrt_dynamic",
        type = "tar.xz",
        licenses = ["https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html"],
    )

    # From Jetpack 4.2 b150
    isaac_new_http_archive(
        name = "tensorrt_aarch64_jetpack42",
        build_file = clean_dep("//third_party:tensorrt_jetpack42.BUILD"),
        sha256 = "60b77aecff5160044ed4e7a0f47ff65ec211ddc6a5dd1acf34119f96b4cfc1fd",
        url = "https://developer.nvidia.com/isaac/download/third_party/tensorrt_jetpack_4_2_b150_aarch64_nano-tar-xz",
        type = "tar.xz",
        licenses = ["https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html"],
    )

    isaac_new_http_archive(
        name = "yolo_pretrained_models",
        build_file = clean_dep("//third_party:yolo_pretrained_models.BUILD"),
        sha256 = "30e674bcc7e2de4ac32ce815bf835126b4b48c01dde03bf7538404f50c47e606",
        url = "https://developer.nvidia.com/isaac/download/third_party/yolo_pretrained_models-tar-xz",
        type = "tar.xz",
        licenses = [
            "https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html",
            "https://raw.githubusercontent.com/pjreddie/darknet/master/LICENSE",
        ],
    )

    isaac_new_http_archive(
        name = "yolo_tensorrt_test_data",
        build_file = clean_dep("//third_party:yolo_tensorrt_test_data.BUILD"),
        sha256 = "917b720f579ea392bdc6ebe063e50faf702a494b4c5ab1ef7071b572463ee35e",
        url = "https://developer.nvidia.com/isaac/download/third_party/yolo_tensorrt_test_data-2018-12-tar-gz",
        type = "tar.gz",
        strip_prefix = "yolo_tensorrt_test_data_v2",
        licenses = [
            "https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html",
            "https://raw.githubusercontent.com/pjreddie/darknet/master/LICENSE",
        ],
    )

    isaac_new_http_archive(
        name = "yolo_tensorrt_lib",
        build_file = clean_dep("//third_party:yolo_tensorrt_lib.BUILD"),
        sha256 = "92048f7b0caaa0d30b503c8afd9696469f4545d1469487f81544caf196b47b51",
        url = "https://developer.nvidia.com/isaac/download/third_party/yolo_library-20190326b-tar-gz",
        type = "tar.gz",
        strip_prefix = "yolo_library_20190326",
        licenses = [
            "@yolo_tensorrt_lib//:LICENSE.md",
            "https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html",
            "https://github.com/pjreddie/darknet/blob/master/LICENSE",
        ],
    )

    isaac_new_http_archive(
        name = "yolo_tensorrt_test_data_gtc2019_demo",
        build_file = clean_dep("//third_party:yolo_tensorrt_test_data_gtc2019_demo.BUILD"),
        sha256 = "dea69b1aa03356a383dcebb681849e05c4647afb1ec85276de9a8f7818cbac40",
        url = "https://developer.nvidia.com/isaac/download/third_party/yolo_tensorrt_test_data_gtc2019_demo_v2-tar-xz",
        type = "tar.xz",
        licenses = [
            "https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html",
            "https://raw.githubusercontent.com/pjreddie/darknet/master/LICENSE",
        ],
    )

    isaac_new_http_archive(
        name = "redtail",
        build_file = clean_dep("//third_party:redtail.BUILD"),
        sha256 = "a25fa2b181606f781220fcc22945ddb483d9d00fe093f113c4e79abb3e556013",
        url = "https://developer.nvidia.com/isaac/download/third_party/redtail-20190625-cc2745047cf5a0964bdd3a38fc8e851491e48e75-zip",
        type = "zip",
        strip_prefix = "redtail-cc2745047cf5a0964bdd3a38fc8e851491e48e75",
        licenses = ["@redtail//:LICENSE.md"],
    )

    isaac_new_http_archive(
        name = "tacotron2_model",
        build_file = clean_dep("//third_party:tacotron2_model.BUILD"),
        sha256 = "ffb88e4734700521925fec5926a4b29336b261368f1b02d2d61f5bb3f6d95d40",
        url = "https://developer.nvidia.com/isaac/download/third_party/tacotron2_streaming_fp32-v1-tar-gz",
        type = "tar.gz",
        licenses = ["//:LICENSE"],
    )

    isaac_new_http_archive(
        name = "waveglow_model",
        build_file = clean_dep("//third_party:waveglow_model.BUILD"),
        sha256 = "a3a08e91470f8870a56e4fc4ff6fe479c31797f8d846200958f77733fa1d6cbb",
        url = "https://developer.nvidia.com/isaac/download/third_party/waveglow_randVect_noiseTrunc_fp16-v0-tar-gz",
        type = "tar.gz",
        licenses = ["//:LICENSE"],
    )

    isaac_new_http_archive(
        name = "libargus_aarch64_nano",
        build_file = clean_dep("//third_party:libargus_nano.BUILD"),
        sha256 = "b989ef88992bdccc99072031225e76d0b84d792be785bcfb54584fd0f2ec7093",
        url = "https://developer.nvidia.com/isaac/download/third_party/libargus-2019-3-aarch64_nano-tgz",
        type = "tgz",
        strip_prefix = "libargus",
        licenses = ["https://raw.githubusercontent.com/pauldotknopf/JetsonTX1Drivers/master/nv_tegra/LICENSE.libargus"],
    )

    isaac_new_http_archive(
        name = "hgmm_impl",
        sha256 = "6f8fa587486aefacf6ca8a9234a96865b43a6299d7d374753ad7a9f26d9be825",
        url = "https://developer.nvidia.com/isaac/download/third_party/libhgmm_impl_bionic_03_25_19-tar-xz",
        build_file = clean_dep("//third_party:libhgmm_impl.BUILD"),
        type = "tar.xz",
        licenses = ["//:LICENSE"],
    )

    isaac_new_http_archive(
        name = "livox_sdk",
        sha256 = "1c62b3f85a548183100cc94730926b64df2feeb10883f7d3bd245708ef0340a5",
        url = "https://developer.nvidia.com/isaac/download/third_party/Livox-SDK-1-0-0-tar-gz",
        build_file = clean_dep("//third_party:livox_sdk.BUILD"),
        type = "tar.gz",
        strip_prefix = "Livox-SDK-1.0.0",
        licenses = ["//:LICENSE"],
    )

    isaac_new_git_repository(
        name = "lfll",
        remote = "https://github.com/nicopauss/LFLL.git",
        commit = "9d29453368c432e373acf712d51515505ef057b0",
        build_file = clean_dep("//third_party:lfll.BUILD"),
        patches = [clean_dep("//third_party:lfll.patch")],
        licenses = ["https://github.com/nicopauss/LFLL/blob/master/LICENSE"],
    )

    isaac_new_git_repository(
        name = "efll",
        remote = "https://github.com/zerokol/eFLL.git",
        commit = "640b8680b6535768f318172b0a28a5e4091d8f60",
        build_file = clean_dep("//third_party:efll.BUILD"),
        licenses = ["https://github.com/zerokol/eFLL/blob/master/LICENSE"],
    )

    isaac_new_http_archive(
        name = "path_segmentation_pretrained_models",
        build_file = clean_dep("//third_party:path_segmentation_pretrained_models.BUILD"),
        sha256 = "697e186c363ba68e8fe6efa590c783ae7132e13889da60df7d65ae582fb0712d",
        url = "https://developer.nvidia.com/isaac/download/third_party/path_segmentation_pretrained_models_2019_06_11-tar-xz",
        type = "tar.xz",
        strip_prefix = "path_segmentation_pretrained_models",
        licenses = ["//:LICENSE"]
    )
