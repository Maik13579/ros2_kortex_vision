from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

configurable_parameters = [
    {"name": "device", "default": "192.168.1.10", "description": "Device IPv4 address"},
    {
        "name": "camera",
        "default": "camera",
        "description": "'camera' should uniquely identify the device. All topics are pushed down into the 'camera' namespace.",
    },
    {
        "name": "camera_link_frame_id",
        "default": "camera_link",
        "description": "Camera link frame identifier",
    },
    {
        "name": "color_frame_id",
        "default": "camera_color_frame",
        "description": "Color camera frame identifier",
    },
    {
        "name": "depth_frame_id",
        "default": "camera_depth_frame",
        "description": "Depth camera frame identifier",
    },
    {
        "name": "color_camera_info_url",
        "default": "",
        "description": "URL of custom calibration file for color camera. See camera_info_manager docs for calibration URL details",
    },
    {
        "name": "depth_camera_info_url",
        "default": "",
        "description": "URL of custom calibration file for depth camera. See camera_info_manager docs for calibration URL details",
    },
    {
        "name": "depth_rtsp_element_config",
        "default": "depth latency=30",
        "description": "RTSP element configuration for depth stream",
    },
    {
        "name": "depth_rtp_depay_element_config",
        "default": "rtpgstdepay",
        "description": "RTP element configuration for depth stream",
    },
    {
        "name": "color_rtsp_element_config",
        "default": "color latency=30",
        "description": "RTSP element configuration for color stream",
    },
    {
        "name": "color_rtp_depay_element_config",
        "default": "rtph264depay",
        "description": "RTP element configuration for color stream",
    },
    {
        "name": "launch_color",
        "default": "true",
        "description": "Launch the color image node",
    },
    {
        "name": "launch_depth",
        "default": "true",
        "description": "Launch the depth image node",
    },
    {
        "name": "depth_registration",
        "default": "false",
        "description": "Hardware depth registration",
    },
    {
        "name": "max_color_pub_rate",
        "default": "30.0",
        "description": "Maximum image publication rate",
    },
    {
        "name": "max_depth_pub_rate",
        "default": "30.0",
        "description": "Maximum image publication rate",
    },
    {
        "name": "use_intra_process_comms",
        "default": "true",
        "description": "Use intra process communication inside the component container",
    },
]


def declare_configurable_parameters():
    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param["default"],
            description=param["description"],
        )
        for param in configurable_parameters
    ]


def launch_setup(context, *args, **kwargs):
    use_intra_process = LaunchConfiguration("use_intra_process_comms")
    launch_depth = LaunchConfiguration("launch_depth").perform(context).lower() == "true"
    launch_color = LaunchConfiguration("launch_color").perform(context).lower() == "true"
    depth_registration = LaunchConfiguration("depth_registration").perform(context).lower() == "true"

    composable_nodes = []

    if launch_depth:
        composable_nodes.append(
            ComposableNode(
                package="kinova_vision",
                plugin="ros_kortex_vision::VisionComponent",
                name="kinova_vision_depth_component",
                namespace=LaunchConfiguration("camera"),
                parameters=[
                    {
                        "camera_type": "depth",
                        "camera_name": "depth",
                        "camera_info_url_default": "package://kinova_vision/launch/calibration/default_depth_calib_%ux%u.ini",
                        "camera_info_url_user": LaunchConfiguration("depth_camera_info_url").perform(context),
                        "stream_config": "rtspsrc location=rtsp://"
                        + LaunchConfiguration("device").perform(context)
                        + "/"
                        + LaunchConfiguration("depth_rtsp_element_config").perform(context)
                        + " ! "
                        + LaunchConfiguration("depth_rtp_depay_element_config").perform(context),
                        "frame_id": LaunchConfiguration("depth_frame_id").perform(context),
                        "max_pub_rate": LaunchConfiguration("max_depth_pub_rate"),
                        "debug": False,
                    }
                ],
                remappings=[
                    ("camera_info", "depth/camera_info"),
                    ("image_raw", "depth/image_raw"),
                ],
                extra_arguments=[{"use_intra_process_comms": use_intra_process}],
            )
        )

    if launch_color:
        composable_nodes.append(
            ComposableNode(
                package="kinova_vision",
                plugin="ros_kortex_vision::VisionComponent",
                name="kinova_vision_color_component",
                namespace=LaunchConfiguration("camera"),
                parameters=[
                    {
                        "camera_type": "color",
                        "camera_name": "color",
                        "camera_info_url_default": "package://kinova_vision/launch/calibration/default_color_calib_%ux%u.ini",
                        "camera_info_url_user": LaunchConfiguration("color_camera_info_url").perform(context),
                        "stream_config": "rtspsrc location=rtsp://"
                        + LaunchConfiguration("device").perform(context)
                        + "/"
                        + LaunchConfiguration("color_rtsp_element_config").perform(context)
                        + " ! "
                        + LaunchConfiguration("color_rtp_depay_element_config").perform(context)
                        + " ! avdec_h264 ! videoconvert",
                        "frame_id": LaunchConfiguration("color_frame_id").perform(context),
                        "max_pub_rate": LaunchConfiguration("max_color_pub_rate"),
                        "debug": False,
                    }
                ],
                remappings=[
                    ("camera_info", "color/camera_info"),
                    ("image_raw", "color/image_raw"),
                ],
                extra_arguments=[{"use_intra_process_comms": use_intra_process}],
            )
        )

    if depth_registration:
        composable_nodes.extend(
            [
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::RegisterNode",
                    name="register_node",
                    namespace=LaunchConfiguration("camera"),
                    remappings=[
                        ("rgb/camera_info", "color/camera_info"),
                        ("depth/camera_info", "depth/camera_info"),
                        ("depth/image_rect", "depth/image_raw"),
                    ],
                    # Due to the resolution disparity between the depth images and
                    # the rgb images from the embedded D410 rgbd camera on the gen3
                    # kinova, when the depth images are registered in the color
                    # image frame, many holes occur in the registered depth data.
                    # Turning on fill_upsampling_holes spreads the lower resolution
                    # depth data out across the registered image to eliminate these
                    # holes.
                    parameters=[
                        {
                            "fill_upsampling_holes": True,
                        }
                    ],
                    extra_arguments=[{"use_intra_process_comms": use_intra_process}],
                ),
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzrgbNode",
                    name="point_cloud_xyzrgb",
                    namespace=LaunchConfiguration("camera"),
                    remappings=[
                        ("rgb/camera_info", "color/camera_info"),
                        ("depth/camera_info", "depth/camera_info"),
                        ("rgb/image_rect_color", "color/image_raw"),
                        ("depth/image_rect", "depth/image_raw"),
                        ("points", "depth/color/points"),
                    ],
                    extra_arguments=[{"use_intra_process_comms": use_intra_process}],
                ),
            ]
        )

    launch_actions = [
        ComposableNodeContainer(
            name="registered_depth_images",
            namespace=LaunchConfiguration("camera"),
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=composable_nodes,
            output="both",
        )
    ]

    if launch_depth:
        launch_actions.append(
            Node(
                package="tf2_ros",
                namespace=LaunchConfiguration("camera"),
                executable="static_transform_publisher",
                name="camera_depth_tf_publisher",
                output="both",
                arguments=[
                    "-0.0195",
                    "-0.005",
                    "0",
                    "0",
                    "0",
                    "0",
                    LaunchConfiguration("camera_link_frame_id"),
                    LaunchConfiguration("depth_frame_id"),
                ],
            )
        )

    if launch_color:
        launch_actions.append(
            Node(
                package="tf2_ros",
                namespace=LaunchConfiguration("camera"),
                executable="static_transform_publisher",
                name="camera_color_tf_publisher",
                output="both",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    LaunchConfiguration("camera_link_frame_id"),
                    LaunchConfiguration("color_frame_id"),
                ],
            )
        )

    return launch_actions


def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters() + [OpaqueFunction(function=launch_setup)]
    )
