import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# detect all 36h11 tags
cfg_36h11 = {
    "image_transport": "compressed",
    "family": "36h11",
    "size": 0.162,
    "max_hamming": 0,
    "z_up": True,
}

def generate_launch_description():
    composable_node = ComposableNode(
        node_name='apriltag',
        package='apriltag_ros', node_plugin='AprilTagNode',
        remappings=[("/apriltag/image/compressed", "/None/corrected_image/compressed"), ("/apriltag/camera_info", "/None/camera_node/camera_info")],
        parameters=[cfg_36h11])
    container = ComposableNodeContainer(
        node_name='tag_container',
        node_namespace='apriltag',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])
