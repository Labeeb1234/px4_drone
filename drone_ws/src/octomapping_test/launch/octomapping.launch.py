from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os

def generate_launch_description():
    share_dir = get_package_share_directory("octomapping_test")
    rviz_config_file = os.path.join(share_dir, "rviz/octomap.rviz")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Whether to use simulation time or not"
    )
    start_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="whether to launch rviz or not"
    )
    start_mapping_arg = DeclareLaunchArgument(
        "start_mapping",
        default_value="true",
        description="whether to start octomapping pkg or not"
    )

    octomapping_node = Node(
        condition=IfCondition(LaunchConfiguration("start_mapping")),
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output='screen',
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),

            "resolution": 0.1,
            "frame_id": "odom",
            "base_frame_id": "x500_obs_0/OakD-Lite/base_link/StereoOV7251",
            
            "sensor_model.max_range": 40.0,
            "incremental_2D_projection": False,
            "occupancy_grid_min_z": 0.1,
            "occupancy_grid_max_z": 1.0,

            "filter_ground_plane": True,
            "ground_filter.distance": 0.04,
            "ground_filter.angle": 0.15,
            "ground_filter.plane_distance": 1.0,
            "pointcloud_min_z": -3.0,
            "pointcloud_max_z": 1.5,
        }],
        remappings=[('cloud_in', 'point_cloud')]
    )


    # add static tf transform between base frame and camera_link to depict the depth cam offset
    static_tf_broadcaster_depth_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depthcamlink_static_tf_pub",
        output="screen",
        arguments=[
        '--x', '0.12',
        '--y', '0.03',
        '--z', '0.242',
        '--roll', '0.0',
        '--pitch', '0.0',
        '--yaw', '0.0',
        '--frame-id', 'x500_obs_0/base_footprint',
        '--child-frame-id', 'x500_obs_0/OakD-Lite/base_link/StereoOV7251'
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    static_tf_broadcaster_rgb_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rgbcamlink_static_tf_pub",
        output="screen",
        arguments=[
            '--x', '0.12',
            '--y', '0.03',
            '--z', '0.242',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'x500_obs_0/base_footprint', 
            '--child-frame-id', 'x500_obs_0/OakD-Lite/base_link/IMX214'
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    start_rviz_cmd = Node(
        condition=IfCondition(LaunchConfiguration("rviz")),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(start_rviz_arg)
    ld.add_action(start_mapping_arg)
    ld.add_action(static_tf_broadcaster_depth_cam)
    ld.add_action(static_tf_broadcaster_rgb_cam)
    ld.add_action(octomapping_node)
    ld.add_action(start_rviz_cmd)

    return ld
