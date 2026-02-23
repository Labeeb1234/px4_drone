from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Whether to use simulation time or not"
    )

    octomapping_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output='screen',
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),

            "resolution": 0.05,
            "frame_id": "x500_obs_0/OakD-Lite/base_link/StereoOV7251",
            "base_frame_id": "x500_obs_0/base_footprint",
            
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
        remappings=[('cloud_in', 'cloud_registered')]
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

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(static_tf_broadcaster_depth_cam)
    ld.add_action(static_tf_broadcaster_rgb_cam)
    ld.add_action(octomapping_node)

    return ld
