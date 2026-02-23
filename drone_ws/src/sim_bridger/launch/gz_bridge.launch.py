from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    share_dir = get_package_share_directory('sim_bridger')
    config_file_fp = os.path.join(share_dir, "config/bridge_topics.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="whether to use sim time or not"
    )

    # For Harmonic and higher a portion of the bridge launching is slightly different

    # For GZ Garden and lower version use this format
    gz_ros_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=["--ros-args", '-p', f'config_file:={config_file_fp}'],
        # parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    # Bridging img topics
    gz_ros_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='ros_gz_image',
        output='screen',
        arguments=['/camera', '/depth_camera'],
        parameters=[{
            "qos": "sensor_data"
        }]
    )

    timer_action1 = TimerAction(
        period=2.0,
        actions=[gz_ros_image_bridge]
    )

    ld = LaunchDescription()
    ld.add_action(gz_ros_bridge_node)
    ld.add_action(timer_action1)
    return ld