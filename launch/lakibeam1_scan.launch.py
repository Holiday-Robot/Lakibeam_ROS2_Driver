import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("lakibeam1"), "rviz", "lakibeam1_scan.rviz"
    )

    frame_id = LaunchConfiguration("frame_id")
    output_topic = LaunchConfiguration("output_topic")
    inverted = LaunchConfiguration("inverted")
    hostip = LaunchConfiguration("hostip")
    port = LaunchConfiguration("port")
    angle_offset = LaunchConfiguration("angle_offset")
    scanfreq = LaunchConfiguration("scanfreq")
    filter = LaunchConfiguration("filter")
    laser_enable = LaunchConfiguration("laser_enable")
    scan_range_start = LaunchConfiguration("scan_range_start")
    scan_range_stop = LaunchConfiguration("scan_range_stop")
    sensorip = LaunchConfiguration("sensorip")
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_frame_id_cmd = DeclareLaunchArgument(
        "frame_id",
        default_value="laser",
    )
    declare_output_topic_cmd = DeclareLaunchArgument(
        "output_topic",
        default_value="/hday/sensor/lidar/scan",
    )
    declare_inverted_cmd = DeclareLaunchArgument(
        "inverted",
        default_value="false",
    )
    declare_hostip_cmd = DeclareLaunchArgument(
        "hostip",
        default_value="0.0.0.0",
    )
    declare_port_cmd = DeclareLaunchArgument(
        "port",
        default_value='"2368"',
    )
    declare_angle_offset_cmd = DeclareLaunchArgument(
        "angle_offset",
        default_value="0",
    )
    declare_filter_cmd = DeclareLaunchArgument(
        "filter",
        default_value='"3"',
    )
    declare_scanfreq_cmd = DeclareLaunchArgument(
        "scanfreq",
        default_value='"30"',
    )
    declare_laser_enable_cmd = DeclareLaunchArgument(
        "laser_enable",
        default_value='"true"',
    )
    declare_scan_range_start_cmd = DeclareLaunchArgument(
        "scan_range_start",
        default_value='"45"',
    )
    declare_scan_range_stop_cmd = DeclareLaunchArgument(
        "scan_range_stop",
        default_value='"315"',
    )
    declare_sensorip_cmd = DeclareLaunchArgument(
        "sensorip",
        default_value="192.168.8.2",
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
    )

    richbeam_lidar_node = Node(
        package="lakibeam1",
        name="richbeam_lidar_node",
        executable="lakibeam1_scan_node",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "frame_id": frame_id,
                "output_topic": output_topic,
                "inverted": inverted,
                "hostip": hostip,
                "port": port,
                "angle_offset": angle_offset,
                "sensorip": sensorip,
                "scanfreq": scanfreq,
                "filter": filter,
                "laser_enable": laser_enable,
                "scan_range_start": scan_range_start,
                "scan_range_stop": scan_range_stop,
            }
        ],
        output="screen",
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(declare_frame_id_cmd)
    ld.add_action(declare_output_topic_cmd)
    ld.add_action(declare_inverted_cmd)
    ld.add_action(declare_hostip_cmd)
    ld.add_action(declare_port_cmd)
    ld.add_action(declare_angle_offset_cmd)
    ld.add_action(declare_filter_cmd)
    ld.add_action(declare_scanfreq_cmd)
    ld.add_action(declare_laser_enable_cmd)
    ld.add_action(declare_scan_range_start_cmd)
    ld.add_action(declare_scan_range_stop_cmd)
    ld.add_action(declare_sensorip_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(richbeam_lidar_node)
    ld.add_action(rviz_node)

    return ld
