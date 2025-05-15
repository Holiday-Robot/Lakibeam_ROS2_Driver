import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("lakibeam1"), "rviz", "lakibeam1_scan_dual.rviz"
    )

    frame_id0 = LaunchConfiguration("frame_id0")
    frame_id1 = LaunchConfiguration("frame_id1")
    output_topic0 = LaunchConfiguration("output_topic0")
    output_topic1 = LaunchConfiguration("output_topic1")
    inverted = LaunchConfiguration("inverted")
    hostip = LaunchConfiguration("hostip")
    port0 = LaunchConfiguration("port0")
    port1 = LaunchConfiguration("port1")
    angle_offset = LaunchConfiguration("angle_offset")
    scanfreq = LaunchConfiguration("scanfreq")
    filter = LaunchConfiguration("filter")
    laser_enable = LaunchConfiguration("laser_enable")
    scan_range_start = LaunchConfiguration("scan_range_start")
    scan_range_stop = LaunchConfiguration("scan_range_stop")
    sensorip = LaunchConfiguration("sensorip")
    log_level = LaunchConfiguration("log_level")
    use_tf = LaunchConfiguration("use_tf")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_frame_id0_cmd = DeclareLaunchArgument(
        "frame_id0",
        default_value="laser0",
    )
    declare_frame_id1_cmd = DeclareLaunchArgument(
        "frame_id1",
        default_value="laser1",
    )
    declare_output_topic0_cmd = DeclareLaunchArgument(
        "output_topic0",
        default_value="scan0",
    )
    declare_output_topic1_cmd = DeclareLaunchArgument(
        "output_topic1",
        default_value="scan1",
    )
    declare_inverted_cmd = DeclareLaunchArgument(
        "inverted",
        default_value="false",
    )
    declare_hostip_cmd = DeclareLaunchArgument(
        "hostip",
        default_value="0.0.0.0",
    )
    declare_port0_cmd = DeclareLaunchArgument(
        "port0",
        default_value='"2368"',
    )
    declare_port1_cmd = DeclareLaunchArgument(
        "port1",
        default_value='"2369"',
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
    declare_use_tf_cmd = DeclareLaunchArgument(
        "use_tf",
        default_value="False",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
    )

    richbeam_lidar_node0 = Node(
        package="lakibeam1",
        name="richbeam_lidar_node0",
        executable="lakibeam1_scan_node",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "frame_id": frame_id0,
                "output_topic": output_topic0,
                "inverted": inverted,
                "hostip": hostip,
                "port": port0,
                "angle_offset": angle_offset,
                "sensorip": sensorip,
                "scanfreq": scanfreq,
                "filter": filter,
                "laser_enable": laser_enable,
                "scan_range_start": scan_range_start,
                "scan_range_stop": scan_range_stop,
            }
        ],
        remappings=[
            (
                "/scan0",
                "/hday/sensor/lidar/scan0",
            ),
        ],
        output="screen",
    )

    richbeam_lidar_node1 = Node(
        package="lakibeam1",
        name="richbeam_lidar_node1",
        executable="lakibeam1_scan_node",
        parameters=[
            {
                "frame_id": frame_id1,
                "output_topic": output_topic1,
                "inverted": inverted,
                "hostip": hostip,
                "port": port1,
                "angle_offset": angle_offset,
                "sensorip": sensorip,
                "scanfreq": scanfreq,
                "filter": filter,
                "laser_enable": laser_enable,
                "scan_range_start": scan_range_start,
                "scan_range_stop": scan_range_stop,
            }
        ],
        remappings=[
            (
                "/scan1",
                "/hday/sensor/lidar/scan1",
            ),
        ],
        output="screen",
    )

    transform_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lidar0_static_tf_node = Node(
        condition=IfCondition(use_tf),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar0_static_tf_publisher",
        output="screen",
        remappings=transform_remappings,
        # [x, y, z, yaw, pitch, roll, frame_id, child_frame_id]
        arguments=["0.05", "0.0", "0.0", "0.0", "0.0", "0.0"] +
                    ["base_link", "laser0"]
    )

    lidar1_static_tf_node = Node(
        condition=IfCondition(use_tf),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar1_static_tf_publisher",
        output="screen",
        remappings=transform_remappings,
        # [x, y, z, yaw, pitch, roll, frame_id, child_frame_id]
        arguments=["-0.05", "0.0", "0.0", "3.1415926535", "0.0", "0.0"] +
                    ["base_link", "laser1"]
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

    ld.add_action(declare_frame_id0_cmd)
    ld.add_action(declare_frame_id1_cmd)
    ld.add_action(declare_output_topic0_cmd)
    ld.add_action(declare_output_topic1_cmd)
    ld.add_action(declare_inverted_cmd)
    ld.add_action(declare_hostip_cmd)
    ld.add_action(declare_port0_cmd)
    ld.add_action(declare_port1_cmd)
    ld.add_action(declare_angle_offset_cmd)
    ld.add_action(declare_filter_cmd)
    ld.add_action(declare_scanfreq_cmd)
    ld.add_action(declare_laser_enable_cmd)
    ld.add_action(declare_scan_range_start_cmd)
    ld.add_action(declare_scan_range_stop_cmd)
    ld.add_action(declare_sensorip_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_tf_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(richbeam_lidar_node0)
    ld.add_action(richbeam_lidar_node1)

    ld.add_action(lidar0_static_tf_node)
    ld.add_action(lidar1_static_tf_node)

    ld.add_action(rviz_node)
    return ld
