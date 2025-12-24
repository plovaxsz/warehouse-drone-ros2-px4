import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_px4_clean_nav = get_package_share_directory('px4_clean_nav')

    # 1. Agent DDS (Wajib jalan duluan)
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # 2. CLOCK BRIDGE (PENYELAMAT TF OLD DATA)
    # Ini memaksa ROS 2 ikut jam Gazebo
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 3. TF Broadcaster (Posisi Drone)
    tf_broadcaster = Node(
        package='px4_clean_nav',
        executable='tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}] # WAJIB TRUE
    )

    # 4. Static Transform (Sensor Lidar)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0.1', '0', '0', '0', 'base_link', 'link'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. Offboard Control (Auto Pilot)
    offboard_control = Node(
        package='px4_clean_nav',
        executable='offboard_control',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. Nav2 + SLAM (Tunggu 5 detik biar Clock sinkron)
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_px4_clean_nav, 'launch', 'bringup.launch.py')
                ),
                launch_arguments={
                    'params_file': os.path.join(pkg_px4_clean_nav, 'config', 'nav2_params.yaml'),
                    'slam': 'True',
                    'map': '',
                    'use_sim_time': 'True', # KUNCI UTAMA
                    'autostart': 'True'
                }.items()
            )
        ]
    )

    # 7. Rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_px4_clean_nav, 'config', 'nav.rviz')],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        micro_xrce_agent,
        clock_bridge, # JANGAN DIHAPUS
        tf_broadcaster,
        static_tf,
        offboard_control,
        nav2_launch,
        rviz
    ])
