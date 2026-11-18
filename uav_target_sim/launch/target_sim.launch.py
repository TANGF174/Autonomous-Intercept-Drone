from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'motion_mode',
        default_value='linear',
        description='Target motion mode: linear, circle, sinusoidal, random_walk'
    )
    path_length_arg = DeclareLaunchArgument(
        'linear_path_length',
        default_value='20.0',
        description='Total endpoint-to-endpoint straight path length in metres'
    )
    speed_arg = DeclareLaunchArgument(
        'linear_max_speed',
        default_value='3.0',
        description='Maximum target speed in m/s'
    )
    acceleration_arg = DeclareLaunchArgument(
        'linear_acceleration',
        default_value='2.0',
        description='Acceleration and braking magnitude in m/s^2'
    )
    position_tolerance_arg = DeclareLaunchArgument(
        'staging_position_tolerance',
        default_value='0.35',
        description='3D staging position tolerance in metres'
    )
    velocity_tolerance_arg = DeclareLaunchArgument(
        'staging_velocity_tolerance',
        default_value='0.25',
        description='3D staging speed tolerance in m/s'
    )
    stable_duration_arg = DeclareLaunchArgument(
        'staging_stable_duration',
        default_value='1.0',
        description='Continuous stable duration before motion starts in seconds'
    )

    target_node = Node(
        package='uav_target_sim',
        executable='uav_target_sim',
        name='uav_target_sim',
        output='screen',
        parameters=[{
            'motion_mode': LaunchConfiguration('motion_mode'),
            'linear_path_length': LaunchConfiguration('linear_path_length'),
            'linear_max_speed': LaunchConfiguration('linear_max_speed'),
            'linear_acceleration': LaunchConfiguration('linear_acceleration'),
            'staging_position_tolerance': LaunchConfiguration('staging_position_tolerance'),
            'staging_velocity_tolerance': LaunchConfiguration('staging_velocity_tolerance'),
            'staging_stable_duration': LaunchConfiguration('staging_stable_duration'),
        }],
    )

    return LaunchDescription([
        mode_arg,
        path_length_arg,
        speed_arg,
        acceleration_arg,
        position_tolerance_arg,
        velocity_tolerance_arg,
        stable_duration_arg,
        target_node,
    ])
