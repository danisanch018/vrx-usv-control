import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    
    pkg_usv_control = get_package_share_directory('usv_control')
    pkg_vrx_gz = get_package_share_directory('vrx_gz')
    
    
    world_path = os.path.join(pkg_usv_control, 'worlds')
    ekf_config = os.path.join(pkg_usv_control, 'config', 'ekf.yaml')
    trajectory_config = os.path.join(pkg_usv_control, 'config', 'trajectory.yaml')


    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[world_path, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    
    vrx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vrx_gz, 'launch', 'competition.launch.py')
        ),
        launch_arguments={
            'world': 'sydney_regatta',
            'headless': 'False',
        }.items()
    )

    
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('gps/fix', '/wamv/sensors/gps/gps/fix'),
            ('imu', '/wamv/sensors/imu/imu/data'),
            ('odometry/gps', '/odometry/gps')
        ]
    )

    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    delayed_localization = TimerAction(
        period=7.0,
        actions=[navsat_node, ekf_node]
    )
    
    path_node = Node(
        package='usv_control_py',
        executable='path_publisher',
        name='path_publisher',
        parameters=[trajectory_config],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        vrx_launch,
        delayed_localization,
        path_node,
    ])