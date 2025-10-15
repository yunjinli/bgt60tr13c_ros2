from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    # default to the packaged YAML, but allow override: params:=/path/to/other.yaml
    default_yaml = PathJoinSubstitution([
        FindPackageShare('bgt60tr13c_driver'),
        'config',
        'radar.yaml'
    ])
    params_arg = DeclareLaunchArgument(
        'params',
        default_value=default_yaml,
        description='Path to YAML params file for bgt60tr13c_node'
    )

    params = LaunchConfiguration('params')

    port_arg = DeclareLaunchArgument("port", default_value='')
    port = LaunchConfiguration("port")

    vis_arg = DeclareLaunchArgument(
        'vis',
        default_value='false',   # or 'true' if you want it on by default
        description='Run the visualization node (true/false)'
    )
    vis = LaunchConfiguration('vis')

    node = Node(
        package='bgt60tr13c_driver',
        executable='radar_node',
        name='bgt60tr13c_node',
        output='screen',
        parameters=[params, 
                    {'port': ParameterValue(port, value_type=str)}
                    ],
    )

    vis_node = Node(
        package='bgt60tr13c_driver',
        executable='raw_frame_viz_cmap',
        name='raw_frame_viz_cmap',
        output='screen',
        parameters=[],
        condition=IfCondition(vis), 
    )


    return LaunchDescription([
        params_arg,
        port_arg,
        vis_arg,
        node,
        vis_node
    ])
