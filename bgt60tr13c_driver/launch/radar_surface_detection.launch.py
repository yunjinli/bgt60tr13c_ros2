from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import sys
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    model_type_str = LaunchConfiguration('model_type').perform(context)
    input_path_str = LaunchConfiguration('model_path').perform(context)
    moving_avg_window = LaunchConfiguration('moving_avg_window')
    bgt_pkg = get_package_share_directory('bgt60tr13c_driver')
    default_model_base = os.path.join(bgt_pkg, 'config')
    print(f"Model Type selected: {model_type_str}")
    
    if model_type_str == 'torch':
        executable_name = 'radar_surface_detection_node'
        node_name = 'radar_surface_detection'
        default_path = os.path.join(default_model_base, 'rsd.pth')
    elif model_type_str == 'onnx':
        executable_name = 'radar_surface_detection_onnx_node'
        node_name = 'radar_surface_detection_onnx'
        default_path = os.path.join(default_model_base, 'qrsd.onnx')
    else:
        print(f"Error: model_type '{model_type_str}' not supported. Use 'torch' or 'onnx'.")
        sys.exit(1)

    if input_path_str == '':
        final_model_path = default_path
    else:
        final_model_path = input_path_str

    node = Node(
        package='bgt60tr13c_driver',
        executable=executable_name,
        name=node_name,
        output='screen',
        parameters=[
            {'model_path': final_model_path,
             'moving_avg_window': moving_avg_window}
        ]
    )

    return [node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_type", 
            default_value='torch', 
            description="Choose between 'torch' or 'onnx'"
        ),
        DeclareLaunchArgument(
            "model_path", 
            default_value='', 
            description="Path to model file (leave empty to use default based on type)"
        ),
        DeclareLaunchArgument(
            "moving_avg_window", 
            default_value='10',
            description="Window size for smoothing"
        ),
        
        OpaqueFunction(function=launch_setup)
    ])
