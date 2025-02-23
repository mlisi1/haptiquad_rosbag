import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    rosbag_file_arg = DeclareLaunchArgument('bag_path', default_value='', description='Path to the rosbag file')
    force_arg = DeclareLaunchArgument('force', default_value='false', description='Launch extra nodes if true')
    residual_arg = DeclareLaunchArgument('residuals', default_value='false', description='Launch extra nodes if true')
    bag_rate_arg = DeclareLaunchArgument('bag_rate', default_value='0.3', description='Launch extra nodes if true')


    description_pkg = get_package_share_directory('anymal_c_simple_description')
    momobs_ros_pkg = get_package_share_directory('momobs_ros2')


    description_launch_file = os.path.join(description_pkg, 'launch', 'floating_base_description.launch.py')   
    momobs_launch_file = os.path.join(momobs_ros_pkg, 'launch', 'bag_wrapper.launch.py')

    momobs = IncludeLaunchDescription(PythonLaunchDescriptionSource(momobs_launch_file))
    description = IncludeLaunchDescription(PythonLaunchDescriptionSource(description_launch_file))


    force_plotter = Node(
        package='momobs_plot',
        executable='force_plotter.py',
        condition=IfCondition(LaunchConfiguration('force')),
        parameters=[{
            'autoscale':True,
            'listening':True
        }]
    )

    residual_plotter = Node(
        package='momobs_plot',
        executable='residual_plotter.py',
        condition=IfCondition(LaunchConfiguration('residuals')),
        parameters=[{
            'autoscale':True,
            'listening':True,
            'x_lim':3000
        }]
    )


    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r', LaunchConfiguration('bag_rate'), LaunchConfiguration('bag_path')],
        output='screen'
    )



    return LaunchDescription(
        [
            rosbag_file_arg,
            force_arg,
            residual_arg,
            bag_rate_arg,
            description,
            momobs,
            force_plotter,
            residual_plotter,
            play_bag
        ]
    )