import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context):

    default_terminal = os.getenv('TERM_PROGRAM', 'gnome-terminal')  # Fallback to GNOME Terminal

    play_bag = ExecuteProcess(
        cmd=[
            default_terminal, '--', 'bash', '-c', 
            '\"ros2 bag play -r {} {}\"'.format(LaunchConfiguration('bag_rate').perform(context), LaunchConfiguration('bag_path').perform(context))
        ],
        output='screen',
        shell=True
    )

    return [play_bag]

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
            'listening':True,
            'legs_prefix': ["LF", "LH", "RF", "RH"],
            'foot_suffix': 'FOOT'
        }]
    )

    residual_plotter = Node(
        package='momobs_plot',
        executable='residual_plotter.py',
        condition=IfCondition(LaunchConfiguration('residuals')),
        parameters=[{
            'autoscale':True,
            'listening':True,
            'x_lim':3000,
            'legs_prefix': ["LF", "LH", "RF", "RH"],
        }]
    )

    load_func = OpaqueFunction(function = launch_setup)


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
            load_func
        ]
    )