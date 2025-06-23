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
    estimate_contacts_arg = DeclareLaunchArgument('estimate_contacts', default_value='false', description='Wether to launch or not the contact estimation package')

    estimate_contacts = LaunchConfiguration("estimate_contacts")

    description_pkg = get_package_share_directory('anymal_c_simple_description')
    haptiquad_ros_pkg = get_package_share_directory('haptiquad_ros2')
    self_pkg = get_package_share_directory('haptiquad_rosbag_bringup')
    haptiquad_contacts_pkg = get_package_share_directory('haptiquad_contacts')

    haptiquad_config = os.path.join(self_pkg, 'config', 'haptiquad_bag.yaml')


    description_launch_file = os.path.join(description_pkg, 'launch', 'floating_base_description.launch.py')   
    haptiquad_launch_file = os.path.join(haptiquad_ros_pkg, 'launch', 'bag_wrapper.launch.py')
    estimate_contacts_launch_file = os.path.join(haptiquad_contacts_pkg, 'launch', 'haptiquad_estimator_mujoco.launch.py')

    haptiquad = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(haptiquad_launch_file),
            launch_arguments={'config_file': haptiquad_config}.items()
            )
    description = IncludeLaunchDescription(PythonLaunchDescriptionSource(description_launch_file))

    estimate_contacts_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(estimate_contacts_launch_file),
        launch_arguments={'rviz': str(True), "plot": str(False)}.items(),
        condition=IfCondition(estimate_contacts)
    )


    world_to_odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '0', '--qy', '0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'world', '--child-frame-id', 'odom'
            ]
        )


    force_plotter = Node(
        package='haptiquad_plot',
        executable='force_plotter.py',
        condition=IfCondition(LaunchConfiguration('force')),
        parameters=[{
            'autoscale':True,
            'listening':True,
            'legs_prefix': ["LF", "LH", "RF", "RH"],
            'foot_suffix': 'FOOT',
            'x_lim': 5.0,
            'memory_limit': 2000,
        }]
    )

    residual_plotter = Node(
        package='haptiquad_plot',
        executable='residual_plotter.py',
        condition=IfCondition(LaunchConfiguration('residuals')),
        parameters=[{
            'autoscale':True,
            'listening':True,
            'x_lim':5.0,
            'memory_limit': 5000,
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
            haptiquad,
            force_plotter,
            residual_plotter,
            estimate_contacts_arg,
            estimate_contacts_launch,
            world_to_odom,
            load_func
        ]
    )