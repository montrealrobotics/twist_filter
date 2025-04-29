from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    active_filters_arg = DeclareLaunchArgument(
        'active_filters',
        default_value=PathJoinSubstitution([
            FindPackageShare('twist_filter'),
            'config', 'active_filters_default_fir.yaml'
        ]),
        description='Path to active filters configuration file'
    )
    
    filter_config_arg = DeclareLaunchArgument(
        'filter_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('twist_filter'),
            'config', 'default_combo.yaml'
        ]),
        description='Path to filter configuration file'
    )
    
    filter_name_arg = DeclareLaunchArgument(
        'filter_name',
        default_value='combo_twist_filter',
        description='Name of the filter node'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/cmd_vel',
        description='Input topic name'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/mux/cmd_vel',
        description='Output topic name'
    )

    filter_1_arg = DeclareLaunchArgument(
        'filter_1',
        default_value='avg',
        description='Filter to apply to linear velocity'
    )

    filter_2_arg = DeclareLaunchArgument(
        'filter_2',
        default_value='lp',
        description='Filter to apply to angular velocity'
    )

    active_filters = LaunchConfiguration('active_filters')
    filter_config = LaunchConfiguration('filter_config')
    filter_name = LaunchConfiguration('filter_name')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    
    filter_node = Node(
        package='twist_filter',
        executable='init_combo.py',
        name=filter_name,
        parameters=[
            active_filters, 
            filter_config,
            {
                'filter_1': LaunchConfiguration('filter_1'),
                'filter_2': LaunchConfiguration('filter_2'),
            }
        ],
        remappings=[
            ('filter_in', input_topic),
            ('filter_out', output_topic),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        active_filters_arg,
        filter_config_arg,
        filter_name_arg,
        input_topic_arg,
        output_topic_arg,
        filter_node,
        filter_1_arg,
        filter_2_arg
    ])