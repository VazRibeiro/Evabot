from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # Declare launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'interface',
        default_value='vcan0',
        description='CAN interface to use (e.g., can0, vcan0)'
    )
    
    can_interface = LaunchConfiguration('interface')
    
    # CAN gateway node
    can_gateway_node = Node(
        package='can_gateway',
        executable='can_gateway_node',
        name='can_gateway',
        parameters=[{
            'interface': can_interface
        }],
        output='screen'
    )
    
    # Configure the gateway after it starts (with delay)
    configure_gateway = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', '/can_gateway/change_state',
                    'lifecycle_msgs/srv/ChangeState', 
                    '{transition: {id: 1}}'
                ],
                output='screen'
            )
        ]
    )
    
    # Activate the gateway after configuration (with delay)
    activate_gateway = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', '/can_gateway/change_state',
                    'lifecycle_msgs/srv/ChangeState', 
                    '{transition: {id: 3}}'
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        can_interface_arg,
        can_gateway_node,
        configure_gateway,
        activate_gateway,
    ])
