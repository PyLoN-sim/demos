from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    mission = LifecycleNode(package='pylon_demo_reusable', executable='mission',
        name='reusable_mission', namespace='', output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('pylon_demo_reusable'), 'config', 'mission.yaml']),
                    {'profile': LaunchConfiguration('profile')}])
    configure = RegisterEventHandler(OnProcessStart(target_action=mission, on_start=[
        EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(mission),
                                   transition_id=Transition.TRANSITION_CONFIGURE))]))
    # Configure automatically. Activation is explicit and requires live preflight.
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='orbital', choices=['orbital', 'hop']),
        DeclareLaunchArgument('bridge', default_value='true'),
        Node(package='pylon_bridge', executable='udp_bridge', name='pylon_bridge',
             arguments=['--host', '127.0.0.1'], condition=IfCondition(LaunchConfiguration('bridge'))),
        configure, mission,
    ])
