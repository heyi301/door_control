import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # 门控制节点
    door_control_node = Node(
        package='door_control',
        executable='door_control_node',
        name='door_control_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        emulate_tty=True,
        prefix=['stdbuf -o L']  # 确保日志输出不被缓冲
    )
    
    return LaunchDescription([
        door_control_node
    ])