import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros
import os

def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('rover_description'),
        'rviz',
        'urdf_config.rviz')

    return launch.LaunchDescription([
      launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                          description='Flag to enable joint_state_publisher_gui'),

      Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
      ),
      
      Node(
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          arguments=['-d', rviz_config_dir],
          output='screen'),
    ])