import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	
	
	rviz2_node = Node(
		package='rviz2',
		executable='rviz2',
		output='log',
		arguments=['-d', [os.path.join(get_package_share_directory("mte544_kalman_filter"), 'launch', 'rviz_config.rviz')]]
	)

	kalman_filter_node = Node(
	    package='mte544_kalman_filter',
	    executable='mte544_kalman_node.py',
	    output='screen',
	)

	return LaunchDescription([
		rviz2_node,
		kalman_filter_node,
	])
