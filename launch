from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():
	pursuit_node = Node(
		package='pure_pursuit',
		executable='pure_pursuit.py',
		name='pure_pursuit',
		output='screen',
		parameters=[{'speed':LaunchConfiguration('speed', default='0.0'),
			'max_turn_angle':LaunchConfiguration('max_turn_angle', default='0.0'),
			'look_ahead':LaunchConfiguration('look_ahead', default='0.0')}
		]
	)

	return LaunchDescription([
        pursuit_node
	])
