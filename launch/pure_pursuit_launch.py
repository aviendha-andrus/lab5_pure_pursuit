from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():
	pursuit_node = Node(
		package='pure_pursuit',
		executable='pure_pursuit_node.py',
		name='pure_pursuit',
		output='screen',
		parameters=[{'speed':LaunchConfiguration('speed', default='0.0'),
			'max_turn':LaunchConfiguration('max_turn', default='0.0'),
			'lookahead':LaunchConfiguration('lookahead', default='0.0')}
		]
	)

	return LaunchDescription([
        pursuit_node
	])

# ros2 launch pure_pursuit pure_pursuit_launch.py speed:=0.7 lookahead:=1.0 max_turn:=0.5

