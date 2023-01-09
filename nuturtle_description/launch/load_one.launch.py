from launch import LaunchDescription, LaunchContext
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.actions import DeclareLaunchArgument, Shutdown, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import SetLaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='color',
            default_value='purple',
            choices=[
                'purple',
                'red',
                'green',
                'blue'],
            description='Color of TurtleBot'),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace = LaunchConfiguration('color'),
            parameters=[
                {"frame_prefix": 
                    PathJoinSubstitution(
                        [(LaunchConfiguration('color')),'']),
                "robot_description" :
                Command([ExecutableInPackage("xacro", "xacro"), " ",
                        PathJoinSubstitution(
                        [FindPackageShare("nuturtle_description"), "urdf/turtlebot3_burger.urdf.xacro"]),
                        " color:=",
                        LaunchConfiguration('color')
                        ]),
                    }],
                    ),

        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=[
                'true',
                'false'],
            description='Flag to enable rviz'),

        SetLaunchConfiguration(name = 'rvizconfig',
                               value = [FindPackageShare("nuturtle_description"),
                                        TextSubstitution(text ='/config/basic_'), 
                                        LaunchConfiguration('color'), 
                                        TextSubstitution(text ='.rviz')]),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace = LaunchConfiguration('color'),
            condition=(LaunchConfigurationEquals('use_rviz', 'true')),
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')
                                                 ],
            on_exit = Shutdown()
            ),

        DeclareLaunchArgument(
            name='use_jsp',
            default_value='true',
            choices=[
                'true',
                'false'],
            description='Flag to enable joint_state_publisher'), 
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace = LaunchConfiguration('color'),
            condition=(LaunchConfigurationEquals('use_jsp', 'true'))
            ),
    ])