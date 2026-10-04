from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description(): 

    num_robot = DeclareLaunchArgument(
      'num_robot', default_value=TextSubstitution(text='4')
    )

    ld = LaunchDescription()

    controller1 = Node(package='open_mrs_py', 
                       executable='vel_controller', 
                       name='controller0',
                       parameters=[{
                           'self_idx': 0,
                           'num_robot': LaunchConfiguration('num_robot')
                       }])
    controller2 = Node(package='open_mrs_py', 
                       executable='vel_controller', 
                       name='controller1', 
                       parameters=[{
                           'self_idx': 1,
                           'num_robot': LaunchConfiguration('num_robot')
                       }])
    controller3 = Node(package='open_mrs_py', 
                       executable='vel_controller', 
                       name='controller2',
                       parameters=[{
                           'self_idx': 2,
                           'num_robot': LaunchConfiguration('num_robot')
                       }])
    controller4 = Node(package='open_mrs_py', 
                       executable='vel_controller', 
                       name='controller3',
                       parameters=[{
                           'self_idx': 3,
                           'num_robot': LaunchConfiguration('num_robot')
                       }])
    # controller5 = Node(package='open_mrs_py', 
    #                    executable='vel_controller', 
    #                    name='controller5',
    #                    parameters=[{
    #                        'self_idx': 4,
    #                        'num_robot': LaunchConfiguration('num_robot')
    #                    }])
    
    ld.add_action(num_robot)
    ld.add_action(controller1)
    ld.add_action(controller2)
    ld.add_action(controller3)
    ld.add_action(controller4)
    # ld.add_action(controller5)


    return ld