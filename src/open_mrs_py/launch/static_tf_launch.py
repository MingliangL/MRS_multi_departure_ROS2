from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 

    ld = LaunchDescription()

    static_tf_1 = Node(package="tf2_ros", 
        executable="static_transform_publisher", 
        name="static_tf_1",
        arguments="0 0 0 0 0 0 Turtlebot1/base_link base_link1".split(' '))
    static_tf_2 = Node(package="tf2_ros", 
        executable="static_transform_publisher", 
        name="static_tf_2",
        arguments="2 -1 0 0 0 0 Turtlebot2/base_link base_link2".split(' '))
    static_tf_3 = Node(package="tf2_ros", 
        executable="static_transform_publisher", 
        name="static_tf_3",
        arguments="0 -2 0 0 0 0 Turtlebot3/base_link base_link3".split(' '))
    
    ld.add_action(static_tf_1)
    ld.add_action(static_tf_2)
    ld.add_action(static_tf_3)
    
    return ld
