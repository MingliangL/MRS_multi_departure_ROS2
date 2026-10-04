from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 

    ld = LaunchDescription()

    base_link_tf1 = Node(package="open_mrs_py", 
        executable="base_link_tf_pub", 
        name="base_link_tf1",
        parameters=[{
            'target_frame': 'base_link1',
            'source_frame': 'Turtlebot1/base_link',
            'pose_offset': [0.0, 0.0, 0.0]
        }])
    base_link_tf2 = Node(package="open_mrs_py", 
        executable="base_link_tf_pub", 
        name="base_link_tf2",
        parameters=[{
            'target_frame': 'base_link2',
            'source_frame': 'Turtlebot2/base_link',
            'pose_offset': [2.0, -1.0, 0.0]
        }])
    base_link_tf3 = Node(package="open_mrs_py", 
        executable="base_link_tf_pub", 
        name="base_link_tf3",
        parameters=[{
            'target_frame': 'base_link3',
            'source_frame': 'Turtlebot3/base_link',
            'pose_offset': [0.0, -2.0, -0.5236]
        }])
    
    ld.add_action(base_link_tf1)
    ld.add_action(base_link_tf2)
    ld.add_action(base_link_tf3)
    
    return ld
