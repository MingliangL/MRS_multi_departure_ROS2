import numpy as m

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformBroadcaster


class BaseLinkTfPub(Node): 

    target_frame = ''
    source_frame = ''
    world_frame = 'odom'
    pose_offset = [0.0, 0.0, 0.0] # [x, y, theta(rad)]
    
    def __init__(self):
        super().__init__('base_link_tf_pub')

        self.declare_parameter('target_frame', 'base_link1')
        self.declare_parameter('source_frame', 'Turtlebot1/base_link')
        self.declare_parameter('pose_offset', [0.0, 0.0, 0.0])

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.pose_offset = self.get_parameter('pose_offset').get_parameter_value().double_array_value

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        self.tf_talker_ = TransformBroadcaster(self)

        timer_period = 1/60.0
        self.timer = self.create_timer(timer_period, self.tf_pub)

    def tf_pub(self):
        try:
            tf_ = self.tf_buffer_.lookup_transform(self.world_frame, self.source_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warning(f'Unable to set tf offset of {self.source_frame}.')
            return

        qx = tf_.transform.rotation.x
        qy = tf_.transform.rotation.y
        qz = tf_.transform.rotation.z
        qw = tf_.transform.rotation.w
        yaw = self.yaw_from_quat(qx,qy,qz,qw) + self.pose_offset[2]

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.target_frame
        
        tf_msg.transform.translation.x = (tf_.transform.translation.x*m.cos(self.pose_offset[2]) - 
                                           tf_.transform.translation.y*m.sin(self.pose_offset[2])) + self.pose_offset[0]
        tf_msg.transform.translation.y = -(-tf_.transform.translation.x*m.sin(self.pose_offset[2]) - 
                                          tf_.transform.translation.y*m.cos(self.pose_offset[2])) + self.pose_offset[1]

        tf_msg.transform.rotation.w = m.cos(yaw/2.0)
        tf_msg.transform.rotation.z = m.sin(yaw/2.0)

        self.tf_talker_.sendTransform(tf_msg)

    def yaw_from_quat(self, x, y, z, w):
        t1 = 2.0*(w*z + x*y)
        t2 = 1.0 - 2.0*(y**2 + z**2)
        yaw = m.arctan2(t1,t2)
        return yaw



def main(args=None):
    rclpy.init(args=args)
    base_link_tf_pub = BaseLinkTfPub()

    try:
        rclpy.spin(base_link_tf_pub)
    except KeyboardInterrupt:
        pass
    
    base_link_tf_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
