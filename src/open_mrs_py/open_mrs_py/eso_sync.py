import rclpy
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension

from open_mrs_srv_msg.msg import DESOMsg


class DESOSynchroniser(Node):

    def __init__(self):
        super().__init__('eso_synchroniser')

        self.declare_parameter('num_robot', 2)
        self.num_robot = self.get_parameter('num_robot').get_parameter_value().integer_value
        if self.num_robot <= 0:
            raise ValueError('num_robot must be greater than zero')

        self.eso_messages = [None] * self.num_robot
        self.received_since_publish = [False] * self.num_robot
        self.eso_subscriptions = [
            self.create_subscription(
                DESOMsg,
                f'/mec_car{self_idx}/eso',
                lambda msg, idx=self_idx: self.eso_cb(msg, idx),
                10,
            )
            for self_idx in range(self.num_robot)
        ]
        self.eso_pub_ = self.create_publisher(DESOMsg, '/eso', 10)


    def eso_cb(self, msg, self_idx):
        expected_length = 2 * self.num_robot
        if len(msg.data) != expected_length:
            self.get_logger().error(
                f'Received {len(msg.data)} ESO values from robot {self_idx}; '
                f'expected {expected_length}'
            )
            return

        self.eso_messages[self_idx] = list(msg.data)
        self.received_since_publish[self_idx] = True

        if not all(self.received_since_publish):
            return

        combined_msg = DESOMsg()
        combined_msg.header.stamp = self.get_clock().now().to_msg()
        combined_msg.header.frame_id = 'eso'

        rows = MultiArrayDimension()
        rows.label = 'rows'
        rows.size = self.num_robot
        rows.stride = 2 * self.num_robot * self.num_robot
        cols = MultiArrayDimension()
        cols.label = 'cols'
        cols.size = 2 * self.num_robot
        cols.stride = 2 * self.num_robot
        combined_msg.layout.dim = [rows, cols]
        combined_msg.data = [
            value
            for robot_message in self.eso_messages
            for value in robot_message
        ]
        self.eso_pub_.publish(combined_msg)
        self.received_since_publish = [False] * self.num_robot



def main(args=None):
    rclpy.init(args=args)
    sync_node = DESOSynchroniser()

    try: 
        rclpy.spin(sync_node)
    except KeyboardInterrupt:
        pass
    finally:
        sync_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()