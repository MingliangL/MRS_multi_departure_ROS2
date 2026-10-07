import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from open_mrs_srv_msg.srv import SetSelfAlert
from open_mrs_py.vel_controller import Modes


def main(args=None):
    rclpy.init(args=args)
    caller_node = Node('multi_departure_caller')
    client1 = caller_node.create_client(SetSelfAlert, '/mec_car1/set_self_alert')
    client2 = caller_node.create_client(SetSelfAlert, '/mec_car2/set_self_alert')

    req1 = SetSelfAlert.Request()
    req2 = SetSelfAlert.Request()

    req1.mode = Modes.LEAVING.value
    req2.mode = Modes.LEAVING.value

    future1 = client1.call_async(req1)
    future2 = client2.call_async(req2)

if __name__ == '__main__':
    main()
    