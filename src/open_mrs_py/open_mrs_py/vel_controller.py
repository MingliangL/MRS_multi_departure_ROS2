import math
import numpy as m
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from example_interfaces.msg import Int32
from example_interfaces.msg import Int32MultiArray
from open_mrs_srv_msg.srv import SetSelfAlert
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Modes(Enum):
    WORKING = 0
    TRANSIENT = 1
    LEFT = 2
    LEAVING = 3

class VelController(Node): 

    self_idx = 0
    num_robot = 0
    self_frame = "mec_car"
    timer_period = 2.0/100.0

    # 2 lowest bits are used to indicate the leaving state of the robot, 6 high bits 
    # indicate robot indices
    self_leave_alert = 0x0000 
    
    other_frame = []
    rho = []
    rel_x = []
    rel_y = []
    rel_th = []
    connection = [] # indicates whether two robots are connected (1) or disconnected (0)
    work_mode = [] # 4 states: 0 working, 1 transient, 2 left, 3 leaving
    theta = 0.0
    mode = 0x0000 # 4 states: 0 working, 1 transient, 2 left, 3 leaving
    l_flag = -1 # indicates which robot is trying to leave
    v_max = 5.0
    w_max = 5.0

    v_0 = 0.5
    theta_f = 0.0
    k_th = 2.0
    k_v = 5.0
    k_rho = 0.2
    max_d = 2.0
    epsilon = 0.1
    # min_d = 0.4
    

    def __init__(self):
        super().__init__('vel_controller')

        self.declare_parameter('self_idx', 1)
        self.declare_parameter('num_robot', 2)

        self.self_idx = self.get_parameter('self_idx').get_parameter_value().integer_value
        self.num_robot = self.get_parameter('num_robot').get_parameter_value().integer_value
        self.self_frame = f"mec_car{self.self_idx}"
        self.self_leave_alert = (self.self_idx << 2)

        for i in range(self.num_robot): 
            self.other_frame.append(f'mec_car{i}')
            self.get_logger().info(f'Append mec_car{i}')
            # print(self.work_mode)

        self.rho = [0.0]*(self.num_robot)
        self.rel_x = [0.0]*(self.num_robot)
        self.rel_y = [0.0]*(self.num_robot)
        self.rel_th = [0.0]*(self.num_robot)
        self.work_mode = [0]*(self.num_robot)
        self.connection = [0]*(self.num_robot)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        self.cmd_vel_pub_ = self.create_publisher(Twist, f'/mec_car{self.self_idx}/cmd_vel', 10)
        self.alert_pub_ = self.create_publisher(Int32, '/alert', 10) # Alert this robot is leaving
        self.mode_pub_ = self.create_publisher(Int32, f'/mec_car{self.self_idx}/mode', 10) # publish controller mode to Unity
        # self.connection_pub_ = self.create_publisher(Int32MultiArray, f'/mec_car{self.self_idx}/connected', 10) # publish connectivity info to Unity
        self.alert_sub_ = self.create_subscription(Int32, '/alert', self.alert_cb, 10)
        self.self_alert_srv_ = self.create_service(SetSelfAlert, 
                                                f'/mec_car{self.self_idx}/self_alert_srv', 
                                                self.self_alert_cb)

        self.timer = self.create_timer(self.timer_period, self.vel_control)


    def self_alert_cb(self, request, response): 
        full_msg = 0xfffffffc + request.mode # prepare input data
        self.self_leave_alert = (self.self_idx << 2) + 0x0003 # reset state of this robot
        self.self_leave_alert &= full_msg

        pub_msg = Int32()
        pub_msg.data = self.self_leave_alert
        self.alert_pub_.publish(pub_msg)

        return response
    
    
    def alert_cb(self, msg):
        mode_msg = Int32()
        if (msg.data>>2)==self.self_idx:
            return
        
        for idx, state in enumerate(self.work_mode):
            if (msg.data>>2) == idx:
                self.work_mode[idx] = msg.data & 0x0003
                if (msg.data & 0x0003) == Modes.LEAVING.value and self.connection[idx]:
                    self.mode = Modes.TRANSIENT.value
                    self.l_flag = idx
                    mode_msg.data = self.mode
                    self.mode_pub_.publish(mode_msg)
                # elif (msg.data & 0x0003) == Modes.LEFT.value and self.connection[idx]: 
                #     self.mode = Modes.WORKING.value
                #     self.l_flag = -1
                #     mode_msg.data = self.mode
                #     self.mode_pub_.publish(mode_msg)
                break
        
        # self.get_logger().info(f'Turtlebot{self.self_idx}: {self.mode}')


    def vel_control(self):
        msg = Twist()
        connected = Int32MultiArray()

        if (self.self_leave_alert & 0x0003) == 2:
            self.cmd_vel_pub_.publish(msg)
            self.connection = [0]*(self.num_robot)
            connected.data = self.connection
            # self.connection_pub_.publish(connected)
            return 
        
        self.get_relative_pos()
        connected.data = self.connection
        # self.connection_pub_.publish(connected)

        if (self.self_leave_alert & 0x0003) == 1:
            if self.is_connected(): 
                self.self_leave_alert = 2 + (self.self_idx<<2)
                alert_msg = Int32()
                alert_msg.data = self.self_leave_alert
                self.alert_pub_.publish(alert_msg)

            self.cmd_vel_pub_.publish(msg)
            return 

        delta = self.pJpz()

        # self.get_logger().info(f'Turtlebot{self.self_idx}: {self.connection}, {self.mode}, {delta}')
        
        if self.mode == Modes.WORKING.value: 
            msg.linear.x = -self.k_v*delta[0]
            msg.linear.y = -self.k_v*delta[1]

        elif self.mode == Modes.TRANSIENT.value: 
            msg.linear.x = -self.k_v*delta[0]
            msg.linear.y = -self.k_v*delta[1]

        # if m.abs(msg.linear.x) > m.abs(self.v_max): 
        #     msg.linear.x = self.v_max*m.sign(msg.linear.x)
        # if m.abs(msg.linear.y) > m.abs(self.v_max): 
        #     msg.linear.y = self.v_max*m.sign(msg.linear.y)
        self.cmd_vel_pub_.publish(msg)


    def pJpz(self): 
        delta = [0.0, 0.0]

        if self.mode == Modes.TRANSIENT.value:
            delta[0] = self.rho[self.l_flag] + self.rho[self.l_flag]/(self.max_d**2-self.rho[self.l_flag]**2)
            delta[1] = 0.0
            return delta

        for i in range(self.num_robot-1): 
            if self.connection[i] == 0:
                continue
            if self.work_mode[i] == Modes.WORKING.value:
                delta[0] += (2*self.rel_x[i]/((self.max_d-self.rho[i])**2) + 
                    2*self.rel_x[i]*self.rho[i]/(self.max_d-self.rho[i])**3)
                delta[1] += (2*self.rel_y[i]/((self.max_d-self.rho[i])**2) + 
                    2*self.rel_y[i]*self.rho[i]/(self.max_d-self.rho[i])**3)
            # elif self.work_mode[i] == Modes.LEFT.value:
            #     continue

        return delta 
    
    
    def is_connected(self): 
        """ Check if all the neighbors are within the distance of half of the communication range. 
            Did not choose to construct the graph Laplacian to save computational power. 
        """
        connected  = True
        for idx, distance in enumerate(self.rho):
            if self.connection[idx] == 0:
                continue
            if distance >= (self.max_d-self.epsilon)/2.0:
                connected = False
                break 

        return connected
    

    def get_relative_pos(self):
        try: 
            tf_ = self.tf_buffer_.lookup_transform('odom', self.self_frame, rclpy.time.Time())
        except TransformException as ex:
                self.get_logger().warning('Unable to find self pose. ')
                return

        qx = tf_.transform.rotation.x
        qy = tf_.transform.rotation.y
        qz = tf_.transform.rotation.z
        qw = tf_.transform.rotation.w
        self.theta = self.yaw_from_quat(qx, qy, qz, qw)
        # self.get_logger().info(f'theta{self.self_idx}: {self.theta}')
        # self.get_logger().info(f'{"{:.2f}".format(tf_.transform.translation.x)}, {"{:.2f}".format(tf_.transform.translation.y)}')

        for idx, frame in enumerate(self.other_frame):
            if idx == self.self_idx:
                continue

            try:
                tf_ = self.tf_buffer_.lookup_transform(self.self_frame, frame, rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warning('Unable to find frames. ')
                continue

            qx = tf_.transform.rotation.x
            qy = tf_.transform.rotation.y
            qz = tf_.transform.rotation.z
            qw = tf_.transform.rotation.w

            self.rel_x[idx] = -tf_.transform.translation.x # robot_i.rel_x[robot_j] = x[i] - x[j], following terms the same
            self.rel_y[idx] = -tf_.transform.translation.y
            # self.rel_x[idx], self.rel_y[idx] = self.to_odom_frame(self.rel_x[idx], self.rel_y[idx], self.theta)
            self.rel_th[idx] = -self.yaw_from_quat(qx, qy, qz, qw)

            # self.get_logger().info(f'Turtlebot{self.self_idx}: rel_th[{idx}] = {self.rel_th[idx]}')

            self.rho[idx] = math.sqrt(self.rel_x[idx]**2+self.rel_y[idx]**2) 

            if (self.self_leave_alert & 0x03) == 1: 
                continue
            if (self.rho[idx] > self.max_d) or self.work_mode[idx]  == Modes.LEFT.value:
                self.connection[idx] = 0
            elif self.rho[idx] <= self.max_d-self.epsilon:
                self.connection[idx] = 1

            if self.connection[idx] == 1 and self.work_mode[idx] == Modes.LEAVING.value:
                self.mode = Modes.TRANSIENT.value
                self.l_flag = idx
                mode_msg = Int32()
                mode_msg.data = self.mode
                self.mode_pub_.publish(mode_msg)


    def yaw_from_quat(self, x, y, z, w):
        t1 = 2.0*(w*z + x*y)
        t2 = 1.0 - 2.0*(y**2 + z**2)
        yaw = m.arctan2(t1,t2)
        return yaw
    

    # def to_odom_frame(self, dx, dy, theta):
    #     dx_o = dx*m.cos(theta) - dy*m.sin(theta)
    #     dy_o = dx*m.sin(theta) + dy*m.cos(theta)
        
    #     return dx_o, dy_o
    


def main(args=None):
    rclpy.init(args=args)
    vel_controller = VelController()

    try:
        rclpy.spin(vel_controller)
    except KeyboardInterrupt:
        pass
    
    vel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    