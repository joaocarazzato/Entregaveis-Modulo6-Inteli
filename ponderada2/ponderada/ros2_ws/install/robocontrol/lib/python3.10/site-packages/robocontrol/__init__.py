import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odometry
from time import sleep
from tf_transformations import euler_from_quaternion
import math

MAX_DIFF = 0.1

global posicoesrota 
posicoesrota = [
[1.0, 2.0],
[2.5, 2.0],
[0.5, 0.5],
[2.0, 2.5],
[4.0, 4.0]
]
    
class TurtleController(Node):
    
    def __init__(self, control_period=0.05):
        super().__init__('turtlecontroller')
        self.x_array = 1

        # se vira depois pra descobrir o que vc fez aqui
        self.odom = Odometry()
        self.actualpose = self.odom.pose.pose
        self.actualpose_x = self.actualpose.position.x
        self.actualpose_y = self.actualpose.position.y

        self.actualpose_x = posicoesrota[(self.x_array - 1)][0]
        self.actualpose_y = posicoesrota[(self.x_array - 1)][1]

        self.setpointodom = Odometry()
        self.setpoint = self.setpointodom.pose.pose
        self.setpoint_x = self.setpoint.position.x
        self.setpoint_y = self.setpoint.position.y

        self.setpoint_x = posicoesrota[self.x_array][0]
        self.setpoint_y = posicoesrota[self.x_array][1]
        print(f'self.pose: x={self.actualpose_x}, y={self.actualpose_y}')
        print(f'self.setpoint: x={self.setpoint_x}, y={self.setpoint_y}')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.control_timer = self.create_timer(timer_period_sec = control_period, callback = self.control_callback)

    def pose_callback(self, msg):
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        ang = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])
        self.actualpose_x = x
        self.actualpose_y = y
        self.get_logger().info(f"O robô está em x={round(x, 2)}, y={round(y, 2)}, theta={round(math.degrees(self.theta), 2)}")

    def control_callback(self):
        msg = Twist()
        self.setpoint_x = posicoesrota[self.x_array][0]
        self.setpoint_y = posicoesrota[self.x_array][1]

        x_diff = self.setpoint_x - self.actualpose_x
        y_diff = self.setpoint_y - self.actualpose_y
        angle_needed = math.atan2(y_diff, x_diff)
        theta_diff = angle_needed - self.theta
        print(f"x_diff={round(abs(x_diff), 2)}, y_diff={round(abs(y_diff), 2)}, theta_diff={round(abs(theta_diff), 2)}")

        if (abs(x_diff) < MAX_DIFF and abs(y_diff) < MAX_DIFF):
            self.x_array += 1

        if abs(theta_diff) > (MAX_DIFF -0.05):
            msg.linear.x = 0.0
            msg.angular.z = 0.2 if theta_diff > 0 else -0.2
        elif abs(x_diff) > MAX_DIFF:
            msg.linear.x = 0.2
        print(f"self.x_array= {self.x_array}, self.setpoint_x= {posicoesrota[self.x_array][0]}, self.setpoint_y= {posicoesrota[self.x_array][1]}")

        if self.x_array == (len(posicoesrota) - 1):
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.get_logger().info("Cheguei no meu destino.")
            exit()
        self.publisher.publish(msg)
            

def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()