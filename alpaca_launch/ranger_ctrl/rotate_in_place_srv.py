import rclpy
from rclpy.node import Node
from ranger_msgs.msg import *
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Twist
import time
import numpy as np
import transforms3d
from ranger_msgs.srv import RotateInPlace
from rclpy.executors import MultiThreadedExecutor
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class RotateInPlaceServiceNode (Node):
    def __init__(self):
        super().__init__('rotate_in_place_service')
        
       
        self.cmd_vel_pub = self.create_publisher (Twist, '/cmd_vel', 10)
        self.theta = 0.0
        self.kp = 0.0

        # 1) Reentrant group so service + sub can run concurrently
        self.cbg = ReentrantCallbackGroup()

        # 2) TF-like QoS (best-effort, volatile)
        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.srv = self.create_service(RotateInPlace, 'rotate_in_place', self.rotate_in_place_callback, callback_group = self.cbg)
        self.create_subscription (TFMessage, '/tf', self.tf_callback, tf_qos,
            callback_group=self.cbg)

        

    def rotate_in_place_callback(self, request, response):
        # Implement the rotation logic here

        t = Twist() 
        self.kp = request.speed
        self.get_logger().info(f"self.kp is: {request.speed}")

        while (abs(request.theta - self.theta) > 0.1):
            # print ("my theta is ", self.theta)
            t.angular.z = max (min( (request.theta - self.theta)*self.kp, 0.3 ), -0.3)
            self.cmd_vel_pub.publish(t) 
            self.get_logger().info(f"Rotating in place: {t.angular.z}")
            time.sleep (0.02)
        
        print ("DONE")
        self.cmd_vel_pub.publish (Twist())

        response.success = True

        return response

        
    def tf_callback (self,msg):
        self.get_logger().info ("received tf")
        self.x = msg.transforms[0].transform.translation.x
        self.y = msg.transforms[0].transform.translation.y  

        quat_x = msg.transforms[0].transform.rotation.x
        quat_y = msg.transforms[0].transform.rotation.y
        quat_z = msg.transforms[0].transform.rotation.z
        quat_w = msg.transforms[0].transform.rotation.w
            # Convert quaternion to Euler angles
        euler = transforms3d.euler.quat2euler([quat_w, quat_x, quat_y, quat_z])
        self.theta = euler[2]
            # print ("my theta is ", np.rad2deg(self.theta))

def main(args=None):
        rclpy.init(args=args)
        node = RotateInPlaceServiceNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()



       

           

# def main(args=None):
#         rclpy.init(args=args)
#         node = RotateInPlaceServiceNode()
#         executor = MultiThreadedExecutor(num_threads=2)
#         executor.add_node(node)
#         try:
#             executor.spin()
       
#         finally:
#             executor.shutdown()
#             node.destroy_node()
#             rclpy.shutdown()

if __name__ == '__main__':
    main()