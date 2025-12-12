###############################
#
# HeadtrackerMapperNode Module 
# Subcribes to Bridge node and publishes to vesc driver node
# Refer to README for complete package structure 
#
##############################
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math


class HeadtrackerMapperNode(Node):
    def __init__(self):
        super().__init__('headtracker_mapper_node')

        #units = degrees
        self.yaw_deadzone = 2.0
        self.pitch_deadzone = 3.0
        self.max_yaw = 25.0
        self.max_pitch = 30.0
        self.counter = 0
        self.past_yall = 0.0
        self.past_pitch = 0.0

        self.subscription = self.create_subscription(PoseStamped,'/head_pose',self.head_pose_callback,10)

        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)

        self.get_logger().info(" Mapper Node Started!")

    #Function will return 0 pitch/yaw if its < deadzone
    def apply_deadzone(self, value_deg,deadzone_deg):
        if abs(value_deg) < deadzone_deg:
            return 0.0
        else: 
            return value_deg
        
    #check what number of input it is, if its divisible by 1000 then we want the value
    def num_counter(self,value_deg, pitch_yall):
        counter_result = self.counter % 100
        self.counter += 1
        if(counter_result == 0):
            return value_deg
        else:
            if(pitch_yall == "pitch"):
                return self.past_pitch
            else:
                return self.past_yall

    #I have no idea why but for some stupid reason the VESC node we were using for steering is not clamped to [-1,1]
    # it's actually clamped to [-1.5,0.5] so -0.5 results in servo => 0.5
    def steering_deg(self, yaw_deg):
        if(yaw_deg < -20):
          return -1.5
        elif(yaw_deg >= -20 and yaw_deg < -10):
          return -1.0
        elif(yaw_deg >= -10 and yaw_deg <= 10):
          return -0.5
        elif(yaw_deg > 10 and yaw_deg < 20):
          return 0.0
        else:
          return 0.5

    #Scale the value based on max degree i.e 15 degrees of pitch --> 15/30 = 0.5 throttle
    def scale(self, value_deg,max_deg):
        scaled = value_deg/max_deg
        return max(-1.0,min(1.0,scaled)) #Clamp to [-1,1]

    #Scale and clamp yaw degree 
    def yaw_to_steering(self, yaw_deg,max_deg):
        yaw_deg = max(-max_deg, min(max_deg, yaw_deg))
        return -(yaw_deg / max_deg) - 0.5

    def head_pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        #translate back from quaternion -> euler // extract yaw, pitch, roll
        #Refer to: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/ for math
        sinr_cosp = 2 * (q.w*q.x + q.y*q.z)
        cosr_cosp = 1 - 2*(q.x*q.x + q.y*q.y)
        roll = math.atan2(sinr_cosp,cosr_cosp)

        sinp = 2 * (q.w*q.y - q.z*q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi/ 2,sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        #end conversion

        yaw_deg = math.degrees(yaw)
        pitch_deg = math.degrees(pitch)

        yaw_deg = self.apply_deadzone(yaw_deg,self.yaw_deadzone)
        pitch_deg = self.apply_deadzone(pitch_deg,self.pitch_deadzone)

        steering = self.steering_deg(-yaw_deg)
        throttle = -self.scale(pitch_deg, self.max_pitch)

        #Using Twist messages for steering/throttle now 
        cmd = Twist()
        cmd.angular.z = steering
        cmd.linear.x = throttle
        self.publisher.publish(cmd)

        self.get_logger().info(
            f"yaw={yaw_deg:.1f} -> steer={steering:.2f},"
            f"pitch={pitch_deg:.1f} -> throttle={throttle:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HeadtrackerMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
