

###############################
#
# OpenTrackBridgeNode Module 
# Handles UDP listening + publishing the parsed data for the next node to read
# Refer to README for complete package structure 
#
##############################
import socket
import struct
import math
from tf_transformations import quaternion_from_euler #This import requires a pip install I think
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
class OpenTrackBridgeNode(Node):

    def __init__(self):
        super().__init__('opentrack_bridge_node')

        #Set up a non-blocking udp server to read on Port 5555
        self.udp_ip = '' #This may have to change to the IP of the pi im unsure, right now it just listens on all interfaces
        self.udp_port = 5555
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip,self.udp_port))
        self.sock.setblocking(False)
        self.get_logger().info(f'Listening for UDP on port {self.udp_port}')

        #publisher and timer
        self.publisher = self.create_publisher(PoseStamped, 'head_pose',10)
        self.timer = self.create_timer(0.005,self.timer_callback)

        self.get_logger().info('OpenTrack Bridge Node Started!')


    def timer_callback(self):

        #Try/catch to actually read the packets 
        try:
            data, addr = self.sock.recvfrom(1024)
        except BlockingIOError:
            return
        except Exception as e:
            self.get_logger().warn(f'UDP recv error: {e}')
            return
        #OpenTrack outputs the packets as 24 bytes
        if len(data) != 48:
            self.get_logger().warn(f"Error: Expected 24 packets got:{len(data)}")
            return

        #Yaw, pitch, and roll are the first 3 values in the packet
        values  = struct.unpack('<6d', data)

        yaw_deg = values[0]
        pitch_deg = values[1]
        roll_deg = values[2]

        #Convert from deg -> rad
        yaw = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        roll = math.radians(roll_deg)
        #Turn into quaternions because ROS uses this as its standard format (honestly not sure if this is necessary but oh well)
        qx,qy,qz,qw = quaternion_from_euler(roll, pitch, yaw)

        #Create PoseStamped message (coordinate frame w/ timestamp)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'head_tracker'

        #We only care about these if we're doing rotation movements which we aren't right now
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        #And away goes the PoseStamped message!
        #self.publisher.publish(msg)
        self.get_logger().info(f'Recieved {len(data)} bytes from {addr} containing: {yaw_deg,pitch_deg,roll_deg}')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenTrackBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()
