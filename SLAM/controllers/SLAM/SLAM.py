#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

try:
    from controller import Robot, Motor, DistanceSensor
except ImportError:
    print("Error: Could not import Webots controller module")
    exit(1)


class PioneerSLAMController(Node):
    def __init__(self):
        super().__init__('pioneer_slam_controller')
        
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Get motors
        self.left_motor = self.robot.getDevice('left wheel')
        self.right_motor = self.robot.getDevice('right wheel')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Get lidar sensor (adjust name if different in your world)
        self.lidar = self.robot.getDevice('Sick LMS 291')
        if self.lidar is None:
            self.get_logger().error("Could not find lidar! Check device name in Webots.")
        else:
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()
        
        # Robot parameters
        self.wheel_radius = 0.0975  # Pioneer 3-DX wheel radius in meters
        self.wheel_distance = 0.33  # Distance between wheels in meters
        
        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # ROS 2 publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ROS 2 subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Pioneer SLAM Controller initialized')
        
    def cmd_vel_callback(self, msg):
        """Convert Twist message to wheel velocities"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Differential drive kinematics
        left_vel = (linear - angular * self.wheel_distance / 2.0) / self.wheel_radius
        right_vel = (linear + angular * self.wheel_distance / 2.0) / self.wheel_radius
        
        self.left_motor.setVelocity(left_vel)
        self.right_motor.setVelocity(right_vel)
    
    def publish_laser_scan(self):
        """Publish lidar data as LaserScan message"""
        if self.lidar is None:
            return
            
        ranges = self.lidar.getRangeImage()
        if ranges is None:
            return
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_laser'
        
        # Sick LMS 291 specifications (adjust if using different sensor)
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = math.pi / 180.0  # 1 degree
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 80.0
        
        scan.ranges = [float(r) if r != float('inf') else scan.range_max for r in ranges]
        
        self.scan_pub.publish(scan)
    
    def update_odometry(self):
        """Update and publish odometry"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Get wheel velocities
        left_vel = self.left_motor.getVelocity() * self.wheel_radius
        right_vel = self.right_motor.getVelocity() * self.wheel_radius
        
        # Calculate robot velocity
        linear_vel = (left_vel + right_vel) / 2.0
        angular_vel = (right_vel - left_vel) / self.wheel_distance
        
        # Update pose
        delta_theta = angular_vel * dt
        delta_x = linear_vel * math.cos(self.theta + delta_theta/2) * dt
        delta_y = linear_vel * math.sin(self.theta + delta_theta/2) * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Quaternion from yaw
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(odom)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
        self.last_time = current_time
    
    def step(self):
        """Main control loop step"""
        if self.robot.step(self.timestep) != -1:
            self.publish_laser_scan()
            self.update_odometry()
            return True
        return False


def main(args=None):
    rclpy.init(args=args)
    
    controller = PioneerSLAMController()
    
    try:
        while rclpy.ok() and controller.step():
            rclpy.spin_once(controller, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()