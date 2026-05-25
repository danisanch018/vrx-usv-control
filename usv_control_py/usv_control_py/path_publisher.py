#!/usr/bin/env python3
import rclpy
import csv 
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped

class WaypointPathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        
        #Hexagonal or S path
        # Declare parameters for waypoints
        waypoint_x_descriptor = ParameterDescriptor(description='List of X coordinates for waypoints')
        waypoint_y_descriptor = ParameterDescriptor(description='List of Y coordinates for waypoints')

        self.declare_parameter('waypoints_x', [0.0], waypoint_x_descriptor)
        self.declare_parameter('waypoints_y', [0.0], waypoint_y_descriptor)

        # Get waypoints from parameters
        waypoints_x = self.get_parameter('waypoints_x').get_parameter_value().double_array_value
        waypoints_y = self.get_parameter('waypoints_y').get_parameter_value().double_array_value

        if len(waypoints_x) != len(waypoints_y):
            self.get_logger().error("Mismatch in number of X and Y waypoints. Path will be empty.")
            self.waypoints = []
        else:
            self.waypoints = []
            for i in range(len(waypoints_x)):
                self.waypoints.append([waypoints_x[i], waypoints_y[i]])

        if not self.waypoints:
            self.get_logger().warn("No waypoints loaded. Please check your trajectory.yaml or parameter settings.")
        else:
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from parameters.")
            for i, wp in enumerate(self.waypoints):
                self.get_logger().debug(f"Waypoint {i}: x={wp[0]}, y={wp[1]}")

        #open csv file, you can change the file name if you want 
        self.csv_path = 'datos_csv.csv'
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y'])
        #Subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered',self.odomCallback, 10)
        #Publisher
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        #Timer
        self.timer = self.create_timer(1.0, self.publish_path)
        
        self.get_logger().info("Path publisher initiated.")

    def odomCallback(self, msg):
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y 
        try:
            self.csv_writer.writerow([self.x, self.y])
            
        except Exception as e:
            self.get_logger().error(f"Error writing to CSV: {e}")

    def publish_path(self):
       
        path_msg = Path()
        

        path_msg.header.frame_id = "odom" 
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = path_msg.header.frame_id
            pose.header.stamp = path_msg.header.stamp
            
            # Position
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = 0.0
            
            # Orientation
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            
            # Add the point to the list of Path
            path_msg.poses.append(pose)
            
        # Publish
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPathPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Closing node...")
    finally:
        node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

