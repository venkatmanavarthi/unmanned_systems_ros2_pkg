import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from unmanned_systems_ros2_pkg import TurtleBotNode, ProNav

class PursuitNode(Node):
    def __init__(self):
        super().__init__('pursuit_node')

        self.turtlebot_pursuer = TurtleBotNode.TurtleBotNode('turtle', 'pursuer')
        self.pro_nav = ProNav.ProNav(3.0)  # Initialize Proportional Navigation with a suitable gain

        self.lidar_subscriber = self.create_subscription(
            LaserScan, 'evader/scan', self.lidar_callback, 10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle/cmd_vel', 10)

        self.evader_position = np.array([0.0, 0.0])  # Initial assumed position of the evader
        self.evader_velocity = np.array([0.0, 0.0])  # Initial assumed velocity of the evader

        self.pn_constant = 1.0  # Fine-tune this based on system dynamics

        # Keep track of the pursuer's previous position for velocity estimation
        self.prev_pursuer_position = np.copy(self.turtlebot_pursuer.current_position)

    def lidar_callback(self, msg: LaserScan):
        # Assuming that Lidar data gives ranges in the forward direction
        # Extracting the nearest point as the evader's position
        print(msg)
        min_range_index = np.argmin(msg.ranges)
        min_range = msg.ranges[min_range_index]

        # Convert Lidar data to Cartesian coordinates
        angle_to_nearest_point = msg.angle_min + min_range_index * msg.angle_increment
        x_evader = min_range * np.cos(angle_to_nearest_point)
        y_evader = min_range * np.sin(angle_to_nearest_point)

        # Estimate evader's velocity using the constant velocity model
        delta_t = 0.1  # Adjust as needed
        if hasattr(self, 'prev_evader_position'):
            self.evader_velocity = np.array([(x_evader - self.prev_evader_position[0]) / delta_t,
                                             (y_evader - self.prev_evader_position[1]) / delta_t])
        else:
            self.evader_velocity = np.array([0.0, 0.0])  # Initial velocity

        # Update evader's position
        self.evader_position = np.array([x_evader, y_evader])

        # Store current position for velocity estimation in the next callback
        self.prev_evader_position = np.copy(self.evader_position)

    def pursue_evader(self):
        while rclpy.ok():
            self.get_logger().info("Pursuing evader...")

            # Implement True Proportional Navigation here
            desired_heading = np.arctan2(
                self.evader_position[1] - self.turtlebot_pursuer.current_position[1],
                self.evader_position[0] - self.turtlebot_pursuer.current_position[0]
            )

            # Use True Proportional Navigation to compute flight path rate
            flight_path_rate = self.pro_nav.true_pro_nav(
                self.turtlebot_pursuer.current_position,
                self.evader_position,
                0.1,  # dt, adjust as needed
                self.evader_velocity,
                (self.turtlebot_pursuer.current_position - self.prev_pursuer_position) / delta_t,
                True,  # Use own heading
                0.0  # Placeholder for LOS value, adjust as needed
            )

            # Publish the pursuit command velocity
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.1  # Adjust linear velocity as needed
            cmd_vel_msg.angular.z = flight_path_rate
            self.cmd_vel_publisher.publish(cmd_vel_msg)

            # Check for collision (you need to implement your own collision detection logic)
            if self.check_collision():
                self.get_logger().info("Collision detected! Evader captured.")
                break

            rclpy.spin_once(self)

    def check_collision(self):
        # Implement collision detection logic based on your system's requirements
        # Return True if a collision is detected, otherwise return False
        return False

def main():
    rclpy.init()
    pursuit_node = PursuitNode()
    pursuit_node.pursue_evader()
    rclpy.spin(pursuit_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
