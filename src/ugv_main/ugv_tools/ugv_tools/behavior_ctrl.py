import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from ugv_interface.action import Behavior
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry 

import math
import threading
import json
import queue

a = "point_a" 
b = "point_b" 
c = "point_c" 
d = "point_d" 
e = "point_e" 
f = "point_f" 
g = "point_g" 

class BehaviorController(Node):
    def __init__(self):
        super().__init__('behavior_ctrl')     
        # Create a subscription to the /odom topic to get the odometry data
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Create a subscription to the /robot_pose topic to get the robot's current pose
        self.create_subscription(PoseStamped, '/robot_pose', self.robot_pose_callback, 10) 
        # Create an action server to handle the behavior action
        self.behavior_action_server = ActionServer(self, Behavior, 'behavior', self.execute_callback)
        # Create a publisher to the /cmd_vel topic to send velocity commands to the robot
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Create a publisher to the /goal_pose topic to send goal poses to the robot
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # Initialize the distance and yaw variables
        self.distance = Pose().position
        self.yaw = 0.0
        self.current_pose = None
        self.behavior_done = None
        self.map_pose = None
        self.points = {}
        
        # Create a queue to store the commands
        self.command_queue = queue.Queue()
        # Create a lock to ensure thread safety
        self.lock = threading.Lock()
        # Create a thread to process the commands
        self.executor_thread = threading.Thread(target=self.process_commands)
        # Start the thread
        self.executor_thread.start()

    def robot_pose_callback(self, msg):
        # Store the current pose of the robot
        self.map_pose = msg.pose
                
    def execute_callback(self, goal_handle):
        # Log the start of the goal execution
        self.get_logger().info('Executing goal...')
        #print(goal_handle.request.command)
        
        # Parse the command from the goal request
        json_list = json.loads(goal_handle.request.command)
        
        # Lock the thread to ensure thread safety
        with self.lock:
            # Iterate through the commands
            for json_data in json_list:
                command_type = json_data['type']
                data_value = json_data['data']
                
                # Create the command string
                if command_type == "stop":
                    command_string = "self.stop()"
                else:
                    command_string = f"self.{command_type}({data_value})"
                
                # Put the command in the queue
                self.command_queue.put(command_string)
        
        # Succeed the goal
        goal_handle.succeed()
        result = Behavior.Result()
        result.result = True
       
        return result        
        
    def process_commands(self):
        # Process the commands in the queue
        while rclpy.ok():
            command_string = self.command_queue.get()
            if command_string is None:
                break  
            self.execute_behavior(command_string)
            self.command_queue.task_done()
    
    def execute_behavior(self, command_string):
        # Execute the command
        print(command_string)
        try:
            exec(command_string)    
        except Exception as e:
            self.get_logger().error(f"Error executing behavior: {e}")
            self.get_logger().error(f"Executed command: {command_string}")
    
    def odom_callback(self, msg):
        # Get the orientation of the robot
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        q0 = msg.pose.pose.orientation.w

        # Calculate the yaw of the robot
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
        
        # Store the distance and yaw of the robot
        self.distance = msg.pose.pose.position
        self.yaw = math.atan2(siny_cosp, cosy_cosp)  
                               
    def drive_on_heading(self, distance):
        # Drive the robot on a heading
        print('Drive on heading')
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  
        twist_msg.angular.z = 0.0
        
        # Store the start distance
        start_distance = self.distance
        print('start distance:', start_distance)
           
        # Calculate the delta distance
        delta_distance = 0
        while abs(delta_distance) < abs(distance):
            diff_x = self.distance.x - start_distance.x
            diff_y = self.distance.y - start_distance.y
            delta_distance = math.hypot(diff_x, diff_y)
            
            print('now distance:', self.distance.x, self.distance.y)    
            print('Distance moved:', delta_distance)
            self.velocity_publisher.publish(twist_msg)
        self.stop()

    def back_up(self, distance):
        # Back up the robot
        print('Back up')
        twist_msg = Twist()
        twist_msg.linear.x = -0.2  
        twist_msg.angular.z = 0.0
        
        # Store the start distance
        start_distance = self.distance
        print('start distance:', start_distance)
  
        # Calculate the delta distance
        delta_distance = 0
        while abs(delta_distance) < abs(distance):
            diff_x = self.distance.x - start_distance.x
            diff_y = self.distance.y - start_distance.y
            delta_distance = math.hypot(diff_x, diff_y)
            
            print('now distance:', self.distance.x, self.distance.y)    
            print('Distance moved:', delta_distance)
            self.velocity_publisher.publish(twist_msg)
        self.stop()
        
    def spin(self, angle):
        # Spin the robot
        print('Spin')
        twist_msg = Twist()
        
        # Determine the direction of the spin
        if angle > 0:
            twist_msg.angular.z = 0.3
        else:
            twist_msg.angular.z = -0.3     
               
        twist_msg.linear.x = 0.0

        # Store the start yaw
        start_yaw = self.yaw
        # Calculate the target yaw
        target_yaw = (start_yaw + math.radians(angle)) % (2 * math.pi)
        
        # Calculate the delta yaw
        delta_yaw = 0.0
        while abs(delta_yaw) < abs(math.radians(angle)):
            delta_yaw = self.yaw - start_yaw
            delta_yaw = (delta_yaw + math.pi) % (2 * math.pi) - math.pi 
            print(f'Rotated angle: {math.degrees(delta_yaw)} degrees')
            self.velocity_publisher.publish(twist_msg)
        self.stop()
        
    def stop(self):
        # Stop the robot
        print('Stop')
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.velocity_publisher.publish(twist_msg)
        self.behavior_done = True 
        
    def destroy_node(self):
        # Put a None command in the queue to stop the thread
        self.command_queue.put(None)  
        # Wait for the thread to finish
        self.executor_thread.join()  
        super().destroy_node()

    def save_map_point(self, point):
        # Save a map point
        if self.map_pose is not None:
            point_pose = Pose()
            point_pose = self.map_pose
            self.points[point] = point_pose
            self.get_logger().info(f'Added point "{point}": {point_pose}')
            self.save_points_to_file()  # Save to file whenever points are updated
        else:
            self.get_logger().warn('No current pose available to create map point.')
        self.behavior_done = True

    def save_points_to_file(self):
        # Save the map points to a file
        with open('/home/ws/ugv_ws/map_points.txt', 'w') as file:
            for point_name, pose in self.points.items():
                file.write(f'{point_name}: Position(x={pose.position.x}, y={pose.position.y}, z={pose.position.z}), Orientation(x={pose.orientation.x}, y={pose.orientation.y}, z={pose.orientation.z}, w={pose.orientation.w})\n')
        self.get_logger().info('Saved points to map_points.txt')

    def pub_nav_point(self, point):
        # Publish a navigation point
        if point in self.points:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose = self.points[point]
            self.goal_publisher.publish(goal_pose)
            self.get_logger().info(f'Sent goal to /goal_pose: {goal_pose.pose.position}')
        else:
            self.get_logger().warn(f'Point "{point}" not found in saved points.')
        self.behavior_done = True
                    
def main(args=None):
    # Initialize the ROS 2 node
    rclpy.init(args=args)
    node = BehaviorController()
    # Spin the node
    rclpy.spin(node)
    # Shutdown the node
    rclpy.shutdown()

if __name__ == '__main__':
    main()

