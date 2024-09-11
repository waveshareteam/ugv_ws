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
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self.robot_pose_callback, 10) 
        self.behavior_action_server = ActionServer(self, Behavior, 'behavior', self.execute_callback)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.distance = Pose().position
        self.yaw = 0.0
        self.current_pose = None
        self.behavior_done = None
        self.map_pose = None
        self.points = {}
        
        self.command_queue = queue.Queue()
        self.lock = threading.Lock()
        self.executor_thread = threading.Thread(target=self.process_commands)
        self.executor_thread.start()

    def robot_pose_callback(self, msg):
        self.map_pose = msg.pose
                
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        #print(goal_handle.request.command)
        
        json_list = json.loads(goal_handle.request.command)
        
        with self.lock:
            for json_data in json_list:
                command_type = json_data['type']
                data_value = json_data['data']
                
                if command_type == "stop":
                    command_string = "self.stop()"
                else:
                    command_string = f"self.{command_type}({data_value})"
                
                self.command_queue.put(command_string)
        
        goal_handle.succeed()
        result = Behavior.Result()
        result.result = True
       
        return result        
        
    def process_commands(self):
        while rclpy.ok():
            command_string = self.command_queue.get()
            if command_string is None:
                break  
            self.execute_behavior(command_string)
            self.command_queue.task_done()
    
    def execute_behavior(self, command_string):
        print(command_string)
        try:
            exec(command_string)    
        except Exception as e:
            self.get_logger().error(f"Error executing behavior: {e}")
            self.get_logger().error(f"Executed command: {command_string}")
    
    def odom_callback(self, msg):
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        q0 = msg.pose.pose.orientation.w

        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
        
        self.distance = msg.pose.pose.position
        self.yaw = math.atan2(siny_cosp, cosy_cosp)  
                               
    def drive_on_heading(self, distance):
        print('Drive on heading')
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  
        twist_msg.angular.z = 0.0
        
        start_distance = self.distance
        print('start distance:', start_distance)
           
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
        print('Back up')
        twist_msg = Twist()
        twist_msg.linear.x = -0.2  
        twist_msg.angular.z = 0.0
        
        start_distance = self.distance
        print('start distance:', start_distance)
  
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
        print('Spin')
        twist_msg = Twist()
        
        if angle > 0:
            twist_msg.angular.z = 0.3
        else:
            twist_msg.angular.z = -0.3     
               
        twist_msg.linear.x = 0.0

        start_yaw = self.yaw
        target_yaw = (start_yaw + math.radians(angle)) % (2 * math.pi)
        
        delta_yaw = 0.0
        while abs(delta_yaw) < abs(math.radians(angle)):
            delta_yaw = self.yaw - start_yaw
            delta_yaw = (delta_yaw + math.pi) % (2 * math.pi) - math.pi 
            print(f'Rotated angle: {math.degrees(delta_yaw)} degrees')
            self.velocity_publisher.publish(twist_msg)
        self.stop()
        
    def stop(self):
        print('Stop')
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.velocity_publisher.publish(twist_msg)
        self.behavior_done = True 
        
    def destroy_node(self):
        self.command_queue.put(None)  
        self.executor_thread.join()  
        super().destroy_node()

    def save_map_point(self, point):
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
        with open('/home/ws/ugv_ws/map_points.txt', 'w') as file:
            for point_name, pose in self.points.items():
                file.write(f'{point_name}: Position(x={pose.position.x}, y={pose.position.y}, z={pose.position.z}), Orientation(x={pose.orientation.x}, y={pose.orientation.y}, z={pose.orientation.z}, w={pose.orientation.w})\n')
        self.get_logger().info('Saved points to map_points.txt')

    def pub_nav_point(self, point):
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
    rclpy.init(args=args)
    node = BehaviorController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

