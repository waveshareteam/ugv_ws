#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vizanti_msgs.srv import Launches
import subprocess

class LaunchesService(Node):

    def __init__(self):
        super().__init__('launches_service')
        self.srv = self.create_service(Launches, '/rosapi/launches', self.get_launches)
        self.get_logger().info('Launches service ready.')

    def get_launches(self, request, response):
        try:
            result = subprocess.run(['ps', 'aux'], capture_output=True, text=True, check=True)
            launches = self.parse_launches(result.stdout)
            response.launches = launches
            self.get_logger().info(f'Request received, responding with: {response.launches}')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error occurred while executing command: {e}")
            response.launches = []
        return response    
        
    def parse_launches(self, ps_output):
        launches = []
        for line in ps_output.splitlines():
            if 'ros2 launch' in line:
                # Extract the launch file name or other relevant information
                parts = line.split()
                launch_index = parts.index('launch') + 1
                if launch_index < len(parts):
                    launches.append(parts[launch_index])
        return launches

def main(args=None):
    rclpy.init(args=args)
    launches_service = LaunchesService()
    rclpy.spin(launches_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

