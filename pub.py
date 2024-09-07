import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ugv_interface.action import Behavior
import json
import sys

class BehaviorClient(Node):

    def __init__(self):
        super().__init__('behavior_client')
        self._action_client = ActionClient(self, Behavior, 'behavior')

    def send_goal(self, command):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = Behavior.Goal()
        goal_msg.command = command

        self.get_logger().info('Sending goal...')
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))

def get_data_from_input(input_choice):
    if input_choice == "1":
        return [{"T": 1, "type": "save_map_point", "data": "a"}]
    elif input_choice == "2":
        return [{"T": 1, "type": "save_map_point", "data": "b"}]
    elif input_choice == "3":
        return [{"T": 1, "type": "pub_nav_point", "data": "a"}]
    elif input_choice == "4":
        return [{"T": 1, "type": "pub_nav_point", "data": "b"}]
    else:
        print("Invalid input. Please choose a number between 1 and 4.")
        sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    behavior_client = BehaviorClient()
    
    if len(sys.argv) != 2:
        print("Usage: python your_script.py <choice>")
        sys.exit(1)
    
    input_choice = sys.argv[1]
    data = get_data_from_input(input_choice)
    
    json_str = json.dumps(data)
    
    behavior_client.send_goal(json_str)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

