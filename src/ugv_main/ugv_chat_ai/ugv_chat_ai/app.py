import threading
from flask import Flask, render_template, request, Response
import requests
import json
import re
import logging

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ugv_interface.action import Behavior
from rclpy.duration import Duration

logger = logging.getLogger(__name__)

app = Flask(__name__)

ros_node = None

class ChatAi(Node):
    def __init__(self):
        super().__init__('chat_ai')
        self._action_client = ActionClient(self, Behavior, 'behavior')

    def publish_behavior(self, message):
        msg = String()
        msg.data = message
        self.behavior_publisher.publish(msg)
        logger.debug(f"Published message: {message}")
        
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
        
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/ai', methods=['POST'])
def ai():
    data = request.get_json()
    messages = data.get('messages', [])

    def stream_response():
        for message in generate(messages):
            yield message

    return Response(stream_response(), mimetype='text/event-stream')

def generate(messages):
    stream_content = ""
    one_message = {"role": "assistant", "content": stream_content}
    context = [one_message]
    i = 0

    try:
        data = {"stream": True, "model": 'gemma2', "messages": messages}
        response = requests.post(
            "http://192.168.10.185:11434/api/chat", 
            headers={"Content-Type": "application/json"}, 
            json=data, 
            stream=True
        )
        response.raise_for_status()

        for line in response.iter_lines():
            line_str = line.decode("utf-8")
            if line_str:
                try:
                    line_json = json.loads(line_str)
                    if line_json.get("done", False):
                        break
                    message_content = line_json.get("message", {}).get("content", "")
                    i += 1
                    if i < 40:
                        logger.debug(message_content)
                    elif i == 40:
                        logger.debug("......")
                    one_message["content"] += message_content
                    yield message_content
                except json.JSONDecodeError:
                    logger.debug(f"JSON 解码错误: {line_str}")
                    yield line_str
                    
            elif line_str.strip():
                logger.debug(line_str)
                yield line_str
        print(one_message["content"])
        pattern = r"```json(.*?)```"

        match = re.search(pattern, one_message["content"], re.DOTALL)

        if match:
            json_str = match.group(1).strip()
            json_str = f"[{','.join(json_str.splitlines())}]"
            #command = json.loads(json_str)

            if ros_node:
                #command_str = json.dumps(command)
                ros_node.send_goal(json_str)
                 
    except requests.exceptions.RequestException as e:
        yield f"服务器请求出错: {str(e)}"

def ros_spin(chat_ai):
    rclpy.spin(chat_ai)

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = ChatAi()

    ros_thread = threading.Thread(target=ros_spin, args=(ros_node,))
    ros_thread.start()

    app.run(host='0.0.0.0', port=5000, debug=True)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

