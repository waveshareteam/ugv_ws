import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from tf2_ros import TransformStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

class TransformProcessor(Node):
    def __init__(self):
        super().__init__('apriltag_track_2')

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        self.timer_period = 2.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.check_transform)
        
    def check_transform(self):
        try:
            # Get the current time from the node's clock
            now = rclpy.time.Time()

            # Lookup the transformation from 'dock_frame' to 'map' at the current time
            trans: TransformStamped = self.tf_buffer.lookup_transform('map', 'dock_frame',now)

            # Print the transformation for debugging
            self.get_logger().info(f'Transformation: {trans.transform.translation}')

            # Create the goal message for NavigateToPose action
            goal = PoseStamped()
            goal.header.stamp = self.navigator.get_clock().now().to_msg()
            goal.header.frame_id = 'map'

            goal.pose.position.x = trans.transform.translation.x
            goal.pose.position.y = trans.transform.translation.y
            goal.pose.position.z = trans.transform.translation.z
            goal.pose.orientation = trans.transform.rotation

            # Send the goal to the action server
            self.navigator.goToPose(goal)

        except TransformException as ex:
            self.get_logger().warn(f'Transform error: {str(ex)}')

def main(args=None):
    rclpy.init(args=args)
    node = TransformProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
