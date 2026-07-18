import json
import math
import os
import queue
import re
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ugv_msgs.action import Behavior

POINT_ALIASES = {name: f'point_{name}' for name in 'abcdefg'}

CONTROL_PERIOD_SEC = 0.05
LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.3
YAW_TOLERANCE_RAD = math.radians(2.0)
DISTANCE_TOLERANCE_M = 0.02
NAV_SERVER_TIMEOUT_SEC = 5.0
DEFAULT_MAP_POINTS_FILE = '/home/ws/ugv_ws/map_points.json'
LEGACY_MAP_POINTS_FILE = '/home/ws/ugv_ws/map_points.txt'


def _normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _copy_pose(pose):
    copied = Pose()
    copied.position.x = pose.position.x
    copied.position.y = pose.position.y
    copied.position.z = pose.position.z
    copied.orientation.x = pose.orientation.x
    copied.orientation.y = pose.orientation.y
    copied.orientation.z = pose.orientation.z
    copied.orientation.w = pose.orientation.w
    return copied


def _pose_to_dict(pose):
    return {
        'position': {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
        },
        'orientation': {
            'x': pose.orientation.x,
            'y': pose.orientation.y,
            'z': pose.orientation.z,
            'w': pose.orientation.w,
        },
    }


def _dict_to_pose(data):
    pose = Pose()
    position = data.get('position', {})
    orientation = data.get('orientation', {})
    pose.position.x = float(position.get('x', 0.0))
    pose.position.y = float(position.get('y', 0.0))
    pose.position.z = float(position.get('z', 0.0))
    pose.orientation.x = float(orientation.get('x', 0.0))
    pose.orientation.y = float(orientation.get('y', 0.0))
    pose.orientation.z = float(orientation.get('z', 0.0))
    pose.orientation.w = float(orientation.get('w', 1.0))
    return pose


class BehaviorController(Node):
    def __init__(self):
        super().__init__('behavior_ctrl')
        self._callback_group = ReentrantCallbackGroup()

        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            PoseStamped, '/robot_pose', self.robot_pose_callback, 10,
            callback_group=self._callback_group,
        )
        self.behavior_action_server = ActionServer(
            self,
            Behavior,
            'behavior',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._callback_group,
        )
        self._nav_goal_handle = None
        self._nav_goal_lock = threading.Lock()

        self._state_lock = threading.Lock()
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._yaw = 0.0
        self._map_pose = None

        self.map_points_file = DEFAULT_MAP_POINTS_FILE
        self.points = {}
        self._points_lock = threading.Lock()
        self.load_points_from_file()

        self._cancel_event = threading.Event()
        self._feedback_lock = threading.Lock()
        self._feedback_status = ''
        self._feedback_progress = 0.0
        self.command_queue = queue.Queue()
        self.executor_thread = threading.Thread(
            target=self.process_commands, name='behavior_executor', daemon=True,
        )
        self.executor_thread.start()

        self._handlers = {
            'stop': self._handle_stop,
            'drive_on_heading': self._handle_drive_on_heading,
            'back_up': self._handle_back_up,
            'spin': self._handle_spin,
            'save_map_point': self._handle_save_map_point,
            'pub_nav_point': self._handle_pub_nav_point,
        }

    def _make_result(self, ok, message=''):
        result = Behavior.Result()
        result.result = bool(ok)
        if hasattr(result, 'message'):
            result.message = str(message)
        return result

    def _set_feedback(self, status, progress=0.0):
        with self._feedback_lock:
            self._feedback_status = status
            self._feedback_progress = max(0.0, min(1.0, float(progress)))

    def _publish_feedback(self, goal_handle):
        feedback_msg = Behavior.Feedback()
        with self._feedback_lock:
            status = self._feedback_status
            progress = self._feedback_progress
        feedback_msg.feedback = True
        if hasattr(feedback_msg, 'status'):
            feedback_msg.status = status
        if hasattr(feedback_msg, 'progress'):
            feedback_msg.progress = progress
        goal_handle.publish_feedback(feedback_msg)

    def goal_callback(self, _goal_request):
        # Accept and queue for serial execution.
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self._cancel_event.set()
        self._cancel_navigation()
        return CancelResponse.ACCEPT

    def robot_pose_callback(self, msg):
        with self._state_lock:
            self._map_pose = _copy_pose(msg.pose)

    def odom_callback(self, msg):
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        q0 = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        with self._state_lock:
            self._odom_x = msg.pose.pose.position.x
            self._odom_y = msg.pose.pose.position.y
            self._yaw = math.atan2(siny_cosp, cosy_cosp)

    def _get_odom_xy(self):
        with self._state_lock:
            return self._odom_x, self._odom_y

    def _get_yaw(self):
        with self._state_lock:
            return self._yaw

    def _get_map_pose(self):
        with self._state_lock:
            if self._map_pose is None:
                return None
            return _copy_pose(self._map_pose)

    def execute_callback(self, goal_handle):
        try:
            json_list = json.loads(goal_handle.request.command)
        except json.JSONDecodeError as exc:
            message = f'Invalid command JSON: {exc}'
            self.get_logger().error(message)
            goal_handle.abort()
            return self._make_result(False, message)

        if not isinstance(json_list, list):
            json_list = [json_list]

        try:
            commands = [self._normalize_command(item) for item in json_list]
        except (KeyError, TypeError, ValueError) as exc:
            message = f'Invalid command entry: {exc}'
            self.get_logger().error(message)
            goal_handle.abort()
            return self._make_result(False, message)

        # A stop-only goal preempts whatever is running.
        if commands and all(command_type == 'stop' for command_type, _ in commands):
            self._cancel_event.set()
            self._cancel_navigation()
            self._drain_command_queue()
            self.stop()
            self._set_feedback('stopped', 1.0)
            self._publish_feedback(goal_handle)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_result(False, 'canceled')
            goal_handle.succeed()
            return self._make_result(True, 'stopped')

        self._set_feedback('queued', 0.0)
        self._publish_feedback(goal_handle)

        batch_done = threading.Event()
        batch_result = {'ok': True, 'canceled': False, 'message': ''}
        self.command_queue.put({
            'commands': commands,
            'done': batch_done,
            'result': batch_result,
        })

        while not batch_done.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                self._cancel_event.set()
                self._cancel_navigation()
                with self._feedback_lock:
                    progress = self._feedback_progress
                self._set_feedback('canceling', progress)
            self._publish_feedback(goal_handle)

        with self._feedback_lock:
            final_progress = self._feedback_progress

        if goal_handle.is_cancel_requested or batch_result['canceled']:
            self._set_feedback('canceled', final_progress)
            self._publish_feedback(goal_handle)
            goal_handle.canceled()
            return self._make_result(False, batch_result.get('message') or 'canceled')

        if batch_result['ok']:
            self._set_feedback('done', 1.0)
            self._publish_feedback(goal_handle)
            goal_handle.succeed()
            return self._make_result(True, batch_result.get('message') or 'ok')

        message = batch_result.get('message') or 'aborted'
        self._set_feedback('aborted', final_progress)
        self._publish_feedback(goal_handle)
        goal_handle.abort()
        return self._make_result(False, message)

    def _normalize_command(self, json_data):
        if not isinstance(json_data, dict):
            raise TypeError('command entry must be an object')
        command_type = str(json_data['type'])
        if command_type not in self._handlers:
            raise ValueError(f'unknown command type: {command_type}')
        return command_type, json_data.get('data', 0)

    def _drain_command_queue(self):
        while True:
            try:
                job = self.command_queue.get_nowait()
            except queue.Empty:
                break
            if job is None:
                self.command_queue.put(None)
                break
            if isinstance(job, dict):
                job['result']['canceled'] = True
                job['result']['ok'] = False
                job['result']['message'] = 'preempted by stop'
                job['done'].set()
            self.command_queue.task_done()

    def process_commands(self):
        while rclpy.ok():
            job = self.command_queue.get()
            try:
                if job is None:
                    break

                self._cancel_event.clear()
                commands = job['commands']
                batch_result = job['result']
                total = max(len(commands), 1)

                for index, (command_type, data_value) in enumerate(commands):
                    if self._cancel_event.is_set():
                        batch_result['canceled'] = True
                        batch_result['ok'] = False
                        batch_result['message'] = 'canceled'
                        break
                    try:
                        self._dispatch(command_type, data_value, index, total)
                    except Exception as exc:
                        message = f'{command_type} failed: {exc}'
                        self.get_logger().error(message)
                        batch_result['ok'] = False
                        batch_result['message'] = message
                        break
                else:
                    if not batch_result.get('message'):
                        batch_result['message'] = 'ok'
            finally:
                if isinstance(job, dict):
                    job['done'].set()
                self.command_queue.task_done()

    def _dispatch(self, command_type, data_value, index=0, total=1):
        self._current_cmd_index = index
        self._current_cmd_total = total
        self._handlers[command_type](data_value)

    def _command_progress(self, local_progress=0.0):
        index = getattr(self, '_current_cmd_index', 0)
        total = max(getattr(self, '_current_cmd_total', 1), 1)
        return (index + max(0.0, min(1.0, local_progress))) / total

    def _handle_stop(self, _data):
        self._set_feedback('stop', self._command_progress(1.0))
        self._cancel_navigation()
        self.stop()

    def _handle_drive_on_heading(self, data):
        self.drive_on_heading(float(data))

    def _handle_back_up(self, data):
        self.back_up(float(data))

    def _handle_spin(self, data):
        self.spin(float(data))

    def _handle_save_map_point(self, data):
        point = self._resolve_point_name(data)
        self._set_feedback(f'save_map_point:{point}', self._command_progress(0.0))
        self.save_map_point(point)
        self._set_feedback(f'save_map_point:{point}', self._command_progress(1.0))

    def _handle_pub_nav_point(self, data):
        point = self._resolve_point_name(data)
        self._set_feedback(f'pub_nav_point:{point}', self._command_progress(0.0))
        self.pub_nav_point(point)
        self._set_feedback(f'pub_nav_point:{point}', self._command_progress(1.0))

    def _resolve_point_name(self, data):
        if not isinstance(data, str):
            raise ValueError(f'point name must be a string, got {data!r}')
        name = data.strip()
        if not name:
            raise ValueError('point name is empty')
        return POINT_ALIASES.get(name, name)

    def _motion_should_stop(self):
        return self._cancel_event.is_set() or not rclpy.ok()

    def drive_on_heading(self, distance):
        speed = LINEAR_SPEED if distance >= 0.0 else -LINEAR_SPEED
        target = abs(float(distance))
        self._set_feedback(f'drive_on_heading:0.00/{target:.2f}m', self._command_progress(0.0))
        if target <= DISTANCE_TOLERANCE_M:
            self._set_feedback(
                f'drive_on_heading:{target:.2f}/{target:.2f}m',
                self._command_progress(1.0),
            )
            self.stop()
            return

        start_x, start_y = self._get_odom_xy()
        timeout_sec = max(5.0, target / LINEAR_SPEED * 2.5 + 2.0)
        deadline = time.monotonic() + timeout_sec
        twist_msg = Twist()
        twist_msg.linear.x = speed
        timed_out = False
        moved = 0.0

        while not self._motion_should_stop():
            x, y = self._get_odom_xy()
            moved = math.hypot(x - start_x, y - start_y)
            local = min(1.0, moved / target) if target > 0.0 else 1.0
            self._set_feedback(
                f'drive_on_heading:{moved:.2f}/{target:.2f}m',
                self._command_progress(local),
            )
            if moved >= target - DISTANCE_TOLERANCE_M:
                break
            if time.monotonic() > deadline:
                timed_out = True
                break
            self.velocity_publisher.publish(twist_msg)
            time.sleep(CONTROL_PERIOD_SEC)

        self.stop()
        if self._cancel_event.is_set():
            return
        if timed_out:
            raise RuntimeError(
                f'drive_on_heading timed out after {moved:.3f}m / {target:.3f}m'
            )
        self._set_feedback('drive_on_heading:done', self._command_progress(1.0))

    def back_up(self, distance):
        target = abs(float(distance))
        self._set_feedback(f'back_up:0.00/{target:.2f}m', self._command_progress(0.0))
        if target <= DISTANCE_TOLERANCE_M:
            self._set_feedback(
                f'back_up:{target:.2f}/{target:.2f}m',
                self._command_progress(1.0),
            )
            self.stop()
            return

        start_x, start_y = self._get_odom_xy()
        timeout_sec = max(5.0, target / LINEAR_SPEED * 2.5 + 2.0)
        deadline = time.monotonic() + timeout_sec
        twist_msg = Twist()
        twist_msg.linear.x = -LINEAR_SPEED
        timed_out = False
        moved = 0.0

        while not self._motion_should_stop():
            x, y = self._get_odom_xy()
            moved = math.hypot(x - start_x, y - start_y)
            local = min(1.0, moved / target) if target > 0.0 else 1.0
            self._set_feedback(
                f'back_up:{moved:.2f}/{target:.2f}m',
                self._command_progress(local),
            )
            if moved >= target - DISTANCE_TOLERANCE_M:
                break
            if time.monotonic() > deadline:
                timed_out = True
                break
            self.velocity_publisher.publish(twist_msg)
            time.sleep(CONTROL_PERIOD_SEC)

        self.stop()
        if self._cancel_event.is_set():
            return
        if timed_out:
            raise RuntimeError(f'back_up timed out after {moved:.3f}m / {target:.3f}m')
        self._set_feedback('back_up:done', self._command_progress(1.0))

    def spin(self, angle):
        angle = float(angle)
        target_deg = abs(angle)
        self._set_feedback(f'spin:0.0/{target_deg:.1f}deg', self._command_progress(0.0))
        if target_deg < math.degrees(YAW_TOLERANCE_RAD):
            self._set_feedback(
                f'spin:{target_deg:.1f}/{target_deg:.1f}deg',
                self._command_progress(1.0),
            )
            self.stop()
            return

        target_yaw = self._get_yaw() + math.radians(angle)
        timeout_sec = max(5.0, abs(math.radians(angle)) / ANGULAR_SPEED * 2.5 + 2.0)
        deadline = time.monotonic() + timeout_sec
        twist_msg = Twist()
        timed_out = False
        error = math.radians(angle)

        while not self._motion_should_stop():
            error = _normalize_angle(target_yaw - self._get_yaw())
            rotated = target_deg - abs(math.degrees(error))
            rotated = max(0.0, min(target_deg, rotated))
            local = min(1.0, rotated / target_deg) if target_deg > 0.0 else 1.0
            self._set_feedback(
                f'spin:{rotated:.1f}/{target_deg:.1f}deg',
                self._command_progress(local),
            )
            if abs(error) <= YAW_TOLERANCE_RAD:
                break
            if time.monotonic() > deadline:
                timed_out = True
                break
            twist_msg.angular.z = ANGULAR_SPEED if error > 0.0 else -ANGULAR_SPEED
            self.velocity_publisher.publish(twist_msg)
            time.sleep(CONTROL_PERIOD_SEC)

        self.stop()
        if self._cancel_event.is_set():
            return
        if timed_out:
            raise RuntimeError(
                f'spin timed out with remaining error {math.degrees(error):.1f} deg'
            )
        self._set_feedback('spin:done', self._command_progress(1.0))

    def stop(self):
        if not rclpy.ok():
            return
        try:
            twist_msg = Twist()
            self.velocity_publisher.publish(twist_msg)
        except Exception:
            # Context may already be shutting down (e.g. Ctrl+C).
            pass

    def _cancel_navigation(self):
        with self._nav_goal_lock:
            goal_handle = self._nav_goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f'Failed to cancel navigate_to_pose: {exc}')

    @staticmethod
    def _goal_status_name(status):
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, str(status))

    def destroy_node(self):
        self._cancel_event.set()
        self._cancel_navigation()
        self.command_queue.put(None)
        if self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2.0)
        self.stop()
        try:
            super().destroy_node()
        except Exception:
            pass

    def load_points_from_file(self):
        if os.path.exists(self.map_points_file):
            try:
                with open(self.map_points_file, 'r', encoding='utf-8') as file:
                    raw = json.load(file)
            except (OSError, json.JSONDecodeError) as exc:
                self.get_logger().error(f'Failed to load {self.map_points_file}: {exc}')
                return
            loaded = {
                str(name): _dict_to_pose(pose_data)
                for name, pose_data in raw.items()
                if isinstance(pose_data, dict)
            }
            with self._points_lock:
                self.points = loaded
            self.get_logger().info(
                f'Loaded {len(loaded)} points from {self.map_points_file}'
            )
            return

        if os.path.exists(LEGACY_MAP_POINTS_FILE):
            loaded = self._load_legacy_points(LEGACY_MAP_POINTS_FILE)
            with self._points_lock:
                self.points = loaded
            if loaded:
                self.save_points_to_file(loaded)
                self.get_logger().info(
                    f'Migrated {len(loaded)} points from {LEGACY_MAP_POINTS_FILE} '
                    f'to {self.map_points_file}'
                )
            return

        self.get_logger().info(f'No map points file at {self.map_points_file}')

    def _load_legacy_points(self, path):
        point_line_re = re.compile(
            r'^([^:]+):\s*Position\(x=([^,]+),\s*y=([^,]+),\s*z=([^)]+)\),\s*'
            r'Orientation\(x=([^,]+),\s*y=([^,]+),\s*z=([^,]+),\s*w=([^)]+)\)\s*$'
        )
        loaded = {}
        with open(path, 'r', encoding='utf-8') as file:
            for line in file:
                line = line.strip()
                if not line:
                    continue
                match = point_line_re.match(line)
                if not match:
                    self.get_logger().warn(f'Skipping invalid legacy line: {line}')
                    continue
                name, px, py, pz, ox, oy, oz, ow = match.groups()
                pose = Pose()
                pose.position.x = float(px)
                pose.position.y = float(py)
                pose.position.z = float(pz)
                pose.orientation.x = float(ox)
                pose.orientation.y = float(oy)
                pose.orientation.z = float(oz)
                pose.orientation.w = float(ow)
                loaded[name] = pose
        return loaded

    def save_map_point(self, point):
        map_pose = self._get_map_pose()
        if map_pose is None:
            raise RuntimeError('No current pose available to create map point')

        with self._points_lock:
            self.points[point] = map_pose
            points_snapshot = {
                name: _copy_pose(pose) for name, pose in self.points.items()
            }
        self.save_points_to_file(points_snapshot)

    def save_points_to_file(self, points=None):
        if points is None:
            with self._points_lock:
                points = {
                    name: _copy_pose(pose) for name, pose in self.points.items()
                }
        payload = {name: _pose_to_dict(pose) for name, pose in points.items()}
        with open(self.map_points_file, 'w', encoding='utf-8') as file:
            json.dump(payload, file, indent=2, sort_keys=True)
            file.write('\n')

    def pub_nav_point(self, point):
        with self._points_lock:
            pose = self.points.get(point)
            if pose is not None:
                pose = _copy_pose(pose)

        if pose is None:
            raise RuntimeError(f'Point "{point}" not found in saved points')

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = pose

        # Do NOT also publish /goal_pose here: Nav2 often listens to that topic and
        # would start a second goal that aborts this action (status=ABORTED/6).
        if not self._nav_to_pose_client.wait_for_server(timeout_sec=NAV_SERVER_TIMEOUT_SEC):
            raise RuntimeError('navigate_to_pose action server not available')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        initial_distance = None

        def _nav_feedback_cb(feedback_msg):
            nonlocal initial_distance
            distance = float(feedback_msg.feedback.distance_remaining)
            if initial_distance is None or distance > initial_distance:
                initial_distance = max(distance, 1e-3)
            local = 1.0 - min(1.0, max(0.0, distance / initial_distance))
            self._set_feedback(
                f'pub_nav_point:{point}:{distance:.2f}m',
                self._command_progress(local),
            )

        send_future = self._nav_to_pose_client.send_goal_async(
            goal_msg, feedback_callback=_nav_feedback_cb,
        )
        while not send_future.done():
            if self._motion_should_stop():
                return
            time.sleep(CONTROL_PERIOD_SEC)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'navigate_to_pose rejected for point "{point}"')

        with self._nav_goal_lock:
            self._nav_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if self._motion_should_stop():
                goal_handle.cancel_goal_async()
                break
            time.sleep(CONTROL_PERIOD_SEC)

        with self._nav_goal_lock:
            self._nav_goal_handle = None

        if self._cancel_event.is_set():
            return

        if not result_future.done():
            raise RuntimeError(f'navigate_to_pose canceled for point "{point}"')

        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status_name = self._goal_status_name(wrapped.status)
            raise RuntimeError(
                f'navigate_to_pose failed for point "{point}" '
                f'(status={status_name}/{wrapped.status})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
