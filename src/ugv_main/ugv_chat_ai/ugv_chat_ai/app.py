import queue
import threading
import re
import json
import logging
from pathlib import Path

import requests
from flask import Flask, render_template, request, Response

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterDescriptor
from ugv_msgs.action import Behavior

logger = logging.getLogger(__name__)

app = Flask(__name__)

ros_node = None

PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_PROMPT_FILE = PACKAGE_DIR / "prompt.txt"


def load_prompt_prefix(prompt_file: Path) -> str:
    path = Path(prompt_file)
    if path.is_file():
        return path.read_text(encoding="utf-8").strip()
    logger.warning("prompt file not found: %s", path)
    return ""


def apply_prompt_prefix(messages, prompt_prefix: str):
    """Prepend prompt.txt content to outgoing chat messages."""
    if not prompt_prefix:
        return list(messages)

    out = list(messages)
    for i, msg in enumerate(out):
        if msg.get("role") == "system":
            merged = dict(msg)
            merged["content"] = f"{prompt_prefix}\n\n{merged.get('content', '')}".strip()
            out[i] = merged
            return out

    return [{"role": "system", "content": prompt_prefix}] + out


def _normalize_behavior_entry(obj):
    if not isinstance(obj, dict):
        return None
    if "type" not in obj or "data" not in obj:
        return None
    return {"type": str(obj["type"]), "data": obj["data"]}


def _behavior_list_to_command(parsed):
    if isinstance(parsed, dict):
        entry = _normalize_behavior_entry(parsed)
        if entry:
            return json.dumps([entry])
        return None

    if isinstance(parsed, list):
        entries = []
        for item in parsed:
            entry = _normalize_behavior_entry(item)
            if entry:
                entries.append(entry)
        if entries:
            return json.dumps(entries)
    return None


def _parse_behavior_raw(raw: str):
    if not raw or not raw.strip():
        return None

    raw = raw.strip()
    try:
        return _behavior_list_to_command(json.loads(raw))
    except json.JSONDecodeError:
        pass

    lines = [line.strip().rstrip(",") for line in raw.splitlines() if line.strip()]
    if len(lines) > 1:
        entries = []
        for line in lines:
            entry = _normalize_behavior_entry(json.loads(line))
            if entry:
                entries.append(entry)
        if entries:
            return json.dumps(entries)
    return None


def extract_behavior_command(assistant_content: str):
    """Parse model output into behavior_ctrl command JSON (list of type/data)."""
    if not assistant_content or not assistant_content.strip():
        return None

    text = assistant_content.strip()
    for tag in ("think", "redacted_reasoning"):
        close = f"</{tag}>"
        text = re.sub(
            rf"<{tag}\b[^>]*>.*?{re.escape(close)}",
            "",
            text,
            flags=re.DOTALL | re.IGNORECASE,
        ).strip()

    candidates = []

    for pattern in (
        r"```json\s*(.*?)\s*```",
        r"```\s*(.*?)\s*```",
    ):
        for match in re.finditer(pattern, text, re.DOTALL | re.IGNORECASE):
            candidates.append(match.group(1).strip())

    for match in re.finditer(r"\{[^{}]*\}", text):
        candidates.append(match.group(0).strip())

    candidates.append(text)

    for raw in candidates:
        command = _parse_behavior_raw(raw)
        if command:
            return command

    return None


class ChatAi(Node):
    def __init__(self):
        super().__init__("chat_ai")
        self._action_client = ActionClient(self, Behavior, "behavior")
        self._goal_queue = queue.Queue()

        self.declare_parameter(
            "server_url",
            "http://192.168.9.130:11434/api/chat",
            ParameterDescriptor(description="LLM chat API URL"),
        )
        self.declare_parameter(
            "model",
            "qwen3:8b",
            ParameterDescriptor(description="LLM model name"),
        )
        self.declare_parameter(
            "request_timeout_sec",
            300.0,
            ParameterDescriptor(description="HTTP read timeout for chat stream"),
        )
        self.declare_parameter(
            "prompt_file",
            str(DEFAULT_PROMPT_FILE),
            ParameterDescriptor(description="Path to prompt.txt prefix"),
        )

        self.server_url = self.get_parameter("server_url").value
        self.model = self.get_parameter("model").value
        self.request_timeout_sec = float(
            self.get_parameter("request_timeout_sec").value
        )
        self.prompt_prefix = load_prompt_prefix(
            Path(self.get_parameter("prompt_file").value)
        )

        self._goal_timer = self.create_timer(0.05, self._process_goal_queue)
        self.get_logger().info(f"chat_ai server_url={self.server_url} model={self.model}")
        if self.prompt_prefix:
            self.get_logger().info(
                f"Loaded prompt prefix ({len(self.prompt_prefix)} chars) from prompt file"
            )
        else:
            self.get_logger().warn("prompt prefix is empty; model may not return robot JSON")

    def enqueue_goal(self, command: str):
        if not command:
            return
        self._goal_queue.put(command)

    def _process_goal_queue(self):
        try:
            command = self._goal_queue.get_nowait()
        except queue.Empty:
            return
        self.send_goal(command)

    def send_goal(self, command: str):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available!")
            return

        goal_msg = Behavior.Goal()
        goal_msg.command = command

        self.get_logger().info(f"Sending behavior goal: {command}")
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"send_goal failed: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            response = future.result()
            msg = getattr(response.result, "message", "")
            self.get_logger().info(
                f"Behavior result: {response.result.result}"
                + (f" ({msg})" if msg else "")
            )
        except Exception as e:
            self.get_logger().error(f"get_result failed: {e}")


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/ai", methods=["POST"])
def ai():
    data = request.get_json(silent=True) or {}
    messages = data.get("messages", [])

    def stream_response():
        yield from generate(messages)

    return Response(stream_response(), mimetype="text/event-stream")


def generate(messages):
    one_message = {"role": "assistant", "content": ""}
    chunk_count = 0

    node = ros_node
    if node is None:
        yield "ROS node not ready"
        return

    try:
        payload = {
            "stream": True,
            "model": node.model,
            "messages": apply_prompt_prefix(messages, node.prompt_prefix),
        }
        response = requests.post(
            node.server_url,
            headers={"Content-Type": "application/json"},
            json=payload,
            stream=True,
            timeout=(10, node.request_timeout_sec),
        )
        response.raise_for_status()

        for line in response.iter_lines():
            if not line:
                continue

            line_str = line.decode("utf-8").strip()
            if not line_str:
                continue

            try:
                line_json = json.loads(line_str)
            except json.JSONDecodeError:
                logger.debug("non-JSON stream line: %s", line_str)
                continue

            if line_json.get("done", False):
                break

            msg = line_json.get("message", {}) or {}
            message_content = msg.get("content") or line_json.get("response") or ""
            if not message_content:
                continue

            chunk_count += 1
            if chunk_count < 40:
                logger.debug(message_content)
            elif chunk_count == 40:
                logger.debug("......")

            one_message["content"] += message_content
            yield message_content

        full_reply = one_message["content"]
        command = extract_behavior_command(full_reply)
        if command:
            node.enqueue_goal(command)
            logger.info("Enqueued behavior command: %s", command)
        else:
            preview = full_reply.strip().replace("\n", " ")[:300]
            logger.warning(
                "No behavior JSON parsed (need type/data). Reply preview: %s",
                preview or "(empty)",
            )

    except requests.exceptions.RequestException as e:
        logger.exception("chat API request failed")
        yield f"Server request error: {str(e)}"
    except (json.JSONDecodeError, ValueError) as e:
        logger.exception("invalid behavior JSON from model")
        yield f"Invalid behavior command: {str(e)}"


def ros_spin(chat_ai):
    rclpy.spin(chat_ai)


def main(args=None):
    global ros_node

    logging.basicConfig(level=logging.INFO)

    rclpy.init(args=args)
    ros_node = ChatAi()

    ros_thread = threading.Thread(target=ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    try:
        app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()