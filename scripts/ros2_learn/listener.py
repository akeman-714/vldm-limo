#!/usr/bin/env python3
"""ros2_learn/listener.py — 最小"订阅者"节点：订阅 /learn/chatter，收到就打印。

配合 talker.py 一起看：talker 在一头"发布"，listener 在另一头"订阅"，
两个程序素不相识，只靠 topic 名字 /learn/chatter 对上。这就是 ROS 的"发布/订阅"。

跑法（先 `source scripts/source_limo_ros_env.sh`）：
    python3 scripts/ros2_learn/listener.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self) -> None:
        super().__init__("learn_listener")
        # create_subscription(消息类型, topic 名, 回调函数, 队列深度) ——> 成为 /learn/chatter 的"订阅者"
        self.sub = self.create_subscription(String, "/learn/chatter", self.on_msg, 10)
        self.get_logger().info("listener 上线：等 /learn/chatter ...")

    def on_msg(self, msg: String) -> None:
        # 关键：你不用主动去"读"。每来一条消息，ROS 自动替你调用这个回调，msg 就是那条数据。
        self.get_logger().info(f"收到: {msg.data}")


def main() -> None:
    rclpy.init()
    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
