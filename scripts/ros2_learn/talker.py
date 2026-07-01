#!/usr/bin/env python3
"""ros2_learn/talker.py — 最小"发布者"节点：每秒往 /learn/chatter 发一句话。

这是你理解 ROS2「节点 / 发布 / 消息(数据格式)」最快的方式：先把它跑起来，
再开 listener.py 或 `ros2 topic echo /learn/chatter` 在另一头看到它。

跑法（先 `source scripts/source_limo_ros_env.sh` 让 ros2/rclpy 可用）：
    python3 scripts/ros2_learn/talker.py
"""

import rclpy  # ROS2 的 Python 客户端库；rclpy.init() 之后这个程序才"上车"
from rclpy.node import Node
from std_msgs.msg import String  # 我们要发的"消息类型"——String 只有一个字段 data


class Talker(Node):
    def __init__(self) -> None:
        super().__init__("learn_talker")  # ← "节点名"，等下能在 `ros2 node list` 里看到
        # create_publisher(消息类型, topic 名, 队列深度) ——> 这个节点成为 /learn/chatter 的"发布者"
        self.pub = self.create_publisher(String, "/learn/chatter", 10)
        self.i = 0
        self.create_timer(1.0, self.tick)  # 每 1.0 秒回调一次 tick()
        self.get_logger().info("talker 上线：每秒发一条到 /learn/chatter")

    def tick(self) -> None:
        msg = String()  # ← 这就是"一条消息 / 一份数据"；它的"数据格式"就是 std_msgs/String
        msg.data = f"hello ros2 #{self.i}"
        self.pub.publish(msg)  # ← "发布"：把这条数据丢到 /learn/chatter 这条流上
        self.get_logger().info(f"发出: {msg.data}")
        self.i += 1


def main() -> None:
    rclpy.init()
    node = Talker()
    try:
        rclpy.spin(node)  # spin = 交出控制权给 ROS，让定时器/回调一直转，直到 Ctrl-C
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
