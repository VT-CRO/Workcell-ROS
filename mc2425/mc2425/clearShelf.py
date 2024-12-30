import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class clearShelf(Node):
    def __init__(self):
        super().__init__("clearShelf")
        self.clearShelfPub = self.create_publisher(String, "clearShelf", 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, lambda: self.publish_once())

    def publish_once(self):
        msg = String()
        msg.data = "clear"
        self.clearShelfPub.publish(msg)
        self.get_logger().info(f"Publishing: clear")
        self.destroy_timer(self.timer)


def main(args=None):
    rclpy.init(args=args)
    clear_Shelf = clearShelf()
    rclpy.spin_once(
        clear_Shelf
    )  # Spin only once, then shutdown ------------------------ spin_once
    clear_Shelf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
