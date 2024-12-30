import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard


class KeyboardMonitor(Node):
    def __init__(self):
        super().__init__("keyboard_monitor")

        # Publisher to send key events
        self.key_pub = self.create_publisher(String, "keyboard_input", 10)

        # Start listening to keyboard inputs
        self.listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )
        self.listener.start()

        self.get_logger().info("Keyboard monitor node started")

    def on_key_press(self, key):
        """
        Callback for key press events.
        """
        try:
            # Publish the key that was pressed
            msg = String()
            msg.data = f"Pressed: {key.char}"  # Normal character keys
            self.key_pub.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")
        except AttributeError:
            # Handle special keys (e.g., ctrl, alt, etc.)
            msg = String()
            msg.data = f"Pressed: {key}"
            self.key_pub.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")

    def on_key_release(self, key):
        """
        Callback for key release events.
        """
        msg = String()
        msg.data = f"Released: {key}"
        self.key_pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

        # Stop listener on specific key (optional)
        if key == keyboard.Key.esc:
            self.get_logger().info("Escape key pressed. Stopping keyboard monitor...")
            self.listener.stop()


def main(args=None):
    rclpy.init(args=args)
    keyboard_monitor = KeyboardMonitor()

    try:
        rclpy.spin(keyboard_monitor)
    except KeyboardInterrupt:
        keyboard_monitor.get_logger().info("Shutting down KeyboardMonitor node.")
    finally:
        keyboard_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
