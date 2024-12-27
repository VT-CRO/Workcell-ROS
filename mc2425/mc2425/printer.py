import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import String
import os


class Printer(Node):
    def __init__(self):
        super().__init__("printer")
        self.startGcodeSub = self.create_subscription(
            String, "startGcode", self.startGcodeCallback, 10
        )
        self.startGcodeSub  # prevent unused variable warning

    def startGcodeCallback(self, msg):
        with open(msg.data, "rb") as gcode:
            pload = {"file": gcode, "print": "true"}
            r = requests.post("http://localhost/server/files/upload", files=pload)
            self._logger.info(f"File {msg.data} sent to printer. Response: {r.text}")
        os.remove(msg.data)


def main(args=None):
    rclpy.init(args=args)
    printer = Printer()
    rclpy.spin(printer)
    printer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
