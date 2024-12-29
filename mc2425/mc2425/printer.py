import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import String
from mc2425_msgs.msg import AddPart
from mc2425.unixSocketHandler import UnixSocketHandler
import os


class Printer(Node):
    def __init__(self):
        super().__init__("printer")
        self.printer_number = int(os.getenv("PRINTER_NUMBER", -1))
        if self.printer_number == -1:
            self.get_logger().error("Printer number not set")
            return
        self.finishedPrintPub = self.create_publisher(AddPart, "finishedPrint", 10)
        
        self.startGcodeSub = self.create_subscription(
            String, "startGcode", self.startGcodeCallback, 10
        )
        self.startGcodeSub  # prevent unused variable warning
        self.get_logger().info(f"Printer {self.printer_number} initialized")
        self.socket_path = f"/tmp/printer_socket"
        self.socket_handler = UnixSocketHandler(
            socket_path=self.socket_path,
            message_callback=self.handle_socket_message,
            logger=self.get_logger(),
        )
        self.socket_handler.setup_socket()
        
    def handle_socket_message(self, message):
        """
        Callback for handling messages received from the UNIX socket.
        :param message: Message received from the socket
        """
        # Parse message (format: height,name,author)
        try:
            height, name, author = message.split(',')
            self.sendFinishedPrint(float(height), name, author)
        except ValueError:
            self.get_logger().error("Invalid message format")

    def sendFinishedPrint(self, height, name, author):
        msg = AddPart()
        msg.print_height = height
        msg.printer_number = self.printer_number
        msg.part_name = name
        msg.author = author
        self.finishedPrintPub.publish(msg)
        self.get_logger().info(f"Printer {self.printer_number} published: {msg}")
    
    def startGcodeCallback(self, msg):
        with open(msg.data, "rb") as gcode:
            pload = {"file": gcode, "print": "true"}
            r = requests.post("http://localhost/server/files/upload", files=pload)
            self._logger.info(f"File {msg.data} sent to printer. Response: {r.text}")
        os.remove(msg.data)


def main(args=None):
    rclpy.init(args=args)
    printer = Printer()
    if printer.printer_number == -1:
        return -1
    rclpy.spin(printer)
    printer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
