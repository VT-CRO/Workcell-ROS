import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import Int32
from mc2425_msgs.srv import FileTransfer
from mc2425_msgs.msg import AddPart
from mc2425.unixSocketHandler import UnixSocketHandler
import os

SAVE_PATH = ""


class Printer(Node):
    def __init__(self):
        super().__init__("printer")
        self.printer_ID = int(os.getenv("PRINTER_ID", -1))
        if self.printer_ID == -1:
            self.get_logger().error("Printer ID not set")
            return
        self.finishedPrintPub = self.create_publisher(AddPart, "finishedPrint", 10)

        self.newPrintSub = self.create_subscription(
            Int32, "initiateReady", self.newPrint(), 10
        )
        self.newPrintSub

        self.request_gcode_client = self.create_client(FileTransfer, "requestGcode")
        while not self.request_gcode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Waiting for MainController service...")

        self.get_logger().info(f"Printer {self.printer_ID} initialized")
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
            items = message.split(",")
            if items[0] == "END":
                command, height, name, author = items
                self.sendFinishedPrint(float(height), name, author)
            elif items[0] == "GCODE":
                self.requestGcode("")
            else:
                self.get_logger().error("Invalid message format")
        except ValueError:
            self.get_logger().error("Invalid message format")

    def sendFinishedPrint(self, height, name, author):
        msg = AddPart()
        msg.print_height = height
        msg.printer_id = self.printer_ID
        msg.part_name = name
        msg.author = author
        self.finishedPrintPub.publish(msg)
        self.get_logger().info(f"Printer {self.printer_ID} published: {msg}")

    def startGcode(self, filename):
        with open(filename, "rb") as gcode:
            pload = {"file": gcode, "print": "true"}
            r = requests.post("http://localhost/server/files/upload", files=pload)
            self._logger.info(f"File {filename} sent to printer. Response: {r.text}")
        os.remove(filename)

    def newPrint(self, msg):
        if msg.data == self.printer_ID:
            self.get_logger().info(
                f"Received new print request for printer {self.printer_ID}"
            )
            self.requestGcode("")
        else:
            self.get_logger().info(f"Ignoring print request for printer {msg.data}")

    def requestGcode(self, filename):
        """
        Request a G-code file from MainController.
        """
        request = FileTransfer.Request()
        request.filename = filename
        self.get_logger().info(f"Requesting G-code file: {filename}")
        future = self.request_gcode_client.call_async(request)
        future.add_done_callback(self.handle_gcode_response)

    def handle_gcode_response(self, future):
        """
        Handle the response from MainController.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"G-code received: {response.filename}")
                # Save the file locally
                file_path = os.path.join(SAVE_PATH, f"test-{response.filename}")
                with open(file_path, "w") as file:
                    file.write(response.filedata)
                self.get_logger().info(f"G-code file saved to: {file_path}")
                self.startGcode(file_path)
            else:
                self.get_logger().error(f"Failed to get G-code: {response.filename}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    printer = Printer()
    if printer.printer_ID == -1:
        return -1
    rclpy.spin(printer)
    printer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
