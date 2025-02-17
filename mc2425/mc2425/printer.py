import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
from mc2425_msgs.srv import FileTransfer
from mc2425_msgs.msg import AddPart
from mc2425.unixSocketHandler import UnixSocketHandler
from mc2425.variables import SAVE_PATH
import os


class Printer(Node):
    def __init__(self):
        super().__init__("printer")
        self.printer_ID = int(os.getenv("PRINTER_ID", -1))
        if self.printer_ID == -1:
            self.get_logger().error("Printer ID not set")
            return
        self.finishedPrintPub = self.create_publisher(AddPart, "finishedPrint", 10)

        self.newPrintSub = self.create_subscription(
            Int32, "initiateReady", self.newPrint, 10
        )
        self.newPrintSub

        self.addFailSub = self.create_subscription(
            AddPart, "addPartFail", self.addFail, 10
        )
        self.addFailSub

        self.request_gcode_client = self.create_client(FileTransfer, "requestGcode")
        while not self.request_gcode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Waiting for MainController service...")
            
        self.checkOnline_client = self.create_client(SetBool, "checkOnline")
            
        self.checkAvailability = None
        self.checkForNewGcode = None

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
                command, xmax, xmin, density, material, height, name, author = items
                self.sendFinishedPrint(float(xmax), float(xmin), float(density), material, float(height), name, author)
            elif items[0] == "GCODE":
                self.requestGcode("")
            elif items[0] == "STATUS":
                self.checkOnline()
            else:
                self.get_logger().error("Invalid message format")
        except ValueError:
            self.get_logger().error("Invalid message format")
            
    def checkOnline(self):
        """
        Request the online status from the service.
        """
        if not self.checkOnline_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service not available!")
            return

        request = SetBool.Request()
        request.data = True  # or False, depending on what you want to check

        future = self.checkOnline_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            online = future.result().success  # Assuming the service returns a success field
            if online:
                self.get_logger().info("Queue is online")
            else:
                self.get_logger().info("Queue is offline")
        else:
            self.get_logger().error("Service call failed!")

    def sendFinishedPrint(self, density, material, height, name, author):
        if self.checkAvailability is not None:
            self.checkAvailability.cancel()
            self.checkAvailability = None
        msg = AddPart()
        msg.print_height = height
        msg.printer_id = self.printer_ID
        msg.part_name = name
        msg.author = author
        msg.material = material
        msg.density = density
        msg.xmin = xmin
        msg.xmax = xmax
        self.finishedPrintPub.publish(msg)
        self.get_logger().info(f"Printer {self.printer_ID} published: {msg}")

    def addFail(self, msg):
        if msg.printer_id == self.printer_ID:
            self.get_logger().info(
                f"Received addPartFail for printer {self.printer_ID}"
            )
            self.checkAvailability = self.create_timer(60,lambda: self.sendFinishedPrint(msg.print_height, msg.part_name, msg.author))
        else:
            self.get_logger().info(f"Ignoring print request for printer {msg.data}")

    def startGcode(self, filename):
        with open(filename, "rb") as gcode:
            pload = {"file": gcode, "print": "true"}
            r = requests.post("http://localhost/server/files/upload", files=pload)
            self._logger.info(f"File {filename} sent to printer")
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
        if self.checkForNewGcode is not None:
            self.checkForNewGcode.cancel()
            self.checkForNewGcode = None
        request = FileTransfer.Request()
        request.filename = filename
        self.get_logger().info(f"Requesting G-code file: {filename}")
        future = self.request_gcode_client.call_async(request)
        future.add_done_callback(self.handle_gcode_response)

    def handle_gcode_response(self, future):
        """
        Handle the response from MainController.
        """
        if not os.path.exists(SAVE_PATH):
            os.makedirs(SAVE_PATH)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"G-code received: {response.filename}")
                # Save the file locally
                file_path = os.path.join(SAVE_PATH, f"{response.filename}")
                with open(file_path, "w") as file:
                    file.write(response.filedata)
                self.get_logger().info(f"G-code file saved to: {file_path}")
                self.startGcode(file_path)
            else:
                self.get_logger().info(f"No files available for download, checking in 60 seconds")
                self.checkForNewGcode = self.create_timer(60, lambda: self.requestGcode(""))
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
