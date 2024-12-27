import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from mc2425_msgs.msg import AddPart
from mc2425_msgs.msg import PnPRemoval
from mc2425_msgs.srv import FileTransfer

from mc2425.shelf import partsShelf
from mc2425.gcode import gcode

SLOT_NUMBER = 24
HEIGHT = 1200


class MainController(Node):
    def __init__(self):
        super().__init__("mainController")
        self.addSubscription = self.create_subscription(
            AddPart, "addPart", self.addPart_callback, 10
        )
        self.addSubscription  # prevent unused variable warning
        self.shelfCheckSub = self.create_subscription(
            Int32, "shelfCheck", self.shelfCheck_callback, 10
        )
        self.shelfCheckSub  # prevent unused variable warning
        self.removePartSub = self.create_subscription(
            Int32, "removePart", self.removePart_callback, 10
        )
        self.removePartSub  # prevent unused variable warning
        self.pnpRemoverPub = self.create_publisher(PnPRemoval, "pnpRemover", 10)
        self.client = self.create_client(FileTransfer, "requestGcode")
        self.timer = self.create_timer(1.0, self.check_service)
        self.shelf = partsShelf(SLOT_NUMBER, HEIGHT)

    def determineRemoval(self):
        """
        TODO
        :return: Boolean on whether it should be removed from plate
        """
        return False

    def addPart(self, name, author, height):
        """
        Adds a part to the shelf and tells the printer to go to a specific slot
        :param data: List of data from a TCP socket request
        :return: String to send back to client
        """
        part = gcode(name, author, float(height))
        confirm, slot = self.shelf.addPart(part)
        if confirm:
            return f"Part added successfully in slot {slot}", slot
        else:
            return "Part not added, skipping...", -1

    # def printReady_callback(self,msg):
    #     self.get_logger().info(f'Received: {msg.data}')
    #     self.shelf.printReady(msg.data)

    def addPart_callback(self, msg):
        self.get_logger().info(
            f"Received: {msg.printer_number} {msg.print_height} {msg.part_name} {msg.author}"
        )
        message, slot = self.addPart(msg.part_name, msg.author, msg.print_height)
        self.get_logger().info(message)
        if slot != -1:
            pnpRemover_msg = PnPRemoval()
            pnpRemover_msg.print_removal = self.determineRemoval()
            pnpRemover_msg.print_number = msg.printer_number
            pnpRemover_msg.shelf_num = slot
            self.pnpRemoverPub.publish(pnpRemover_msg)
            self.get_logger().info(
                f"Publishing: Removal: {pnpRemover_msg.print_removal}, Printer Number: {pnpRemover_msg.print_number}, Slot Number: {pnpRemover_msg.shelf_num}"
            )

    def shelfCheck_callback(self, msg):
        shelfNum = msg.data
        self.get_logger().info(f"Received: {shelfNum}")
        part = self.shelf.shelfChecker(shelfNum)
        if part is not None:
            self.get_logger().info(f"{part} found at #{shelfNum}")
        else:
            self.get_logger().info(f"No part found at #{shelfNum}")

    def removePart_callback(self, msg):
        shelfNum = msg.data
        self.get_logger().info(f"Received: {shelfNum}")
        remove = self.shelf.removePart(shelfNum)
        if remove:
            self.get_logger().info(f"Successful remove at #{shelfNum}")
        else:
            self.get_logger().info(f"No part found at #{shelfNum}")

    def check_service(self):
        if self.client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("Printer service is available. Sending file...")
            self.send_file()
        else:
            pass
            # self.get_logger().info("Waiting for printer service...")

    def send_file(self):
        request = FileTransfer.Request()
        file_name = "example.gcode"
        request.filename = "example.gcode"
        try:
            with open(file_name, "r") as file:
                request.filedata = file.read()
        except Exception as e:
            self.get_logger().error(f"Failed to read file: {str(e)}")
        future = self.client.call_async(request)
        future.add_done_callback(self.file_transfer_callback)

    def file_transfer_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"File transfer successful: {response.message}")
            else:
                self.get_logger().info(f"File transfer failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    Main_Controller = MainController()
    rclpy.spin(Main_Controller)
    Main_Controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
