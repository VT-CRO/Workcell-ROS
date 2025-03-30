import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from mc2425_msgs.msg import PnPRemoval
from mc2425.unixSocketHandler import UnixSocketHandler
from mc2425.gridReader import shelf_status
import requests


class gantry(Node):
    def __init__(self):
        super().__init__("gantry")
        self.pnpSub = self.create_subscription(
            PnPRemoval, "pnpRemover", self.pnp_callback, 10
        )
        self.pnpSub  # prevent unused variable warning
        
        self.initiateReady = self.create_publisher(Int32, "initiateReady", 10)
        
        self.get_logger().info("Gantry initialized")
        self.socket_path = f"/tmp/gantry_socket"
        self.socket_handler = UnixSocketHandler(
            socket_path=self.socket_path,
            message_callback=self.handle_socket_message,
            logger=self.get_logger(),
        )
        self.socket_handler.setup_socket()
        
        self.command_queue = []
        
        #self.timer_period = 5  # seconds
        #self.timer = self.create_timer(self.timer_period, self.check_queue)
        
    def check_ready(self):
        url = "http://localhost/printer/objects/query"
        payload = {"objects": {"display_status": None}}
        try:
            response = requests.post(url, json=payload, timeout=5)
            response.raise_for_status()  # Raise HTTPError for bad responses (4xx or 5xx)
            result = response.json()
            if result["result"]["status"]["display_status"]["message"] is None:
                return True
            else:
                return False
        except:
            pass
        
    def check_queue(self):
        if len(self.command_queue) != 0:
            if self.check_ready():
                msg = self.command_queue.pop(0)
                if msg.print_removal:
                    self.get_logger().info("Sending to the removal machine...")
                    # TODO: Implement removal task
                else:
                    self.get_logger().info(
                        f"Moving plate on printer {msg.print_id} to shelf #{msg.shelf_num}"
                    )
                    data = {"script": f"MOVE_PLATE PRINTER_NUMBER={msg.print_id} SHELF_NUMBER={msg.shelf_num} PLATE_NUMBER={msg.plate_num}"}
                    requests.post("http://localhost/printer/gcode/script", json=data)
            else:
                pass
        else:
            self.get_logger().info("Nothing to pick up")

    def pnp_callback(self, msg):
        self.get_logger().info(
            f"Received: {msg.print_removal} {msg.print_id} {msg.shelf_num} {msg.plate_num}"
        )
        self.command_queue.append(msg)
        self.check_queue()
            
    def ensureReady(self, print_id, shelf_num):
        # TODO: Implement logic where it can go false
        #inShelf = shelf_status(shelf_num)
        return True
            
    def handle_socket_message(self, message):
        """
        Callback for handling messages received from the UNIX socket.
        :param message: Message received from the socket
        """
        # Parse message (format: height,name,author)
        try:
            items = message.split(",")
            if items[0] == "MOVE_COMPLETE":
                _, print_id, shelf_num = items
                if self.ensureReady(print_id, int(shelf_num)):
                    self.get_logger().info("Shelf movement successful")
                    msg = Int32()
                    msg.data = int(print_id)
                    self.initiateReady.publish(msg)
                else:
                    self.get_logger().info("Shelf movement failed")
            else:
                self.get_logger().error("Invalid message format")
        except ValueError:
            self.get_logger().error("Invalid message format")


def main(args=None):
    rclpy.init(args=args)
    Gantry = gantry()
    rclpy.spin(Gantry)
    Gantry.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
