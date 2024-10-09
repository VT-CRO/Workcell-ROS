import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mc2425_msgs.msg import AddPart

from mc2425.shelf import partsShelf
from mc2425.gcode import gcode

SLOT_NUMBER = 24
HEIGHT = 1200

class MainController(Node):
    def __init__(self):
        super().__init__('mainController')
        self.addSubscription = self.create_subscription(
            AddPart,
            'addPart',
            self.addPart_callback,
            10)
        self.addSubscription  # prevent unused variable warning
        self.shelf = partsShelf(SLOT_NUMBER, HEIGHT)

    def addPart(self,name,author,height):
        """
        Adds a part to the shelf and tells the printer to go to a specific slot
        :param data: List of data from a TCP socket request
        :return: String to send back to client
        """
        part = gcode(name, author, float(height))
        confirm, slot = self.shelf.addPart(part)
        if confirm:
            return f"Part added successfully in slot {slot}"
        else:
            return "Part not added, skipping..."

    def utilizeData(self, function, parameters):
        if function == "ADD":
            self.get_logger().info(self.addPart(parameters))

    def addPart_callback(self, msg):
        self.get_logger().info(f'Received: {msg.printer_number} {msg.print_height} {msg.part_name} {msg.author}')
        self.get_logger().info(self.addPart(msg.part_name,msg.author,msg.print_height))
        # data = msg.data.split()
        # function = data[0]
        # data.pop(0)
        # elf.utilizeData(function,data)



def main(args=None):
    rclpy.init(args=args)
    Main_Controller = MainController()
    rclpy.spin(Main_Controller)
    Main_Controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
