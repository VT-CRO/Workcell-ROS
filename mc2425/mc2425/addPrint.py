# DEPRECATED: This node is not used in the system - AV 12/27/24

import rclpy
from rclpy.node import Node
from mc2425_msgs.msg import AddPart


class addPrint(Node):
    def __init__(self):
        super().__init__('addPrint')
        # Declare and get the parameter for height
        self.declare_parameter('height', 0.0)
        self.declare_parameter('printer', 0)
        self.declare_parameter('name', "")
        self.declare_parameter('author', "")

        heightParam = self.get_parameter('height').get_parameter_value().double_value
        printerNumber = self.get_parameter('printer').get_parameter_value().integer_value
        nameParam = self.get_parameter('name').get_parameter_value().string_value
        authorParam = self.get_parameter('author').get_parameter_value().string_value

        # Create publisher
        self.publisher_ = self.create_publisher(AddPart, 'finishedPrint', 10)

        # Set up a timer for async behavior, even if we only publish once
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, lambda: self.publish_once(heightParam,printerNumber,nameParam,authorParam))


    def publish_once(self,height,number,name,author):
        # Create a customizable string message
        msg = AddPart()
        msg.print_height = float(height)
        msg.printer_id = int(number)
        msg.part_name = name
        msg.author = author
        msg.xmax = 300.0
        msg.xmin = 0.0
        msg.density = 10.0
        msg.material = "PLA"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.printer_id} {msg.print_height}, {msg.part_name} {msg.author}')

        # Stop the timer after publishing once
        self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    add_print = addPrint()
    rclpy.spin_once(add_print)  # Spin only once, then shutdown ------------------------ spin_once
    add_print.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
