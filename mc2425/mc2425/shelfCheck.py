import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class shelfCheck(Node):
    def __init__(self):
        super().__init__('shelfCheck')
        # Declare and get the parameter for height
        self.declare_parameter('shelf', 0)

        shelfNumber = self.get_parameter('shelf').get_parameter_value().integer_value

        # Create publisher
        self.publisher_ = self.create_publisher(Int32, 'shelfCheck', 10)

        # Set up a timer for async behavior, even if we only publish once
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, lambda: self.publish_once(shelfNumber))


    def publish_once(self,shelfNumber):
        # Create a customizable string message
        msg = Int32()
        msg.data = int(shelfNumber)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

        # Stop the timer after publishing once
        self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    shelf_check = shelfCheck()
    rclpy.spin_once(shelf_check)  # Spin only once, then shutdown ------------------------ spin_once
    shelf_check.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
