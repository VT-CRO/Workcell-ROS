import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from mc2425_msgs.msg import PnPRemoval

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pickAndPlace')
        self.pnpSub = self.create_subscription(PnPRemoval,'pnpRemover',self.pnp_callback,10)
        self.pnpSub  # prevent unused variable warning

    def pnp_callback(self, msg):
        self.get_logger().info(f'Received: {msg.print_removal} {msg.print_number} {msg.shelf_num}')
        if msg.print_removal:
            self.get_logger().info("Sending to the removal machine...")
            # TODO: Implement removal task
        else:
            self.get_logger().info(f"Moving plate on printer {msg.print_number} to shelf #{msg.shelf_num}")
            # TODO: Send G-Code to move print

def main(args=None):
    rclpy.init(args=args)
    Pick_And_Place = PickAndPlace()
    rclpy.spin(Pick_And_Place)
    Pick_And_Place.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
