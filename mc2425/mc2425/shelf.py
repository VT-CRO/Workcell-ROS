import rclpy
from rclpy.node import Node
from mc2425_msgs.msg import ShelfUpdate
from mc2425.gridReader import shelf_status
from mc2425.variables import SLOT_NUMBER, PLATE_BANK
#import random

# def shelf_status(slot_index):
#     # Simulate a changing status for testing
#     # For instance, returning a random value or an incrementing counter

#         return random.randint(0,0)

class ShelfStatus(Node):
    def __init__(self):
        super().__init__("shelf_status")

        # Publisher to send shelf status updates
        self.shelf_pub = self.create_publisher(ShelfUpdate, "shelf_status", 10)

        # Initialize status list for slots and plates
        self.status = [1,1,1]
        for i in range(SLOT_NUMBER):
            self.status.append(shelf_status(i))
            msg = ShelfUpdate()
            msg.shelf_num = i
            msg.status = bool(self.status[i])
            msg.type = "shelf"
            self.shelf_pub.publish(msg)
        for i in range(PLATE_BANK):
            self.status.append(shelf_status(i))
            msg = ShelfUpdate()
            msg.shelf_num = i
            msg.status = bool(self.status[i])
            msg.type = "plate_bank"
            self.shelf_pub.publish(msg)

        self.get_logger().info("Shelf node started")

        # Create a timer that calls the update callback every 0.5 seconds.
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        

    def timer_callback(self):
        self.update_shelf_status()

    def update_shelf_status(self):
        # Preserve the old status to compare with the updated one.
        old_status = list(self.status)

        # Check for new data in each shelf slot and publish if updated
        for i in range(0,SLOT_NUMBER):
            self.status[i] = shelf_status(i)
            if self.status[i] != old_status[i]:
                msg = ShelfUpdate()
                msg.shelf_num = i
                msg.status = bool(self.status[i])
                msg.type = "shelf"
                self.shelf_pub.publish(msg)
                self.get_logger().info(
                    f"Published update for shelf {i}: {bool(self.status[i])}"
                )
        for i in range(SLOT_NUMBER,PLATE_BANK):
            self.status[i] = shelf_status(i)
            if self.status[i] != old_status[i]:
                msg = ShelfUpdate()
                msg.shelf_num = i
                msg.type = "plate_bank"
                msg.status = bool(self.status[i])
                self.shelf_pub.publish(msg)
                self.get_logger().info(
                    f"Published update for plate bank {i}: {bool(self.status[i])}"
                )
            

def main(args=None):
    rclpy.init(args=args)
    shelf_status_node = ShelfStatus()

    try:
        rclpy.spin(shelf_status_node)
    except KeyboardInterrupt:
        shelf_status_node.get_logger().info("Shutting down shelf node.")
    finally:
        shelf_status_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
