import rclpy
from rclpy.node import Node
from mc2425_msgs.srv import FileTransfer
from std_msgs.msg import String


class RequestGcode(Node):
    def __init__(self):
        super().__init__("requestGcode")
        self.service = self.create_service(
            FileTransfer, "requestGcode", self.handle_file_transfer
        )
        self.publisher = self.create_publisher(String, "startGcode", 10)
        self.get_logger().info("Printer is ready for file transfer.")

    def handle_file_transfer(self, request, response):
        self.get_logger().info(f"Receiving file: {request.filename}")
        # Process the file data (e.g., save to disk)
        try:
            with open(f"{request.filename[:-6]}-output.gcode", "w") as file:
                file.write(request.filedata)
            response.success = True
            response.message = (
                f"File {request.filename} received and saved successfully."
            )
            self.get_logger().info(response.message)
            msg = String()
            msg.data = request.filename
            self.publisher.publish(msg)
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to save file: {str(e)}"
            self.get_logger().error(response.message)

        # Signal main loop to stop spinning
        self.get_logger().info("Request processed. Shutting down PrinterNode...")
        self.executor.shutdown()  # Signal the executor to stop
        return response


def main(args=None):
    rclpy.init(args=args)
    Request_Gcode = RequestGcode()
    executor = rclpy.executors.SingleThreadedExecutor()
    Request_Gcode.executor = (
        executor  # Pass executor to the node for controlled shutdown
    )
    executor.add_node(Request_Gcode)

    try:
        executor.spin()
    except KeyboardInterrupt:
        Request_Gcode.get_logger().info("PrinterNode interrupted by user.")
    finally:
        Request_Gcode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
