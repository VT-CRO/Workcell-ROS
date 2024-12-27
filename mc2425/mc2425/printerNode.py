import rclpy
from rclpy.node import Node
from mc2425_msgs.srv import FileTransfer  # Replace with your package name


class PrinterNode(Node):
    def __init__(self):
        super().__init__("printer_node")
        self.service = self.create_service(
            FileTransfer, "/file_transfer", self.handle_file_transfer
        )
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
    printer_node = PrinterNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    printer_node.executor = (
        executor  # Pass executor to the node for controlled shutdown
    )
    executor.add_node(printer_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        printer_node.get_logger().info("PrinterNode interrupted by user.")
    finally:
        printer_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
