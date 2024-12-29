import socket
import os
import threading


class UnixSocketHandler:
    def __init__(self, socket_path, message_callback, logger):
        """
        Initializes the UnixSocketHandler.

        :param socket_path: Path for the UNIX socket
        :param message_callback: Function to call when a message is received
        :param logger: Logger instance for logging messages
        """
        self.socket_path = socket_path
        self.message_callback = message_callback
        self.logger = logger
        self.sock = None
        self.listener_thread = None

    def setup_socket(self):
        """
        Sets up the UNIX socket and starts the listener thread.
        """
        try:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            if os.path.exists(self.socket_path):
                os.remove(self.socket_path)
            self.sock.bind(self.socket_path)
            self.logger.info(f"Listening on UNIX socket: {self.socket_path}")

            # Start the listener thread
            self.listener_thread = threading.Thread(target=self._listen_for_messages, daemon=True)
            self.listener_thread.start()
        except Exception as e:
            self.logger.error(f"Failed to set up UNIX socket: {str(e)}")

    def _listen_for_messages(self):
        """
        Internal method to listen for incoming messages on the socket.
        """
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                message = data.decode('utf-8').strip()
                self.logger.info(f"Received socket message: {message}")
                self.message_callback(message)
            except Exception as e:
                self.logger.error(f"Socket listener error: {str(e)}")
                break

    def cleanup_socket(self):
        """
        Cleans up the UNIX socket when the handler is destroyed.
        """
        if self.sock:
            self.sock.close()
            if os.path.exists(self.socket_path):
                os.remove(self.socket_path)
            self.logger.info(f"Socket at {self.socket_path} cleaned up.")
