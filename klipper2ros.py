import socket
import argparse

socket_path = "/tmp/printer_socket"


def main(message):
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client_socket:
            client_socket.sendto(message.encode('utf-8'), socket_path)
            print(f"Message sent: {message}")
    except Exception as e:
        print(f"Failed to send message: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send a message to the printer")
    parser.add_argument("message", type=str, help="Message to send")
    args = parser.parse_args()
    message = args.message
    main(message)