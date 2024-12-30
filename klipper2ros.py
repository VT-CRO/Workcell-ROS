import socket
import argparse

def main(message,socket_path):
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client_socket:
            client_socket.sendto(message.encode("utf-8"), socket_path)
            print(f"Message sent: {message}")
    except Exception as e:
        print(f"Failed to send message: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send a message to a socket.")
    parser.add_argument("-s", "--socketLoc", required=True, default="/tmp/printer_socket", help="Path to the socket")
    parser.add_argument("-m", "--message", required=True, nargs="+", help="Message to send")
    args = parser.parse_args()
    message = args.message
    if type(message) == list:
        message = " ".join(message)
    main(message,args.socketLoc)
