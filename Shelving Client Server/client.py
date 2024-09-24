"""
Client that can interact with the server with various commands

Written by Andrew Viola
"""


import argparse
import socket
import sys

BUFFER_SIZE = 2 ** 12
ENCODING = "UTF-8"

def main():
    args = parser.parse_args()
    server_address = (args.target, args.port)
    print(f"Connecting to {args.target}:{args.port}")
    try:
        peer_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        peer_socket.connect(server_address)

        # Connect to the server with the specific function
        print(f"Connected to {args.target}:{args.port}")
        with peer_socket:
            if args.function == "ADD":
                height = args.height + args.initial_layer_height
                data = args.function + " " + args.name + " " + args.author + " " + str(height)
            elif args.function == "CHECK":
                data = "CHECK DATA"
            elif args.function == "REMOVE":
                data = args.function + " " + str(args.slot)
            data = data.encode(ENCODING)
            peer_socket.send(data)
            peer_socket.shutdown(socket.SHUT_WR)
            received_data = peer_socket.recv(BUFFER_SIZE)
            message = received_data.decode(ENCODING)
            print(" ".join(message.split()))
            if message.split()[0] == "ERROR":
                sys.exit(1)

    except OSError as err:
        print(f"error: {err}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="Add part to shelf",
        description="When provided a name, author, initial layer height, overall height, add part to shelf"
    )
    parser.add_argument("target",type=str)
    parser.add_argument("port",type=int)
    parser.add_argument("function", type=str)
    parser.add_argument("-n", "--name",type=str)
    parser.add_argument("-a","--author",type=str)
    parser.add_argument("-he","--height",type=float)
    parser.add_argument("-i","--initial_layer_height",type=float)
    parser.add_argument("-s", "--slot", type=int)
    main()