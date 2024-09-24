"""
Server program that accepts responses on a specified part. Has a variety of functions to load parts into the shelving unit
and to submit gcode commands to a specified printer

Written by Andrew Viola
"""

import socket
import argparse
import logging
import sys
import requests
from shelf import partsShelf
from gcode import gcode

DEFAULT_PORT = 10005
IP_ADDRESS = "0.0.0.0"
ENCODING = "UTF-8"
BUFFER_SIZE = 2 ** 12
SLOT_NUMBER = 24
HEIGHT = 1200

PRINTER_IP = "cro.local"

logger = logging.getLogger("Server")

# Instantiates a shelf object
shelf = partsShelf(SLOT_NUMBER, HEIGHT)

def addPart(data):
    """
    Adds a part to the shelf and tells the printer to go to a specific slot
    :param data: List of data from a TCP socket request
    :return: String to send back to client
    """
    part = gcode(data[1],data[2],float(data[3]))
    confirm, slot = shelf.addPart(part)
    response = requests.post(f"http://{PRINTER_IP}:7125/printer/gcode/script", params={"script": f"GO_TO_SLOT SLOT={slot}"})
    if confirm and response.status_code == 200:
        return f"Part added successfully in slot {slot}"
    else:
        return "Part not added, skipping..."

def removePart(data):
    """
    Removes a part from the shelf
    :param data: List of data from a TCP socket request
    :return: String to send back to client
    """
    confirm = shelf.removePart(int(data[1]))
    if confirm:
        return "Part removed successfully"
    else:
        return "Part not located or does not start at this layer..."


def main():

    port = parser.parse_args().port
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)

    server_address = (IP_ADDRESS, port)

    # Starts up the TCP server
    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(server_address)
        server.listen(20) # Can accept up to 20 connections into the queue
    except OSError as err:
        logger.error(f"cannot listen on port {port}: {err}")
        raise SystemExit(1)

    logger.info(f" Listening at IP Address: {IP_ADDRESS} Port: {port}")

    # Listens for incoming connections
    while True:
        connection, address = server.accept()
        with connection:
            logger.info(f" Accepted connection from {address}")
            try:
                # Get data, decode and check what function it is
                while data := connection.recv(2**12):
                    logger.info(f" received: data: {data}")
                    output = "Function not found, nothing happened...".encode(ENCODING)
                    data = data.strip().decode(ENCODING)
                    data = data.split()
                    if data[0] == "ADD":
                        output = addPart(data).encode(ENCODING)
                    elif data[0] == "CHECK":
                        output = str(shelf).encode(ENCODING)
                    elif data[0] == "REMOVE":
                        output = removePart(data).encode(ENCODING)
                    connection.send(output)
            except OSError as err:
                logger.error(f" error communicating with {address}: {err}")

            logger.info(f" disconnected from {address}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="Shelf Storage",
        description="Adds a part to the shelf",
    )
    parser.add_argument("-p","--port",type=int,default=DEFAULT_PORT,help="The port your server will run on")
    main()
