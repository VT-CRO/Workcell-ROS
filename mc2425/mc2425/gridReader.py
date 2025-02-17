import serial
import argparse

# Serial port configuration
SERIAL_PORT = '/dev/ttyACM0'  # Your Pico's serial port
BAUD_RATE = 115200  # Match the baud rate of your Pico

def read_grid(serial_connection):
    """
    Reads a 10x10 grid from the serial connection and converts it to a single list.
    Assumes the grid is sent in the format:
    Shelf Status:
    1 1 1 0 0 0 0 0 0 0
    ...
    Returns:
        A single list containing the grid values (row-major order).
    """
    grid = []
    while True:
        # Read a line from the serial connection
        line = serial_connection.readline().decode('utf-8').strip()
        
        # Skip the "Shelf Status:" line
        if line == "Shelf Status:":
            continue
        
        # Parse grid rows
        if line:
            row = line.split()
            if len(row) == 10 and all(c in '01' for c in row):
                grid.extend([int(c) for c in row])  # Add row to the single list
        
        # Stop reading after 10 rows
        if len(grid) == 100:
            break
    
    return grid

def shelf_status(index):
    """
    Reads the grid from the serial port and returns the value at the specified index.
    Args:
        index (int): The index to check (0-99).
    Returns:
        int: 1 if the value at the index is 1, 0 if the value is 0.
    Raises:
        IndexError: If the index is out of range.
        serial.SerialException: If there is an issue with the serial connection.
    """
    if not (0 <= index < 100):
        raise IndexError("Index out of range. Valid range is 0 to 99.")
    
    try:
        # Open the serial connection
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            # Read the grid
            grid = read_grid(ser)
            if not grid:
                raise ValueError("Failed to read a valid grid.")
            
            # Return the value at the specified index
            return grid[index]
    except serial.SerialException as e:
        raise serial.SerialException(f"Error opening serial port: {e}")
    
if __name__ == "__main__":
    # Test the shelf_status function
    parser = argparse.ArgumentParser(description="Test the shelf_status function.")
    parser.add_argument("index", type=int, help="The index to check (0-99).")
    args = parser.parse_args()
    status = shelf_status(args.index)
    print(f"Shelf status at index {args.index}: {status}")
