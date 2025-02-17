import serial
import argparse

# Serial port configuration
SERIAL_PORT = '/dev/ttyACM0'  # Your Pico's serial port
BAUD_RATE = 115200  # Match the baud rate of your Pico

def read_grid(serial_connection):
    """
    Reads a flattened grid list from the serial connection.
    Assumes the grid is sent in the format:
    Shelf Status List:
    [1, 1, 1, 0, 0, 0, ..., 0]
    Returns:
        A single list containing the grid values.
    """
    while True:
        # Read a line from the serial connection
        line = serial_connection.readline().decode('utf-8').strip()
        
        # Look for the "Shelf Status List:" prefix
        if line.startswith("Shelf Status List:"):
            # Extract the list from the line
            list_str = line[len("Shelf Status List:"):].strip()
            try:
                # Convert the string to a Python list
                grid = eval(list_str)
                
                # Validate the grid
                if len(grid) == 100 and all(c in [0, 1] for c in grid):
                    return grid
                else:
                    raise ValueError("Invalid grid data received.")
            except Exception as e:
                raise ValueError(f"Error parsing grid data: {e}")

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
    try:
        status = shelf_status(args.index)
        print(f"Shelf status at index {args.index}: {status}")
    except Exception as e:
        print(f"Error: {e}")
