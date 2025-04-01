#!/usr/bin/env python3
import lgpio
import time
import statistics
import sys

# --- Default Configuration (can be overridden) ---
DEFAULT_GPIO_SIG = 23      # BCM pin number for the Signal pin
DEFAULT_NUM_READINGS = 3   # Number of readings to average
DEFAULT_READ_DELAY_S = 0.1 # Delay between readings in seconds
DEFAULT_SOUND_SPEED_CM_S = 34300 # Speed of sound in cm/s
DEFAULT_TIMEOUT_S = 0.1    # Timeout for waiting for echo (in seconds)
DEFAULT_TRIGGER_PULSE_S = 0.00001 # Duration of trigger pulse (10 us)
DEFAULT_GPIO_CHIP = 4      # Try 0 if 4 doesn't work for your Pi model

# --- Internal Helper Function ---
def _measure_single_distance_mm(h, sig_pin, sound_speed_cm_s, timeout_s, trigger_pulse_s):
    """
    Measures a single distance reading using lgpio handle.
    Returns distance in mm or None on failure.
    """
    pulse_start_time = 0
    pulse_end_time = 0

    try:
        # --- Send Trigger Pulse ---
        lgpio.gpio_claim_output(h, sig_pin, 0) # Claim pin for output, start low
        time.sleep(0.000002)                   # Small delay
        lgpio.gpio_write(h, sig_pin, 1)        # Go high
        time.sleep(trigger_pulse_s)
        lgpio.gpio_write(h, sig_pin, 0)        # Go low
        lgpio.gpio_free(h, sig_pin)            # Release pin

        # --- Receive Echo Pulse ---
        lgpio.gpio_claim_input(h, sig_pin)     # Claim pin for input

        start_wait_time = time.time()
        # Wait for Echo to go HIGH (pulse start)
        while lgpio.gpio_read(h, sig_pin) == 0:
            pulse_start_time = time.time()
            if pulse_start_time - start_wait_time > timeout_s:
                lgpio.gpio_free(h, sig_pin)
                return None # Timeout

        start_wait_time = time.time() # Reset timeout timer
        # Wait for Echo to go LOW (pulse end)
        while lgpio.gpio_read(h, sig_pin) == 1:
            pulse_end_time = time.time()
            if pulse_end_time - start_wait_time > timeout_s:
                lgpio.gpio_free(h, sig_pin)
                return None # Timeout

        lgpio.gpio_free(h, sig_pin) # Free the pin after successful read

        # --- Calculate Distance ---
        pulse_duration = pulse_end_time - pulse_start_time
        # Distance in cm = (Time * SpeedOfSound_cm) / 2
        distance_cm = (pulse_duration * sound_speed_cm_s) / 2
        # Convert to mm
        distance_mm = round(distance_cm * 10, 1) # Round to 1 decimal place for mm
        return distance_mm

    except lgpio.error as e:
        # print(f"lgpio error during measurement: {e}", file=sys.stderr) # Optional debug
        # Attempt to free the pin in case of error
        try:
            lgpio.gpio_free(h, sig_pin)
        except lgpio.error:
            pass # Ignore if already free or other error
        return None
    except Exception as e: # Catch other potential errors
        # print(f"Unexpected error during measurement: {e}", file=sys.stderr) # Optional debug
        try:
            lgpio.gpio_free(h, sig_pin)
        except lgpio.error:
            pass
        return None


# --- Importable Function ---
def get_average_distance_mm(
    sig_pin=DEFAULT_GPIO_SIG,
    num_readings=DEFAULT_NUM_READINGS,
    read_delay_s=DEFAULT_READ_DELAY_S,
    gpio_chip=DEFAULT_GPIO_CHIP,
    sound_speed_cm_s=DEFAULT_SOUND_SPEED_CM_S,
    timeout_s=DEFAULT_TIMEOUT_S,
    trigger_pulse_s=DEFAULT_TRIGGER_PULSE_S,
    min_range_mm=10, # Minimum plausible distance in mm (1 cm)
    max_range_mm=4000 # Maximum plausible distance in mm (400 cm / 4m)
):
    """
    Opens GPIO chip, takes multiple readings from an ultrasonic sensor,
    averages them, and returns the result in millimeters.

    Args:
        sig_pin (int): BCM GPIO pin number for the sensor's signal line.
        num_readings (int): Number of readings to take for averaging.
        read_delay_s (float): Delay in seconds between readings.
        gpio_chip (int): The lgpio chip number to use (usually 4 or 0 for Pi 5).
        sound_speed_cm_s (float): Speed of sound in cm/s for calculation.
        timeout_s (float): Timeout in seconds for waiting for echo pulses.
        trigger_pulse_s (float): Duration of the trigger pulse in seconds.
        min_range_mm (int): Minimum valid distance reading in mm.
        max_range_mm (int): Maximum valid distance reading in mm.

    Returns:
        float: The average distance in millimeters, rounded to 1 decimal place.
        None: If no valid readings could be obtained or an error occurred.
    """
    h = None
    distances_mm = []
    try:
        # Open GPIO chip device
        h = lgpio.gpiochip_open(gpio_chip)

        # Allow sensor to settle briefly after opening chip? Optional.
        # time.sleep(0.1)

        for i in range(num_readings):
            dist_mm = _measure_single_distance_mm(
                h, sig_pin, sound_speed_cm_s, timeout_s, trigger_pulse_s
            )

            if dist_mm is not None:
                # Check if reading is within plausible range
                if min_range_mm < dist_mm < max_range_mm:
                    distances_mm.append(dist_mm)
                # else: # Optional: print discarded out-of-range readings
                #    print(f" Discarding reading: {dist_mm:.1f} mm (Out of range)")
            # else: # Optional: print failed readings
            #    print(f" Reading {i+1}: Failed (Timeout or lgpio error)")

            if i < num_readings - 1: # Don't delay after the last reading
                 time.sleep(0.01)

        if not distances_mm:
            # print("No valid readings obtained.", file=sys.stderr) # Optional debug
            return None

        average_distance_mm = statistics.mean(distances_mm)
        return round(average_distance_mm, 1)

    except lgpio.error as e:
        print(f"Fatal lgpio error: {e}", file=sys.stderr)
        if "GPIO chip not found" in str(e):
             print(f"Try changing gpio_chip parameter (tried {gpio_chip}).", file=sys.stderr)
        elif "permission denied" in str(e):
             print("Ensure pigpiod daemon is running (sudo systemctl start pigpiod)", file=sys.stderr)
        return None # Indicate failure
    except Exception as e:
        print(f"An unexpected error occurred: {e}", file=sys.stderr)
        return None # Indicate failure
    finally:
        if h is not None:
            # No need to explicitly free pin here, _measure_single_distance_mm does it
            lgpio.gpiochip_close(h)
            # print(f"Closed GPIO chip {gpio_chip}") # Optional debug


if __name__ == '__main__':
    print("Starting ultrasonic sensor test...")

    # Example 1: Using default settings
    print("\nTaking measurement with default settings...")
    avg_dist = get_average_distance_mm()
    if avg_dist is not None:
        print("-" * 25)
        print(f"Average Distance: {avg_dist:.1f} mm")
        print("-" * 25)
    else:
        print("Failed to get measurement.")

    # Example 2: Using custom settings (e.g., more readings)
    print("\nTaking measurement with 10 readings...")
    avg_dist_custom = get_average_distance_mm(num_readings=10, read_delay_s=0.2)
    if avg_dist_custom is not None:
        print("-" * 25)
        print(f"Average Distance (10 readings): {avg_dist_custom:.1f} mm")
        print("-" * 25)
    else:
        print("Failed to get measurement with custom settings.")

    print("\nUltrasonic sensor test finished.")
