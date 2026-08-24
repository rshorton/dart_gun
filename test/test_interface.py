#!/usr/bin/env python3
import sys
import time
import json
import argparse
import serial

def send_and_receive(ser, payload):
    """Encodes JSON payload, writes to serial port, and prints response packet."""
    # Convert dictionary to minimized JSON string with a newline terminator
    raw_send = json.dumps(payload) + "\n"
    print(f"--> Sending: {raw_send.strip()}")
    
    # Flush incoming buffers to clear old messages, then transmit
    ser.reset_input_buffer()
    ser.write(raw_send.encode('utf-8'))
    
    # Read the response line from the hardware board
    raw_response = ser.readline().decode('utf-8').strip()
    print(f"<-- Received: {raw_response}\n")
    return raw_response

def main():
    parser = argparse.ArgumentParser(description="Arduino Serial Command Line Test Tool")
    parser.add_argument('-p', '--port', default='/dev/ttyACM0', help="Serial port path (e.g., /dev/ttyACM0 or /dev/ttyUSB0)")
    parser.add_argument('-b', '--baud', type=int, default=115200, help="Baud rate matching Arduino setup")
    args = parser.parse_args()

    print(f"Connecting to hardware interface at {args.port} ({args.baud} baud)...")
    try:
        # Open port with a 2-second timeout window to prevent hanging loops
        ser = serial.Serial(args.port, args.baud, timeout=10)
        # Allow bootloader time window to finish resetting the chip
        time.sleep(2) 
    except Exception as e:
        print(f"Error opening port: {e}")
        print("Tip: Check device connection or permissions ('sudo chmod a+rw /dev/ttyACM0')")
        sys.exit(1)

    print("Connection established. Beginning operational tests...\n")

    get_status_cmd = {"name": "get_status"}
    send_and_receive(ser, get_status_cmd)
    time.sleep(1.0);

    fire_cmd = {
        "name": "fire",
        "args": {
            "speed": 3,
            "pan_angle": -25,
            "tilt_angle": 10,
            "count": 1
        }
    }
    send_and_receive(ser, fire_cmd)
    time.sleep(1.0);

    fire_cmd["args"]["pan_angle"] = 25;
    send_and_receive(ser, fire_cmd)
    time.sleep(1.0);

    send_and_receive(ser, get_status_cmd)
    time.sleep(1.0);

    reset_cmd = {"name": "reset"}
    send_and_receive(ser, reset_cmd)

    time.sleep(1.0);

    ser.close()
    print("Testing sequence finished cleanly.")

if __name__ == "__main__":
    main()
