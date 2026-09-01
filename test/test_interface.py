#!/usr/bin/env python3
import sys
import time
import json
import argparse
import serial
import logging

fire_cmd = {
    "name": "fire",
    "args": {
        "speed": 3,
        "pan_angle": 0,
        "tilt_angle": 0,
        "count": 1
    }
}

def send_and_receive(ser, payload):
    """Encodes JSON payload, writes to serial port, and prints response packet."""
    raw_send = json.dumps(payload) + "\n"
    logging.info(f"--> Sending: {raw_send.strip()}")
    
    ser.reset_input_buffer()
    ser.write(raw_send.encode('utf-8'))
    
    raw_response = ser.readline().decode('utf-8').strip()
    logging.info(f"<-- Received: {raw_response}\n")
    return json.loads(raw_response)

# Send command and retry if previous command in progress
def send_with_retry(ser, payload, retry_period, num_retries):
    for i in range(1, num_retries):
        resp = send_and_receive(ser, payload)
        if resp["cmd_result"] != "busy":
            return
        time.sleep(retry_period)
        logging.info(f"gun is busy - retrying\n")        
    logging.info(f"cmd not accepted - gun busy\n")

def send_with_retry_until_not_pending(ser, payload, retry_period, num_retries):
    for i in range(1, num_retries):
        resp = send_and_receive(ser, payload)
        if resp["pending"] != True:
            return
        time.sleep(retry_period)
        logging.info(f"a cmd is pending - retrying\n")        

def test_status(ser):
    while True:
        get_status_cmd = {"name": "get_status"}
        send_and_receive(ser, get_status_cmd)
        time.sleep(1.0)

def test_fire(ser):
    fire_cmd["args"]["count"] = 2
    send_and_receive(ser, fire_cmd)
    time.sleep(3.0)

def test_fire_all(ser):
    get_status_cmd = {"name": "get_status"}
    resp = send_and_receive(ser, get_status_cmd)

    while bool(resp.get('empty', True)) != True:
        fire_cmd["args"]["count"] = 1
        resp = send_and_receive(ser, fire_cmd)
        time.sleep(1.0)
    test_status(ser)

def test_aim(ser):

    delay = 0.4

    fire_cmd["args"]["count"] = 0

    angle_list = [(10, -10), (30, -10), (45, -10), (55, -10)]

    for pan, tilt in angle_list:
        print(f"pan, tilt: {pan}, {tilt}")

        fire_cmd["args"]["pan_angle"] = pan
        fire_cmd["args"]["tilt_angle"] = tilt
        send_and_receive(ser, fire_cmd)

        time.sleep(delay)

def test_reset(ser):
    reset_cmd = {"name": "reset"}
    send_and_receive(ser, reset_cmd)
    time.sleep(2.0)

def test_all(ser):
    get_status_cmd = {"name": "get_status"}
    send_and_receive(ser, get_status_cmd)
    time.sleep(1.0)

    retry_interval = 0.1
    retry_attempts = 50

    # Send fire command, but wait each time for the gun
    # to finish the previous cmd
    send_with_retry(ser, fire_cmd, retry_interval, retry_attempts)

    fire_cmd["args"]["pan_angle"] = 25
    send_with_retry(ser, fire_cmd, retry_interval, retry_attempts)

    fire_cmd["args"]["pan_angle"] = 60
    send_with_retry(ser, fire_cmd, retry_interval, retry_attempts)

    fire_cmd["args"]["pan_angle"] = -60
    send_with_retry(ser, fire_cmd, retry_interval, retry_attempts)

    send_with_retry_until_not_pending(ser, get_status_cmd, retry_interval, retry_attempts)

    time.sleep(1.0)

    reset_cmd = {"name": "reset"}
    send_and_receive(ser, reset_cmd)

    time.sleep(2.0)


def main():
    parser = argparse.ArgumentParser(description="Arduino Serial Command Line Test Tool")
    parser.add_argument('-p', '--port', default='/dev/ttyACM0', help="Serial port path (e.g., /dev/ttyACM0 or /dev/ttyUSB0)")
    parser.add_argument('-b', '--baud', type=int, default=115200, help="Baud rate matching Arduino setup")
    args = parser.parse_args()

    logging.basicConfig(
        format='%(asctime)s.%(msecs)03d - %(levelname)s - %(message)s',
        level=logging.INFO,
        datefmt='%Y-%m-%d %H:%M:%S'
)
    logging.info(f"Connecting to hardware interface at {args.port} ({args.baud} baud)...")
    try:
        # Open port with a 2-second timeout window to prevent hanging loops
        ser = serial.Serial(args.port, args.baud, timeout=10)
        # Allow bootloader time window to finish resetting the chip
        time.sleep(2) 
    except Exception as e:
        logging.info(f"Error opening port: {e}")
        logging.info("Tip: Check device connection or permissions ('sudo chmod a+rw /dev/ttyACM0')")
        sys.exit(1)

    logging.info("Connection established. Beginning operational tests...\n")

    #test_status(ser)
    #test_fire(ser)
    #test_fire_all(ser)

    test_aim(ser)
    #test_reset(ser)
    #test_all(ser)

    ser.close()
    logging.info("Testing sequence finished cleanly.")

if __name__ == "__main__":
    main()
