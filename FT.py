import serial
import time
import json
import re


class DeviceDataParser:
    def __init__(self):
        self.current_transmitter_id = None
        self.current_receiver_id = None

    def parse_line(self, line):
        # Extract transmitter ID
        transmitter_match = re.search(r'transmitter id (\d+)', line)
        if transmitter_match:
            self.current_transmitter_id = int(transmitter_match.group(1))
            return None

        # Extract receiver ID
        receiver_match = re.search(r'receiver id: (\d+)', line)
        if receiver_match:
            self.current_receiver_id = int(receiver_match.group(1))
            return None

        # Extract JSON data
        if '{"data":' in line:
            json_match = re.search(r'\{.*\}', line)
            if json_match:
                json_str = json_match.group()
                try:
                    data = json.loads(json_str)
                    # Structure the data for easy forwarding
                    structured_data = {
                        "transmitter_id": self.current_transmitter_id,
                        "receiver_id": self.current_receiver_id,
                        "timestamp": time.time(),
                        "payload": data
                    }
                    return structured_data
                except json.JSONDecodeError:
                    print(f"Invalid JSON: {json_str}")

        return None


if __name__ == '__main__':
    ser = serial.Serial('COM8', 115200)
    parser = DeviceDataParser()

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()

                structured_data = parser.parse_line(line)
                if structured_data:
                    print(f"📡 Device {structured_data['transmitter_id']} → {structured_data['receiver_id']}")
                    print(f"   Message: {structured_data['payload']['data']}")
                    print(f"   Temperature: {structured_data['payload']['m_tmp']}°C")
                    print(f"   Time: {time.strftime('%H:%M:%S', time.localtime(structured_data['timestamp']))}")
                    print("-" * 40)

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()
        print("Serial closed.")

