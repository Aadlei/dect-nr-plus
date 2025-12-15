import paho.mqtt.client as mqtt
import serial
import time
import json
import re


class DeviceDataParser:
    def __init__(self):
        self.current_transmitter_id = None
        self.current_receiver_id = None

    def parse_line(self, line):
        # Extract receiver ID if present (adjust pattern if you have this somewhere)
        receiver_match = re.search(r'receiver id:?\s*(\d+)', line, re.IGNORECASE)
        if receiver_match:
            self.current_receiver_id = int(receiver_match.group(1))
            return None
        
        # Look for JSON data after "Data:" prefix
        if 'Data:' in line or '{"msg_type"' in line:
            json_match = re.search(r'\{.*\}', line)
            if json_match:
                json_str = json_match.group()
                try:
                    data = json.loads(json_str)
                    
                    # Extract transmitter ID from JSON
                    transmitter_id = data.get('transmitter_long_id')
                    
                    if transmitter_id:
                        structured_data = {
                            "transmitter_id": transmitter_id,
                            "receiver_id": self.current_receiver_id,
                            "timestamp": time.time(),
                            "payload": {
                                "msg_type": data.get('msg_type'),
                                "msg": data.get('msg'),
                                "m_tmp": data.get('m_tmp')
                            }
                        }
                        return structured_data
                except json.JSONDecodeError as e:
                    print(f"Invalid JSON: {json_str} - Error: {e}")
        
        return None
    
def on_connect(client, userdata, connect_flags, reason_code, properties):
    if reason_code == 0:
        print("✅ Connected to MQTT Broker!")
    else:
        print(f"❌ Failed to connect, return code {reason_code}")


if __name__ == '__main__':
    # MQTT Setup
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "SerialToMQTT")
    mqtt_client.on_connect = on_connect
    mqtt_client.connect("10.225.150.248", 1883, 60)
    mqtt_client.loop_start()
    
    # Serial Setup
    ser = serial.Serial('COM9', 115200)
    parser = DeviceDataParser()
    
    # Chica code
    try:
        print("Listening to serial port...")
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                structured_data = parser.parse_line(line)
                
                if structured_data:
                    print(f"📡 Device {structured_data['transmitter_id']} → {structured_data['receiver_id']}")
                    print(f"   Message: {structured_data['payload']['msg']}")
                    print(f"   Temperature: {structured_data['payload']['m_tmp']}°C")
                    print(f"   Type: {structured_data['payload']['msg_type']}")
                    print(f"   Time: {time.strftime('%H:%M:%S', time.localtime(structured_data['timestamp']))}")
                    print("-" * 40)
                    
                    # Publish to MQTT
                    topic = f"devices/tx{structured_data['transmitter_id']}/rx{structured_data['receiver_id']}"
                    mqtt_client.publish(topic, json.dumps(structured_data))
                    print(f"📤 Published to {topic}")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        ser.close()
        print("Serial and MQTT closed.")

