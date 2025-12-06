import serial
import struct
import time
import sys
import numpy as np

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'  # <--- CHECK YOUR PORT
BAUD_RATE = 115200

# Firmware Protocol:
# Header (1) + Length (4) + Payload (64) = 69 Bytes
TOTAL_PACKET_SIZE = 69 
PAYLOAD_CAPACITY = 64

# --- Protocol IDs ---
ACK_ID = 0x70
NACK_ID = 0x71
DATA_ID = 0x72
DISCONNECT_ID = 0x73

class DroneCalibrator:
    def __init__(self, port):
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=5.0)
            self.ser.reset_input_buffer()
            print(f"[+] Opened serial port {port}")
        except serial.SerialException as e:
            print(f"[!] Could not open port: {e}")
            sys.exit(1)

    def send_packet(self, data: bytes):
        if len(data) > PAYLOAD_CAPACITY:
            raise ValueError("Data too long for packet")
        payload = data + b'\x00' * (PAYLOAD_CAPACITY - len(data))
        self.ser.write(payload)

    def receive_packet(self):
        # Expect exactly 69 bytes
        raw_bytes = self.ser.read(TOTAL_PACKET_SIZE)
        
        if len(raw_bytes) != TOTAL_PACKET_SIZE:
            return None, None

        # Unpack: ID (1 byte), Length (4 bytes), Payload (64 bytes)
        resp_id, valid_len, payload_buffer = struct.unpack('<BI64s', raw_bytes)
        
        # Extract valid data
        real_payload = payload_buffer[0:valid_len]
        return resp_id, real_payload

    def wait_for_ack(self, context="Command"):
        resp_id, _ = self.receive_packet()
        if resp_id == ACK_ID:
            return True
        elif resp_id == NACK_ID:
            print(f"[!] NACK received for: {context}")
            return False
        elif resp_id == DATA_ID:
             print(f"[!] Unexpected DATA packet received during: {context}")
             return False
        elif resp_id is None:
            print(f"[!] Timeout waiting for: {context}")
            return False
        else:
            print(f"[?] Unknown ID {hex(resp_id)} received for: {context}")
            return False

    def calibrate_accel(self):
        print("\n--- Accelerometer Calibration (6-Point) ---")
        self.send_packet(b"ACCELPX")
        if not self.wait_for_ack("Enter ACCELPX Mode"): return

        steps = [
            "Positive X (Nose Up)", "Negative X (Nose Down)",
            "Positive Y (Left Wing Up)", "Negative Y (Right Wing Up)",
            "Positive Z (Upside Down)", "Negative Z (Level)"
        ]

        for i, desc in enumerate(steps):
            print(f"\n[Step {i+1}/6] Align Drone: {desc}")
            input("    Press Enter when stable...")
            print("    Sampling... (Hold still for ~1.5s)")
            self.send_packet(b"NEXT") # Content doesn't matter, just triggers next step

            resp_id, payload = self.receive_packet()
            
            if i == len(steps) - 1: # Final step
                if resp_id == DATA_ID:
                    self.parse_accel_data(payload)
                else:
                    print(f"[!] Error: Expected DATA, got {hex(resp_id) if resp_id else 'None'}")
            else:
                if resp_id == ACK_ID:
                    print("    Step captured.")
                else:
                    print(f"[!] Step failed. ID: {hex(resp_id) if resp_id else 'None'}")
                    return

    def calibrate_gyro(self):
        print("\n--- Gyroscope Calibration ---")
        print("Ensure the drone is completely stationary on a flat surface.")
        
        # 1. Enter Gyro Mode
        print("\n[1/2] Entering Gyro Mode...")
        self.send_packet(b"GYRO")
        if not self.wait_for_ack("Enter GYRO Mode"):
            return
        print("[+] Mode Set.")

        # 2. Trigger Calibration
        print("\n[2/2] Ready to Calibrate.")
        input("    Press Enter to start sampling...")
        
        print("    Sending trigger...", end='', flush=True)
        self.send_packet(b"NEXT") # The content doesn't matter, just triggers 'process'
        print(" Sampling... (Hold still for ~2s)")

        # 3. Receive Data
        resp_id, payload = self.receive_packet()
        
        if resp_id == DATA_ID:
            print("[+] Calibration Complete! Processing Data...")
            self.parse_gyro_data(payload)
        elif resp_id == ACK_ID:
            print("[!] Received ACK but expected DATA. Logic mismatch?")
        else:
             print(f"[!] Failed. Response ID: {hex(resp_id) if resp_id else 'None'}")

    def parse_accel_data(self, data):
        # Expects 48 bytes: 9 floats (Matrix) + 3 floats (Bias)
        if len(data) < 48:
            print(f"[!] Error: Insufficient data {len(data)}/48 bytes")
            return
        
        unpacked = struct.unpack('<9f3f', data[0:48])
        s_inv_vals = unpacked[0:9]
        bias_vals = unpacked[9:12]
        
        s_inv = np.array(s_inv_vals).reshape(3, 3, order='F') # Column-major
        bias = np.array(bias_vals)

        print("\n" + "="*50)
        print("✅ ACCELEROMETER RESULTS")
        print("="*50)
        print("Inverse Sensitivity Matrix (S_inv):\n", s_inv)
        print("\nBias Vector (b):\n", bias)
        
        print("\nvvv COPY INTO main.rs vvv")
        print(f"inv_sens_matrix = SMatrix::<f32, 3, 3>::from_column_slice(&[{', '.join(f'{x:.6f}' for x in s_inv_vals)}]);")
        print(f"accel_bias = SVector::<f32, 3>::from_column_slice(&[{', '.join(f'{x:.6f}' for x in bias_vals)}]);")
        print("^^^ COPY ABOVE ^^^")

    def parse_gyro_data(self, data):
        # Expects 12 bytes: 3 floats (Bias X, Y, Z)
        if len(data) < 12:
            print(f"[!] Error: Insufficient data {len(data)}/12 bytes")
            return

        gx, gy, gz = struct.unpack('<3f', data[0:12])
        
        print("\n" + "="*50)
        print("✅ GYRO RESULTS")
        print("="*50)
        print(f"Gyro Bias: X={gx:.4f}, Y={gy:.4f}, Z={gz:.4f}")
        print("\n(Note: Gyro bias is usually recalibrated on every boot,")
        print(" but you can use these as default start values if desired.)")

    def connect(self):
        print("Connecting to drone...")
        self.send_packet(b"CONNECT")
        if self.wait_for_ack("CONNECT"):
            print("[+] Connected!")
            return True
        return False

    def disconnect(self):
        try:
            self.send_packet(b"DISCONNECT")
            resp_id, _ = self.receive_packet()
            if resp_id == DISCONNECT_ID:
                print("[+] Drone acknowledged disconnect.")
                self.ser.close()
                print("\n[+] Disconnected.")
        except:
            pass

if __name__ == "__main__":
    calib = DroneCalibrator(SERIAL_PORT)
    try:
        if calib.connect():
            while True:
                print("\n--- Main Menu ---")
                print("1. Calibrate Accelerometer (6-Point)")
                print("2. Calibrate Gyroscope (Stationary)")
                print("q. Quit")
                choice = input("Select: ")
                
                if choice == '1':
                    calib.calibrate_accel()
                elif choice == '2':
                    calib.calibrate_gyro()
                elif choice == 'q':
                    break
    except KeyboardInterrupt:
        print("\n[!] Interrupted.")
    finally:
        calib.disconnect()