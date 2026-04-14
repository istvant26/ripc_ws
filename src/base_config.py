#!/usr/bin/env python3
"""
Nuclear option: Send RTCM 1005 enable command 10 times
"""
import serial
import time

def ubx_checksum(msg):
    ck_a = ck_b = 0
    for byte in msg:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def send_ubx(ser, msg_class, msg_id, payload):
    header = bytes([0xB5, 0x62, msg_class, msg_id])
    length = len(payload).to_bytes(2, 'little')
    msg = header + length + payload
    checksum = ubx_checksum(msg[2:])
    ser.write(msg + checksum)
    time.sleep(0.3)

device = '/dev/ttyACM0'
ser = serial.Serial(device, 115200, timeout=1)

print("Sending RTCM 1005 enable command 10 times...")

for i in range(10):
    print(f"  Attempt {i+1}/10")
    
    # Enable RTCM 1005 on USB
    payload = bytes([0x00, 0x07, 0x00, 0x00])  # RAM + BBR + Flash
    payload += (0x209102bf).to_bytes(4, 'little')  # CFG-MSGOUT-RTCM_3X_TYPE1005_USB
    payload += bytes([1])  # Rate = 1
    
    send_ubx(ser, 0x06, 0x8a, payload)
    time.sleep(0.5)

# Save 3 times
print("\nSaving configuration 3 times...")
for i in range(3):
    save_payload = bytes([0x00, 0x00, 0x00, 0x00])
    save_payload += bytes([0x1F, 0x1F, 0x00, 0x00])
    save_payload += bytes([0x00, 0x00, 0x00, 0x00])
    save_payload += bytes([0x1F])
    
    send_ubx(ser, 0x06, 0x09, save_payload)
    time.sleep(1)

ser.close()

print("\n✓ Done. Wait 5 seconds then run: python3 test_script.py")
print("You MUST see 'RTCM Type: 1005' or RTK will not work!")