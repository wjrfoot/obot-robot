#!/usr/bin/env python3
import csv
import socket
import struct
import time
from datetime import datetime
from pathlib import Path

CAN_RAW = 1
CAN_RTR_FLAG = 0x40000000

IDS = {
    0x7B7: "left",
    0x7D7: "right",
}

sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)
sock.bind(("can0",))
sock.settimeout(0.01)

filename = Path.home() / (
    f"obot_braking_{datetime.now():%Y%m%d_%H%M%S}.csv"
)

max_voltage = {"left": 0.0, "right": 0.0}
samples = {"left": 0, "right": 0}

print(f"Recording to {filename}")
print("Press Ctrl+C to stop and display results.")

try:
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "controller", "voltage_V", "current_A"
        ])

        next_poll = time.monotonic()

        while True:
            now = time.monotonic()

            if now >= next_poll:
                for can_id in IDS:
                    frame = struct.pack(
                        "=IB3x8s",
                        can_id | CAN_RTR_FLAG,
                        0,
                        bytes(8)
                    )
                    sock.send(frame)

                next_poll = now + 0.05

            try:
                frame = sock.recv(16)
            except socket.timeout:
                continue

            can_id, dlc, data = struct.unpack("=IB3x8s", frame)

            # Ignore remote requests and unrelated frames.
            if can_id & CAN_RTR_FLAG:
                continue

            can_id &= 0x7FF

            if can_id not in IDS or dlc < 8:
                continue

            voltage, current = struct.unpack("<ff", data)
            controller = IDS[can_id]

            timestamp = datetime.now().isoformat(timespec="milliseconds")

            writer.writerow([
                timestamp, controller,
                round(voltage, 4),
                round(current, 4)
            ])

            samples[controller] += 1
            max_voltage[controller] = max(
                max_voltage[controller], voltage
            )

except KeyboardInterrupt:
    pass

finally:
    sock.close()

    print("\nResults:")
    for controller in ("left", "right"):
        print(
            f"{controller}: "
            f"{samples[controller]} samples, "
            f"maximum voltage = {max_voltage[controller]:.3f} V"
        )

    print(f"\nCSV saved: {filename}")
