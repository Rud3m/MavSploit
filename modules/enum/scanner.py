from sploitkit import *
from pymavlink import mavutil

class MavlinkScanner(Module):
    def __init__(self):
        super().__init__()
        self.name = "Mavlink Scanner"
        self.description = "Scans for Mavlink devices on the network."

    def run(self, ip_range):
        print(f"Starting scan on IP range: {ip_range}")
        for ip in ip_range:
            try:
                connection = mavutil.mavlink_connection(f'udp:{ip}:14550')
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                print(f"Found Mavlink device at {ip}")
            except Exception as e:
                print(f"No Mavlink device found at {ip}: {str(e)}")

# Optionally, you can add configuration options or parameters as needed.
