from sploitkit import Module, Config, Option
from pymavlink import mavutil
import ipaddress

class MavlinkScanner(Module):
    """Mavlink Scanner - Scans for Mavlink devices on the network."""

    # Configuration for the scanner module
    config = Config({
        Option(
            name='ip_range',
            description='IP range of the network you want to scan',
            required=True,
            ): "10.13.0.0/24", # Default value
        Option(
            name='port',
            description='MavLink Port',
            required=True,
            ): "14550",  # Default value
    })

    def run(self):
            ip_range = self.config['ip_range'] # Retrieve the IP range from config
            port = self.config['port']  # Retrieve the port from config

            # Convert the IP range into a list of IP addresses
            network = ipaddress.ip_network(ip_range, strict=False)

            for ip in network:
                try:
                    # Attempt to connect to each IP in the range on the specified port
                    connection = mavutil.mavlink_connection(f'udp:{ip}:{port}')
                    # Send a heartbeat to check if it's a Mavlink device
                    connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0
                    )
                    print(f"Found Mavlink device at {ip}")
                except Exception as e:
                    print(f"No Mavlink device found at {ip}: {str(e)}")