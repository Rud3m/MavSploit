from sploitkit import Module, Config, Option
from pymavlink import mavutil
import ipaddress
import socket
from concurrent.futures import ThreadPoolExecutor, as_completed

def scan_ip_range(cidr_range, ports):
    successful_services = []

    network = ipaddress.ip_network(cidr_range, strict=False)
    tasks = []
    with ThreadPoolExecutor(max_workers=100) as executor:
        for ip in network.hosts():
            for port in ports:
                tasks.append(executor.submit(scan_ip_port, str(ip), port))
        
        for future in as_completed(tasks):
            result = future.result()
            if result:
                successful_services.append(result)
    
    return successful_services

# Function to scan a single IP and port combination
def scan_ip_port(ip, port):
    if check_mavlink_service(ip, port, protocol="tcp"):
        if probe_service(ip, port, protocol="tcp"):
            return (ip, port, "tcp")
    elif check_mavlink_service(ip, port, protocol="udp"):
        if probe_service(ip, port, protocol="udp"):
            return (ip, port, "udp")
    return None

# Function to check if a MAVLink service is available on a specific IP and port, for a given protocol
def check_mavlink_service(ip, port, protocol="udp"):
    try:
        if protocol == "udp":
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.settimeout(3)
                sock.sendto(b"", (ip, port))
                return True
        elif protocol == "tcp":
            with socket.create_connection((ip, port), timeout=3) as sock:
                return True
    except (socket.timeout, ConnectionRefusedError, OSError):
        return False

# Function to probe a MAVLink service using pymavlink and gather vehicle information
def probe_service(ip, port, protocol="udp"):
    master = None
    try:
        connection_string = f"{protocol}:{ip}:{port}"
        
        # Connecting to the MAVLink service using pymavlink's mavutil
        master = mavutil.mavlink_connection(connection_string)
        
        # Wait for heartbeat to confirm connection
        heartbeat = master.wait_heartbeat(timeout=5)
        if heartbeat:
            print(f"Connected to {ip}:{port} - Heartbeat received")
            print(f"Vehicle Type: {heartbeat.type}, Autopilot Type: {heartbeat.autopilot}, Base Mode: {heartbeat.base_mode}, System Status: {heartbeat.system_status}")
            
            # Request and print system status
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
                0, 0, 0, 0, 0, 0
            )
            sys_status = master.recv_match(type='SYS_STATUS', blocking=True, timeout=10)
            if sys_status:
                print(f"System Status: Battery Remaining: {sys_status.battery_remaining}%")
            
            # Request and print autopilot version (firmware details)
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                0, 0, 0, 0, 0, 0
            )
            autopilot_version = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=10)
            if autopilot_version:
                print(f"Autopilot Version: {autopilot_version.flight_sw_version}, Vendor ID: {autopilot_version.vendor_id}, Product ID: {autopilot_version.product_id}")
                return True
            else:
                print(f"No autopilot version received from {ip}:{port}")
        else:
            print(f"Failed to receive heartbeat from {ip}:{port}")
            
    except Exception as e:
        # print(f"Error probing {ip}:{port} over {protocol} - {str(e)}")
        pass
    finally:
        if master:
            master.close()
    return False

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
            required=False,
            ): [14550, 14551, 5760, 5770]
    })

    def run(self):
        ip_range = self.config['ip_range']  # Retrieve the IP range from config
        port = self.config['port']  # Retrieve the port from config

        # Ensure that the port is always a list of integers
        if not isinstance(port, list):
            port = [port]
        port = [int(p) for p in port]  # Convert all port values to integers

        print(f"Scanning IP range {ip_range} on port {port}...")
        successful_services = scan_ip_range(ip_range, port)

        if successful_services:
            print(f"Successfully detected MAVLink from: {successful_services}")
        else:
            print("No MAVLink services with vehicle details found in the specified IP range.")