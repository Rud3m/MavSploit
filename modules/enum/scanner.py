from sploitkit import Module, Config, Option
from pymavlink import mavutil
import ipaddress
import socket
from concurrent.futures import ThreadPoolExecutor, as_completed
import serial.tools.list_ports

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

def scan_serial_ports(baud_rate=57600, timeout=5):
    """
    Enumerate and test serial ports for MAVLink devices.
    Returns list of tuples: (port, description, vehicle_info)
    """
    found_devices = []

    # Enumerate all serial ports
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("No serial ports found on this system")
        return found_devices

    print(f"\nFound {len(ports)} serial ports, testing each for MAVLink...")

    for port_info in ports:
        port = port_info.device
        description = port_info.description

        print(f"\nTesting {port}: {description}")

        master = None
        try:
            # Try to connect to the serial port
            connection_string = f"{port},{baud_rate}"
            master = mavutil.mavlink_connection(connection_string)

            # Wait for heartbeat
            print(f"  Waiting for heartbeat (timeout: {timeout}s)...")
            heartbeat = master.wait_heartbeat(timeout=timeout)

            if heartbeat:
                print(f"  ✓ MAVLink device found!")
                print(f"    Vehicle Type: {heartbeat.type}")
                print(f"    Autopilot: {heartbeat.autopilot}")
                print(f"    System ID: {master.target_system}")
                print(f"    Component ID: {master.target_component}")

                # Collect device info
                device_info = {
                    'port': port,
                    'description': description,
                    'baud': baud_rate,
                    'vehicle_type': heartbeat.type,
                    'autopilot': heartbeat.autopilot,
                    'system_id': master.target_system,
                    'component_id': master.target_component,
                }

                found_devices.append(device_info)
            else:
                print(f"  ✗ No heartbeat received")

        except Exception as e:
            print(f"  ✗ Error: {str(e)}")
        finally:
            if master:
                master.close()

    return found_devices

class MavlinkScanner(Module):
    """
    Mavlink Scanner - Scans for Mavlink devices on network or enumerates serial ports.

    Connection:
        - Network mode: Scans IP range for MAVLink devices
        - Serial mode: Enumerates and tests local serial ports

    Usage:
        Network scanning:
            set mode network
            set ip_range 192.168.1.0/24
            run

        Serial enumeration:
            set mode serial
            set baud 57600
            run
    """

    # Configuration for the scanner module
    config = Config({
        Option(
            name='mode',
            description='Scan mode: network or serial',
            required=True,
        ): "network",  # Default value
        Option(
            name='ip_range',
            description='IP range for network scanning (CIDR notation)',
            required=False,
        ): "10.13.0.0/24",  # Default value
        Option(
            name='port',
            description='MAVLink ports for network scanning (comma-separated)',
            required=False,
        ): [14550, 14551, 5760, 5770],
        Option(
            name='baud',
            description='Baud rate for serial port testing',
            required=False,
        ): "57600",  # Default value
        Option(
            name='serial_timeout',
            description='Timeout in seconds when testing serial ports',
            required=False,
        ): "5",  # Default value
    })

    def run(self):
        mode = self.config['mode'].lower()

        separator = "=" * 80
        self.logger.info(separator)
        self.logger.info("MAVLink Scanner")
        self.logger.info(separator)

        if mode == 'network':
            # Network scanning mode
            ip_range = self.config['ip_range']
            port = self.config['port']

            # Ensure that the port is always a list of integers
            if not isinstance(port, list):
                port = [port]
            port = [int(p) for p in port]

            self.logger.info(f"Mode: Network Scanning")
            self.logger.info(f"IP Range: {ip_range}")
            self.logger.info(f"Ports: {port}")
            self.logger.info(separator)

            print(f"\nScanning IP range {ip_range} on ports {port}...")
            successful_services = scan_ip_range(ip_range, port)

            print(f"\n{separator}")
            if successful_services:
                self.logger.success(f"Found {len(successful_services)} MAVLink devices:")
                for ip, port, protocol in successful_services:
                    self.logger.success(f"  - {protocol}:{ip}:{port}")
            else:
                self.logger.warning("No MAVLink services found in the specified IP range")
            print(f"{separator}")

        elif mode == 'serial':
            # Serial enumeration mode
            baud = int(self.config['baud'])
            timeout = int(self.config['serial_timeout'])

            self.logger.info(f"Mode: Serial Port Enumeration")
            self.logger.info(f"Baud Rate: {baud}")
            self.logger.info(f"Timeout: {timeout}s")
            self.logger.info(separator)

            found_devices = scan_serial_ports(baud, timeout)

            print(f"\n{separator}")
            if found_devices:
                self.logger.success(f"Found {len(found_devices)} MAVLink devices on serial ports:\n")
                for device in found_devices:
                    self.logger.success(f"Port: {device['port']}")
                    self.logger.info(f"  Description: {device['description']}")
                    self.logger.info(f"  Baud Rate: {device['baud']}")
                    self.logger.info(f"  Vehicle Type: {device['vehicle_type']}")
                    self.logger.info(f"  Autopilot: {device['autopilot']}")
                    self.logger.info(f"  System ID: {device['system_id']}")
                    self.logger.info(f"  Component ID: {device['component_id']}")
                    print()
            else:
                self.logger.warning("No MAVLink devices found on serial ports")
            print(f"{separator}")

        else:
            self.logger.error(f"Invalid mode: {mode}")
            self.logger.info("Supported modes: network, serial")