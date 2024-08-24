from sploitkit import Module, Config, Option
import nmap
from pymavlink import mavutil

class MavlinkScanner(Module):
    """Mavlink Scanner - Scans for common Mavlink ports 14550 (UDP) and 5760 (TCP) on a given IP or IP range and verifies if its actually Mavlink."""

    # Configuration for the Nmap scanner module
    config = Config({
        Option(
            name='target_ip',
            description='IP range of the network you want to scan',
            required=True,
        ): "10.13.0.3",  # Default value
    })

    def scan_ports(self, target_ip):
        """
        Use Nmap to scan for open ports 14550 (UDP) and 5760 (TCP) on the specified IP.
        """
        nm = nmap.PortScanner()

        # Define the ports to scan and specify protocols
        ports_to_scan = '14550,5760'
        arguments = '-sU -sS -p U:14550,T:5760'  # -sU for UDP scan, -p 14550,5760 for TCP port 5760

        # Perform the scan
        self.logger.info(f"Starting scan on IP range: {target_ip} for UDP port 14550 and TCP port 5760")
        nm.scan(target_ip, arguments=arguments)

        # Process and display the results
        for host in nm.all_hosts():
            for proto in nm[host].all_protocols():
                lport = nm[host][proto].keys()
                for port in lport:
                    port_state = nm[host][proto][port]['state']
                    if port_state == 'open':
                        self.logger.info(f"Host: {host}, Port: {port} ({proto.upper()}) is open. Verifying Mavlink...")
                        is_mavlink = self.verify_mavlink(host, port, proto)
                        if is_mavlink:
                            self.logger.success(f"{host}:{port} is running Mavlink.")
                        else:
                            self.logger.warning(f"{host}:{port} is open but cannot verify it's Mavlink.")
                    else:
                        self.logger.info(f"Host: {host}, Port: {port} ({proto.upper()}) is {port_state}.")

    def verify_mavlink(self, ip, port, protocol):
        """
        Verify if the open port is running Mavlink by waiting for a heartbeat.
        """
        try:
            if protocol == 'udp':
                connection = mavutil.mavlink_connection(f'udp:{ip}:{port}')
            elif protocol == 'tcp':
                connection = mavutil.mavlink_connection(f'tcp:{ip}:{port}')
            else:
                self.logger.warning(f"Unsupported protocol: {protocol.upper()}")
                return False

            connection.wait_heartbeat(timeout=10)  # Wait for a heartbeat with a 10-second timeout
            return True
        except Exception as e:
            self.logger.error(f"Error verifying Mavlink on {ip}:{port}: {e}")
            return False

    def run(self):
        # Retrieve the target IP range from the config
        target_ip = self.config['target_ip']

        # Perform the port scan
        self.scan_ports(target_ip)

