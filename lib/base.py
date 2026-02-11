"""
Base module for MAVLink connections.

Provides unified connection handling for both network (UDP/TCP) and serial connections.
"""

from sploitkit import Module, Config, Option
# Force MAVLink 2.0 globally (better message routing and compatibility)
import os
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
import re





class MAVLinkModule(Module):
    """
    Base class for MAVLink modules with unified connection handling.

    Supports both network and serial connections:
        Network:
            - udp:IP:PORT       - Bidirectional (binds locally, good for receiving)
            - udpout:IP:PORT    - Send-only (for spoofing - sends TO the port)
            - udpin:IP:PORT     - Listen-only (receives from the port)
            - udpbcast:IP:PORT  - Broadcast mode
            - tcp:IP:PORT       - TCP connection
        Serial:
            - /dev/ttyUSB0, /dev/ttyACM0 (Linux)
            - COM1, COM3 (Windows)

    For spoofing/injection attacks, use 'udpout:' to ensure packets are sent
    TO the target port (14550) rather than FROM port 14550.

    Usage:
        class MyModule(MAVLinkModule):
            config = MAVLinkModule.config + Config({
                # Add module-specific options here
            })

            def run(self):
                master = self.connect_drone()
                # Use master for MAVLink communication
                self.close_connection(master)
    """

    # Default config that child classes will extend
    config = Config({
        Option(
            name='connection',
            description='Connection string: /dev/ttyUSB0 for serial, udp:IP:PORT or udpout:IP:PORT for network',
            required=True,
        ): "udpout:127.0.0.1:14550",
        Option(
            name='baud',
            description='Baud rate for serial connections (common: 57600, 115200, 921600)',
            required=False,
        ): "57600",
    })

    def parse_connection_string(self, connection):
        """
        Parse connection string and determine connection type.

        Supported formats:
            Serial (Linux):   /dev/ttyUSB0, /dev/ttyACM0, /dev/serial0
            Serial (Windows): COM1, COM3, COM10
            Network (UDP):    udp:192.168.1.1:14550    (bidirectional)
                              udpout:192.168.1.1:14550 (send-only, for spoofing)
                              udpin:0.0.0.0:14550      (listen-only)
                              udpbcast:192.168.1.255:14550 (broadcast)
            Network (TCP):    tcp:192.168.1.1:5760
                              tcpin:0.0.0.0:5760       (listen mode)

        For spoofing/injection: Use 'udpout:' to send TO the target port
            - udpout:10.0.0.5:14550 sends to destination port 14550
            - udp:10.0.0.5:14550 binds to local port 14550

        Examples:
            >>> parse_connection_string('/dev/ttyUSB0')
            ('/dev/ttyUSB0', 'serial')

            >>> parse_connection_string('udpout:192.168.1.1:14550')
            ('udpout:192.168.1.1:14550', 'network')

            >>> parse_connection_string('COM3')
            ('COM3', 'serial')

        Args:
            connection (str): Connection string

        Returns:
            tuple: (connection_string, connection_type)
                connection_type: 'serial' or 'network'

        Raises:
            ValueError: If connection string format is invalid
        """
        connection = connection.strip()

        # Check for serial device patterns
        if connection.startswith('/dev/'):
            # Linux/Unix serial devices: /dev/ttyUSB0, /dev/ttyACM0, /dev/serial0
            return (connection, 'serial')
        elif re.match(r'^COM\d+$', connection, re.IGNORECASE):
            # Windows COM ports: COM1, COM3, COM10, etc.
            return (connection.upper(), 'serial')
        elif ':' in connection:
            # Network connections: udp:IP:PORT, tcp:IP:PORT, udpin:IP:PORT, etc.
            return (connection, 'network')
        else:
            raise ValueError(
                f"Invalid connection string: '{connection}'\n"
                f"Supported formats:\n"
                f"  Serial (Linux):  /dev/ttyUSB0, /dev/ttyACM0\n"
                f"  Serial (Windows): COM1, COM3\n"
                f"  Network (UDP):    udp:192.168.1.1:14550\n"
                f"  Network (TCP):    tcp:192.168.1.1:5760"
            )

    def connect_drone(self, connection=None, baud=None, timeout=10, skip_heartbeat=False, source_system=None, source_component=None):
        """
        Establish connection to MAVLink device (serial or network).

        This method automatically detects whether to use serial or network
        connection based on the connection string format.

        Args:
            connection (str): Connection string (if None, uses self.config['connection'])
            baud (int): Baud rate for serial (if None, uses self.config['baud'])
            timeout (int): Heartbeat timeout in seconds (default: 10)
            skip_heartbeat (bool): Skip waiting for heartbeat (for send-only connections)
            source_system (int): Source system ID (if None, uses config or default)
            source_component (int): Source component ID (if None, uses config or default)

        Returns:
            mavutil.mavlink_connection: Connected MAVLink master instance

        Raises:
            ValueError: If connection string is invalid
            Exception: If connection fails or timeout waiting for heartbeat

        Examples:
            # Network connection (bidirectional)
            master = self.connect_drone('udp:192.168.1.1:14550')

            # Network output-only (for spoofing - sends TO port 14550)
            master = self.connect_drone('udpout:192.168.1.1:14550', source_system=1)

            # Serial connection
            master = self.connect_drone('/dev/ttyUSB0', baud=57600)

            # Using config values
            master = self.connect_drone()
        """
        # Use config values if not provided
        if connection is None:
            connection = self.config['connection']
        if baud is None:
            baud = int(self.config['baud'])

        # Get source IDs from config if available (try/except to handle modules without these options)
        if source_system is None:
            try:
                source_system = int(self.config['src_system_id'])
            except (KeyError, ValueError):
                pass  # Module doesn't have src_system_id option

        if source_component is None:
            try:
                source_component = int(self.config['src_component_id'])
            except (KeyError, ValueError):
                pass  # Module doesn't have src_component_id option

        # Parse connection string to determine type
        conn_str, conn_type = self.parse_connection_string(connection)

        # Check if this is a send-only connection
        is_send_only = conn_str.startswith('udpout:') or conn_str.startswith('udpbcast:')

        # Build full connection string based on type
        if conn_type == 'serial':
            # Serial connection: add baud rate
            # pymavlink format: /dev/ttyUSB0,57600
            full_conn_str = f"{conn_str},{baud}"
            self.logger.info(f"Connecting to serial device: {conn_str} @ {baud} baud")
        else:
            # Network connection: use as-is
            full_conn_str = conn_str
            if is_send_only:
                self.logger.info(f"Opening send-only connection: {conn_str}")
            else:
                self.logger.info(f"Connecting to network: {conn_str}")

        try:
            # Establish connection using pymavlink with source IDs
            if source_system is not None:
                self.logger.info(f"Setting source system ID: {source_system}")
                if source_component is not None:
                    self.logger.info(f"Setting source component ID: {source_component}")
                    master = mavutil.mavlink_connection(full_conn_str, source_system=source_system, source_component=source_component)
                else:
                    master = mavutil.mavlink_connection(full_conn_str, source_system=source_system)
            else:
                master = mavutil.mavlink_connection(full_conn_str)

            # For send-only connections (udpout, udpbcast), skip heartbeat wait
            if is_send_only or skip_heartbeat:
                if is_send_only:
                    self.logger.success(f"Send-only connection established (type: {conn_type})")
                    self.logger.info("Note: Using udpout - packets will send TO the specified port")
                else:
                    self.logger.success(f"Connection established (type: {conn_type})")
                return master

            # Wait for heartbeat to confirm bidirectional connection
            self.logger.info(f"Waiting for heartbeat (timeout: {timeout}s)...")
            heartbeat = master.wait_heartbeat(timeout=timeout)

            if heartbeat:
                self.logger.success(f"Connected to MAVLink device (type: {conn_type})")
                self.logger.info(f"System ID: {master.target_system}")
                self.logger.info(f"Component ID: {master.target_component}")
                self.logger.info(f"MAVLink version: {master.WIRE_PROTOCOL_VERSION}")
            else:
                raise Exception(f"Timeout waiting for heartbeat after {timeout}s")

            return master

        except Exception as e:
            self.logger.error(f"Connection failed: {str(e)}")
            self.logger.error(f"Connection string: {full_conn_str}")

            # Provide helpful troubleshooting hints
            if conn_type == 'serial':
                self.logger.error(
                    "Troubleshooting tips for serial connection:\n"
                    "  - Check device exists: ls /dev/ttyUSB* /dev/ttyACM*\n"
                    "  - Check permissions: sudo chmod 666 /dev/ttyUSB0\n"
                    "  - Try different baud rates: 57600, 115200, 921600\n"
                    "  - Verify device is MAVLink-enabled"
                )
            else:
                self.logger.error(
                    "Troubleshooting tips for network connection:\n"
                    "  - Check target is reachable: ping <IP>\n"
                    "  - Verify port is correct (common: 14550, 14551, 5760)\n"
                    "  - Check firewall settings\n"
                    "  - For spoofing, use 'udpout:IP:PORT' to send TO the port\n"
                    "  - 'udp:IP:PORT' binds locally, 'udpout:IP:PORT' sends only"
                )

            raise

    def close_connection(self, master):
        """
        Safely close MAVLink connection.

        Args:
            master: MAVLink master instance to close
        """
        if master:
            try:
                master.close()
                self.logger.info("Connection closed successfully")
            except Exception as e:
                self.logger.debug(f"Error closing connection: {str(e)}")
