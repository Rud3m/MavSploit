from sploitkit import Module, Config, Option
import serial.tools.list_ports
from pymavlink import mavutil
import platform

class SerialPortDetector(Module):
    """
    Automatically detect and enumerate all available serial ports on the system.

    This module scans for USB serial devices and displays detailed information
    including device paths, descriptions, hardware IDs, and optionally tests
    for MAVLink connectivity.

    Usage:
        use enum/serial_port_detector
        set test_mavlink true
        set baud 57600
        run
    """

    config = Config({
        Option(
            name='test_mavlink',
            description='Test each port for MAVLink heartbeat (slower but more informative)',
            required=False,
        ): "false",
        Option(
            name='baud',
            description='Baud rate to use when testing MAVLink connectivity',
            required=False,
        ): "57600",
        Option(
            name='timeout',
            description='Timeout in seconds when testing for MAVLink heartbeat',
            required=False,
        ): "3",
    })

    def get_all_ports(self):
        """
        Enumerate all serial ports available on the system.
        Returns a list of port info objects.
        """
        ports = serial.tools.list_ports.comports()
        return sorted(ports, key=lambda p: p.device)

    def test_mavlink_port(self, port_path, baud, timeout):
        """
        Test if a serial port has a MAVLink device connected.
        Returns dict with connection info or None if no MAVLink detected.
        """
        master = None
        try:
            connection_string = f"{port_path},{baud}"
            master = mavutil.mavlink_connection(connection_string)

            # Wait for heartbeat
            heartbeat = master.wait_heartbeat(timeout=timeout)

            if heartbeat:
                # Get vehicle type and autopilot names
                vehicle_type = mavutil.mavlink.enums['MAV_TYPE'].get(heartbeat.type, {}).get('name', 'UNKNOWN')
                autopilot = mavutil.mavlink.enums['MAV_AUTOPILOT'].get(heartbeat.autopilot, {}).get('name', 'UNKNOWN')

                return {
                    'has_mavlink': True,
                    'vehicle_type': vehicle_type,
                    'autopilot': autopilot,
                    'system_id': master.target_system,
                    'component_id': master.target_component,
                    'base_mode': heartbeat.base_mode,
                    'system_status': heartbeat.system_status,
                }
        except Exception as e:
            # Port exists but no MAVLink or connection failed
            return {'has_mavlink': False, 'error': str(e)}
        finally:
            if master:
                try:
                    master.close()
                except:
                    pass

        return {'has_mavlink': False}

    def format_size(self, value):
        """Helper to format values, handling None"""
        return str(value) if value else "N/A"

    def run(self):
        test_mavlink = self.config['test_mavlink'].lower() in ['true', 'yes', '1']
        baud = int(self.config['baud'])
        timeout = int(self.config['timeout'])

        separator = "=" * 80
        self.logger.info(separator)
        self.logger.info("Serial Port Auto-Detection")
        self.logger.info(separator)
        self.logger.info(f"Operating System: {platform.system()} {platform.release()}")
        self.logger.info(f"MAVLink Testing: {'Enabled' if test_mavlink else 'Disabled'}")
        if test_mavlink:
            self.logger.info(f"Test Baud Rate: {baud}")
            self.logger.info(f"Test Timeout: {timeout}s")
        self.logger.info(separator)

        # Get all serial ports
        ports = self.get_all_ports()

        if not ports:
            self.logger.warning("\nNo serial ports detected on this system")
            self.logger.info("\nPossible reasons:")
            self.logger.info("  - No USB serial devices connected")
            self.logger.info("  - Drivers not installed (Windows: FTDI, CP210x, CH340)")
            self.logger.info("  - Insufficient permissions (Linux: add user to 'dialout' group)")
            print(f"\n{separator}")
            return

        self.logger.success(f"\nFound {len(ports)} serial port(s):\n")

        # Display each port
        for idx, port_info in enumerate(ports, 1):
            print(f"\n{separator}")
            self.logger.success(f"[{idx}] {port_info.device}")
            print(f"{separator}")

            # Basic port information
            print(f"  Description:     {self.format_size(port_info.description)}")
            print(f"  Hardware ID:     {self.format_size(port_info.hwid)}")
            print(f"  Manufacturer:    {self.format_size(port_info.manufacturer)}")
            print(f"  Product:         {self.format_size(port_info.product)}")
            print(f"  Serial Number:   {self.format_size(port_info.serial_number)}")
            print(f"  Location:        {self.format_size(port_info.location)}")

            # USB-specific info
            if port_info.vid is not None:
                print(f"  Vendor ID:       0x{port_info.vid:04X}")
            if port_info.pid is not None:
                print(f"  Product ID:      0x{port_info.pid:04X}")

            # Platform-specific path information
            if platform.system() == "Linux":
                print(f"\n  Usage (Linux):   set connection {port_info.device}")
                print(f"                   set baud {baud}")
            elif platform.system() == "Windows":
                print(f"\n  Usage (Windows): set connection {port_info.device}")
                print(f"                   set baud {baud}")
            else:
                print(f"\n  Usage:           set connection {port_info.device}")
                print(f"                   set baud {baud}")

            # Test for MAVLink if requested
            if test_mavlink:
                print(f"\n  Testing for MAVLink...")
                mavlink_info = self.test_mavlink_port(port_info.device, baud, timeout)

                if mavlink_info.get('has_mavlink'):
                    self.logger.success(f"  ✓ MAVLink Device Detected!")
                    print(f"    Vehicle Type:    {mavlink_info['vehicle_type']}")
                    print(f"    Autopilot:       {mavlink_info['autopilot']}")
                    print(f"    System ID:       {mavlink_info['system_id']}")
                    print(f"    Component ID:    {mavlink_info['component_id']}")
                    print(f"    Base Mode:       {mavlink_info['base_mode']}")
                    print(f"    System Status:   {mavlink_info['system_status']}")
                else:
                    self.logger.warning(f"  ✗ No MAVLink detected")
                    if 'error' in mavlink_info and mavlink_info['error']:
                        # Only show error if it's not a timeout
                        if 'timeout' not in mavlink_info['error'].lower():
                            print(f"    Error: {mavlink_info['error']}")

        # Summary
        print(f"\n{separator}")
        self.logger.success(f"Detection Complete: {len(ports)} port(s) found")

        if test_mavlink:
            mavlink_ports = [p for p in ports if test_mavlink]
            # We'd need to store results to count properly, but for now just inform
            self.logger.info("Use the scanner module for comprehensive MAVLink detection")
        else:
            self.logger.info("Tip: Set 'test_mavlink true' to automatically detect MAVLink devices")

        print(f"{separator}\n")

        # Platform-specific tips
        if platform.system() == "Linux":
            self.logger.info("Linux Tips:")
            print("  • List USB devices: lsusb")
            print("  • Monitor connections: dmesg | grep tty")
            print("  • Fix permissions: sudo usermod -a -G dialout $USER")
            print("  • Common paths: /dev/ttyUSB*, /dev/ttyACM*")
        elif platform.system() == "Windows":
            self.logger.info("Windows Tips:")
            print("  • View ports: Device Manager > Ports (COM & LPT)")
            print("  • List ports: mode")
            print("  • Install drivers: FTDI, CP210x, or CH340 drivers")
            print("  • Common paths: COM1, COM3, COM4, etc.")

        print()
