from sploitkit import Module, Config, Option
from pymavlink import mavutil
import os
import subprocess
import time
from datetime import datetime

class WiresharkPluginGenerator(Module):
    """
    Wireshark Plugin Generator - Detects autopilot type from heartbeats and generates appropriate Wireshark Lua dissector.

    Connection:
        Supports both network and serial connections:
        - Network (Listen): udpin:0.0.0.0:14550, tcpin:0.0.0.0:5760
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set interface udpin:0.0.0.0:14550
        set autopilot_type auto
        run

        OR

        set interface /dev/ttyUSB0
        set baud 57600
        run
    """

    # Configuration for the module
    config = Config({
        Option(
            name='interface',
            description='Interface to listen on (udpin:0.0.0.0:14550 for network, /dev/ttyUSB0 for serial)',
            required=False,
        ): "udpin:0.0.0.0:14550",  # Default value
        Option(
            name='baud',
            description='Baud rate for serial connections (default: 57600)',
            required=False,
        ): "57600",  # Default value
        Option(
            name='listen_timeout',
            description='Seconds to listen for heartbeats (0 for manual detection)',
            required=False,
        ): "10",  # Default value
        Option(
            name='autopilot_type',
            description='Autopilot type (auto/ardupilot/px4/common/generic) - auto detects from heartbeat',
            required=False,
        ): "auto",  # Default value
        Option(
            name='output_dir',
            description='Directory to save the Wireshark plugin',
            required=True,
        ): "./wireshark_plugins",  # Default value
        Option(
            name='mavlink_version',
            description='MAVLink protocol version (1.0 or 2.0)',
            required=False,
        ): "2.0",  # Default value
    })

    # MAVLink autopilot type mapping
    MAV_AUTOPILOT = {
        0: "GENERIC",
        1: "RESERVED",
        2: "SLUGS",
        3: "ARDUPILOTMEGA",
        4: "OPENPILOT",
        5: "GENERIC_WAYPOINTS_ONLY",
        6: "GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY",
        7: "GENERIC_MISSION_FULL",
        8: "INVALID",
        9: "PPZ",
        10: "UDB",
        11: "FP",
        12: "PX4",
        13: "SMACCMPILOT",
        14: "AUTOQUAD",
        15: "ARMAZILA",
        16: "AEROB",
        17: "ASLUAV",
        18: "SMARTAP",
        19: "AIRRAILS",
        20: "REFLEX",
    }

    # Map autopilot types to MAVLink XML definition files
    AUTOPILOT_TO_DIALECT = {
        "ARDUPILOTMEGA": "ardupilotmega",
        "PX4": "common",  # PX4 uses common dialect
        "GENERIC": "common",
        "GENERIC_WAYPOINTS_ONLY": "common",
        "GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY": "common",
        "GENERIC_MISSION_FULL": "common",
        "OPENPILOT": "common",
        "AUTOQUAD": "autoquad",
        "SLUGS": "slugs",
        "UDB": "ualberta",
        "ASLUAV": "asluav",
    }

    def detect_autopilot(self, interface, timeout, baud=57600):
        """
        Listen for MAVLink heartbeats and detect the autopilot type.
        Returns the detected autopilot type string.
        """
        # Build connection string
        if interface.startswith('/dev/') or interface.upper().startswith('COM'):
            # Serial connection - add baud rate
            connection_string = f"{interface},{baud}"
            self.logger.info(f"Listening on serial: {interface} @ {baud} baud for heartbeats...")
        elif interface.startswith(('udp:', 'tcp:', 'udpin:', 'tcpin:', 'udpout:', 'tcpout:')):
            # Network connection with protocol prefix - use as-is
            connection_string = interface
            self.logger.info(f"Listening on {connection_string} for heartbeats...")
        else:
            # Unrecognized format
            self.logger.warning(f"Unrecognized interface format: {interface}")
            connection_string = interface
            self.logger.info(f"Listening on {connection_string} for heartbeats...")

        self.logger.info(f"Waiting up to {timeout} seconds to detect autopilot...")

        try:
            master = mavutil.mavlink_connection(connection_string)
            start_time = time.time()

            while True:
                # Check timeout
                if timeout > 0 and (time.time() - start_time) > timeout:
                    self.logger.warning("Timeout reached without detecting heartbeat.")
                    return None

                # Listen for heartbeat
                msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)

                if msg:
                    autopilot_id = msg.autopilot
                    autopilot_name = self.MAV_AUTOPILOT.get(autopilot_id, f"UNKNOWN({autopilot_id})")

                    self.logger.success(f"Detected autopilot: {autopilot_name}")
                    self.logger.info(f"System ID: {msg.get_srcSystem()}")
                    self.logger.info(f"Vehicle Type: {msg.type}")

                    master.close()
                    return autopilot_name

        except Exception as e:
            self.logger.error(f"Error detecting autopilot: {str(e)}")
            return None

    def get_mavlink_definitions_path(self):
        """
        Find the path to MAVLink message definitions.
        """
        try:
            # First, check the project root directory
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            definitions_path = os.path.join(project_root, 'message_definitions', 'v1.0')

            if os.path.exists(definitions_path):
                self.logger.info(f"Using message definitions from: {definitions_path}")
                return definitions_path

            # Fallback: Check pymavlink installation directory
            import pymavlink
            pymavlink_path = os.path.dirname(pymavlink.__file__)
            definitions_path = os.path.join(pymavlink_path, 'message_definitions', 'v1.0')

            if os.path.exists(definitions_path):
                self.logger.info(f"Using message definitions from: {definitions_path}")
                return definitions_path

            # Alternative pymavlink location
            definitions_path = os.path.join(pymavlink_path, '..', 'message_definitions', 'v1.0')
            if os.path.exists(definitions_path):
                return os.path.abspath(definitions_path)

            self.logger.error("Could not find MAVLink message definitions directory")
            self.logger.error(f"Checked locations:")
            self.logger.error(f"  - {os.path.join(project_root, 'message_definitions', 'v1.0')}")
            self.logger.error(f"  - {os.path.join(pymavlink_path, 'message_definitions', 'v1.0')}")
            return None

        except Exception as e:
            self.logger.error(f"Error locating MAVLink definitions: {str(e)}")
            return None

    def generate_plugin(self, dialect, output_dir, mavlink_version):
        """
        Generate the Wireshark Lua plugin using mavgen.
        """
        definitions_path = self.get_mavlink_definitions_path()
        if not definitions_path:
            return False

        # Construct path to XML definition file
        xml_file = os.path.join(definitions_path, f"{dialect}.xml")

        if not os.path.exists(xml_file):
            self.logger.error(f"Definition file not found: {xml_file}")
            self.logger.info(f"Available definitions in {definitions_path}:")
            try:
                for f in os.listdir(definitions_path):
                    if f.endswith('.xml'):
                        self.logger.info(f"  - {f}")
            except:
                pass
            return False

        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)

        # Construct output filename
        output_file = os.path.join(output_dir, f"mavlink_{mavlink_version.replace('.', '_')}_{dialect}.lua")

        self.logger.info(f"Generating Wireshark plugin...")
        self.logger.info(f"Dialect: {dialect}")
        self.logger.info(f"MAVLink Version: {mavlink_version}")
        self.logger.info(f"Output: {output_file}")

        # Run mavgen to generate the plugin
        try:
            cmd = [
                'python3', 'tools/mavgen.py',
                '--lang=WLua',
                f'--wire-protocol={mavlink_version}',
                f'--output={output_file}',
                xml_file
            ]

            self.logger.info(f"Running: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

            if result.returncode == 0:
                self.logger.success(f"Plugin generated successfully!")
                self.logger.success(f"File: {output_file}")
                return output_file
            else:
                self.logger.error(f"Failed to generate plugin:")
                self.logger.error(f"Exit code: {result.returncode}")
                if result.stdout:
                    self.logger.error(f"stdout: {result.stdout}")
                if result.stderr:
                    self.logger.error(f"stderr: {result.stderr}")
                return False

        except subprocess.TimeoutExpired:
            self.logger.error("Plugin generation timed out after 30 seconds")
            return False
        except Exception as e:
            self.logger.error(f"Error running mavgen: {str(e)}")
            return False

    def print_installation_instructions(self, plugin_file):
        """
        Print instructions for installing the generated plugin.
        """
        separator = "=" * 80
        self.logger.success(f"\n{separator}")
        self.logger.success("WIRESHARK PLUGIN INSTALLATION INSTRUCTIONS")
        self.logger.success(f"{separator}\n")

        print("The Wireshark plugin has been generated. To install it:\n")

        print("1. Copy the plugin to Wireshark's plugin directory:")
        print("   Linux:   ~/.local/lib/wireshark/plugins/")
        print("            (or ~/.wireshark/plugins/ for older versions)")
        print("   Windows: C:\\Program Files\\Wireshark\\plugins\\\n")

        print(f"   Command (Linux):")
        plugin_filename = os.path.basename(plugin_file)
        print(f"   mkdir -p ~/.local/lib/wireshark/plugins/")
        print(f"   cp {plugin_file} ~/.local/lib/wireshark/plugins/\n")

        print("2. (Optional) Edit port bindings in the plugin:")
        print(f"   Default ports: 14550, 14580, 18570")
        print(f"   Edit the last lines of {plugin_filename} to add/remove ports\n")

        print("3. Restart Wireshark\n")

        print("4. Verify installation:")
        print("   Help > About Wireshark > Plugins")
        print(f"   Look for: {plugin_filename}\n")

        print("5. Capture MAVLink traffic:")
        print("   Start capturing on the appropriate interface")
        print("   MAVLink messages will be automatically dissected\n")

        self.logger.success(f"{separator}")

    def run(self):
        # Retrieve configuration options
        interface = self.config['interface']
        baud = int(self.config['baud'])
        listen_timeout = int(self.config['listen_timeout'])
        autopilot_type = self.config['autopilot_type'].strip().upper()
        output_dir = self.config['output_dir']
        mavlink_version = self.config['mavlink_version']

        self.logger.info("=" * 80)
        self.logger.info("MAVLink Wireshark Plugin Generator")
        self.logger.info("=" * 80)

        # Determine the dialect to use
        dialect = None

        if autopilot_type == "AUTO":
            # Auto-detect from heartbeat
            detected_autopilot = self.detect_autopilot(interface, listen_timeout, baud)

            if not detected_autopilot:
                self.logger.warning("Failed to auto-detect autopilot. Using 'common' dialect as fallback.")
                dialect = "common"
            else:
                # Map detected autopilot to dialect
                dialect = self.AUTOPILOT_TO_DIALECT.get(detected_autopilot, "common")
                self.logger.info(f"Using dialect: {dialect} (for {detected_autopilot})")
        else:
            # Use manually specified autopilot type
            if autopilot_type in ["ARDUPILOT", "ARDUPILOTMEGA"]:
                dialect = "ardupilotmega"
            elif autopilot_type == "PX4":
                dialect = "common"
            elif autopilot_type == "COMMON":
                dialect = "common"
            elif autopilot_type == "GENERIC":
                dialect = "common"
            else:
                dialect = autopilot_type.lower()

            self.logger.info(f"Using manually specified dialect: {dialect}")

        # Generate the plugin
        plugin_file = self.generate_plugin(dialect, output_dir, mavlink_version)

        if plugin_file:
            # Print installation instructions
            self.print_installation_instructions(plugin_file)
        else:
            self.logger.error("Failed to generate Wireshark plugin")
