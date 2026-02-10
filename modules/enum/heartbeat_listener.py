from sploitkit import Module, Config, Option
from pymavlink import mavutil
import time
import signal
import sys
from datetime import datetime
from collections import defaultdict

class HeartbeatListener(Module):
    """
    Heartbeat Listener - Passively listens for MAVLink heartbeats and displays device information in real-time.

    Connection:
        Supports both network and serial connections:
        - Network (Listen): udpin:0.0.0.0:14550, tcpin:0.0.0.0:5760
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set interface udpin:0.0.0.0:14550
        run

        OR

        set interface /dev/ttyUSB0
        set baud 57600
        run
    """

    # Configuration for the listener module
    config = Config({
        Option(
            name='interface',
            description='Interface to listen on (udpin:0.0.0.0:14550 for network, /dev/ttyUSB0 for serial)',
            required=True,
        ): "udpin:0.0.0.0:14550",  # Default value - listen on all interfaces
        Option(
            name='baud',
            description='Baud rate for serial connections (default: 57600)',
            required=False,
        ): "57600",  # Default value
        Option(
            name='timeout',
            description='Duration to listen in seconds (0 for infinite)',
            required=False,
        ): "0",  # Default value - listen indefinitely
        Option(
            name='show_duplicates',
            description='Show duplicate heartbeats from same system (true/false)',
            required=False,
        ): "false",  # Default value
    })

    # MAVLink enumerations for human-readable output
    MAV_TYPE = {
        0: "GENERIC",
        1: "FIXED_WING",
        2: "QUADROTOR",
        3: "COAXIAL",
        4: "HELICOPTER",
        5: "ANTENNA_TRACKER",
        6: "GCS",
        7: "AIRSHIP",
        8: "FREE_BALLOON",
        9: "ROCKET",
        10: "GROUND_ROVER",
        11: "SURFACE_BOAT",
        12: "SUBMARINE",
        13: "HEXAROTOR",
        14: "OCTOROTOR",
        15: "TRICOPTER",
        16: "FLAPPING_WING",
        17: "KITE",
        18: "ONBOARD_CONTROLLER",
        19: "VTOL_DUOROTOR",
        20: "VTOL_QUADROTOR",
        21: "VTOL_TILTROTOR",
        22: "VTOL_RESERVED2",
        23: "VTOL_RESERVED3",
        24: "VTOL_RESERVED4",
        25: "VTOL_RESERVED5",
        26: "GIMBAL",
        27: "ADSB",
        28: "PARAFOIL",
        29: "DODECAROTOR",
        30: "CAMERA",
        31: "CHARGING_STATION",
        32: "FLARM",
        33: "SERVO",
        34: "ODID",
        35: "DECAROTOR",
        36: "BATTERY",
        37: "PARACHUTE",
        38: "LOG",
        39: "OSD",
        40: "IMU",
        41: "GPS",
        42: "WINCH",
    }

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

    MAV_STATE = {
        0: "UNINIT",
        1: "BOOT",
        2: "CALIBRATING",
        3: "STANDBY",
        4: "ACTIVE",
        5: "CRITICAL",
        6: "EMERGENCY",
        7: "POWEROFF",
        8: "FLIGHT_TERMINATION",
    }

    MAV_MODE_FLAG = {
        128: "SAFETY_ARMED",
        64: "MANUAL_INPUT_ENABLED",
        32: "HIL_ENABLED",
        16: "STABILIZE_ENABLED",
        8: "GUIDED_ENABLED",
        4: "AUTO_ENABLED",
        2: "TEST_ENABLED",
        1: "CUSTOM_MODE_ENABLED",
    }

    def decode_base_mode(self, base_mode):
        """Decode base_mode bitfield into human-readable flags."""
        flags = []
        for bit, name in sorted(self.MAV_MODE_FLAG.items(), reverse=True):
            if base_mode & bit:
                flags.append(name)
        return flags if flags else ["DISARMED"]

    def format_heartbeat(self, heartbeat, source_system, source_component, first_seen=False):
        """Format heartbeat data in a nice, readable way."""
        separator = "=" * 80

        # Get human-readable values
        mav_type = self.MAV_TYPE.get(heartbeat.type, f"UNKNOWN({heartbeat.type})")
        autopilot = self.MAV_AUTOPILOT.get(heartbeat.autopilot, f"UNKNOWN({heartbeat.autopilot})")
        system_status = self.MAV_STATE.get(heartbeat.system_status, f"UNKNOWN({heartbeat.system_status})")
        mode_flags = self.decode_base_mode(heartbeat.base_mode)

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        if first_seen:
            self.logger.success(f"\n{separator}")
            self.logger.success(f"NEW DEVICE DETECTED!")
        else:
            print(f"\n{separator}")

        print(f"Timestamp:          {timestamp}")
        print(f"System ID:          {source_system}")
        print(f"Component ID:       {source_component}")
        print(f"Vehicle Type:       {mav_type}")
        print(f"Autopilot:          {autopilot}")
        print(f"System Status:      {system_status}")
        print(f"Base Mode:          {heartbeat.base_mode} (0x{heartbeat.base_mode:02X})")
        print(f"Mode Flags:         {', '.join(mode_flags)}")
        print(f"Custom Mode:        {heartbeat.custom_mode}")
        print(f"MAVLink Version:    {heartbeat.mavlink_version}")
        print(f"{separator}")

    def run(self):
        # Retrieve configuration options
        interface = self.config['interface']
        timeout = int(self.config['timeout'])
        show_duplicates = self.config['show_duplicates'].lower() == "true"
        baud = int(self.config['baud'])

        # Build connection string
        # Check if it's a serial device
        if interface.startswith('/dev/') or interface.upper().startswith('COM'):
            # Serial connection - add baud rate
            connection_string = f"{interface},{baud}"
            self.logger.info(f"Starting heartbeat listener on serial: {interface} @ {baud} baud")
        elif interface.startswith(('udp:', 'tcp:', 'udpin:', 'tcpin:', 'udpout:', 'tcpout:')):
            # Network connection with protocol prefix - use as-is
            connection_string = interface
            self.logger.info(f"Starting heartbeat listener on {connection_string}")
        else:
            # Legacy format without protocol prefix - shouldn't happen with new config but handle it
            self.logger.warning(f"Unrecognized interface format: {interface}")
            connection_string = interface
            self.logger.info(f"Starting heartbeat listener on {connection_string}")
        self.logger.info(f"Listening for MAVLink heartbeats... (Press Ctrl+C to stop)")

        if timeout > 0:
            self.logger.info(f"Will listen for {timeout} seconds")
        else:
            self.logger.info("Listening indefinitely")

        # Track seen systems to avoid duplicates
        seen_systems = defaultdict(dict)

        # Flag to handle graceful shutdown
        stop_listening = False

        def signal_handler(sig, frame):
            nonlocal stop_listening
            stop_listening = True

        # Register signal handler
        original_sigint = signal.signal(signal.SIGINT, signal_handler)

        master = None
        try:
            # Connect to MAVLink
            master = mavutil.mavlink_connection(connection_string)

            start_time = time.time()
            heartbeat_count = 0

            while not stop_listening:
                # Check timeout
                if timeout > 0 and (time.time() - start_time) > timeout:
                    self.logger.info(f"\nTimeout reached. Stopping listener.")
                    break

                # Listen for heartbeat messages with short timeout for responsiveness
                msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)

                if msg:
                    heartbeat_count += 1
                    source_system = msg.get_srcSystem()
                    source_component = msg.get_srcComponent()
                    system_key = (source_system, source_component)

                    # Check if this is a new system or if we should show duplicates
                    is_new_system = system_key not in seen_systems

                    if is_new_system or show_duplicates:
                        self.format_heartbeat(msg, source_system, source_component, first_seen=is_new_system)

                        # Store system info
                        seen_systems[system_key] = {
                            'type': msg.type,
                            'autopilot': msg.autopilot,
                            'base_mode': msg.base_mode,
                            'custom_mode': msg.custom_mode,
                            'system_status': msg.system_status,
                            'mavlink_version': msg.mavlink_version,
                            'count': seen_systems[system_key].get('count', 0) + 1,
                            'last_seen': datetime.now()
                        }
                    else:
                        # Just update the counter silently
                        seen_systems[system_key]['count'] += 1
                        seen_systems[system_key]['last_seen'] = datetime.now()

                        # Print a simple status update
                        if seen_systems[system_key]['count'] % 10 == 0:
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] System {source_system}: {seen_systems[system_key]['count']} heartbeats received")

        except KeyboardInterrupt:
            self.logger.info("\n\nListener stopped by user.")
        except Exception as e:
            self.logger.error(f"Error during listening: {str(e)}")
        finally:
            # Restore original signal handler
            signal.signal(signal.SIGINT, original_sigint)

            # Close connection
            if master:
                master.close()

            if stop_listening:
                self.logger.info("\n\nListener stopped by user (Ctrl+C).")

            # Print summary
            self.logger.success(f"\n{'=' * 80}")
            self.logger.success(f"LISTENING SESSION SUMMARY")
            self.logger.success(f"{'=' * 80}")
            self.logger.success(f"Total heartbeats received: {heartbeat_count}")
            self.logger.success(f"Unique systems detected:   {len(seen_systems)}")

            if seen_systems:
                self.logger.success(f"\nDetected Systems:")
                for (sys_id, comp_id), info in seen_systems.items():
                    mav_type = self.MAV_TYPE.get(info['type'], f"UNKNOWN({info['type']})")
                    autopilot = self.MAV_AUTOPILOT.get(info['autopilot'], f"UNKNOWN({info['autopilot']})")
                    print(f"  - System {sys_id}, Component {comp_id}: {mav_type} ({autopilot}) - {info['count']} heartbeats")

            self.logger.success(f"{'=' * 80}")
