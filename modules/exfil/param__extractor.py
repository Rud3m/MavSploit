from sploitkit import Config, Option
from lib.base import MAVLinkModule
from pymavlink import mavutil
import json
import os

class MavlinkParameterExtractor(MAVLinkModule):
    """
    Mavlink Parameter Extractor - Extracts and saves all parameters from a Mavlink-enabled device.

    Connection:
        Supports both network and serial connections:
        - Network: udp:192.168.1.1:14550 or tcp:192.168.1.1:5760
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set connection udp:192.168.1.1:14550
        set save_path ./drone_params.json
        run
    """

    # Inherit connection config from base class and add module-specific options
    config = MAVLinkModule.config + Config({
        Option(
            name='save_path',
            description='Path to save the extracted parameters (as JSON)',
            required=False,
        ): "./mavlink_parameters.json",  # Default value
    })

    def extract_parameters(self, master):
        """
        Extracts and returns all parameters from the connected Mavlink device.
        Exits early if STAT_RUNTIME is encountered.
        """
        self.logger.info("Requesting parameters from the device...")
        master.mav.param_request_list_send(master.target_system, master.target_component)
        
        parameters = {}
        while True:
            message = master.recv_match(type='PARAM_VALUE', blocking=True)
            if message is None:
                break
            param_id = message.param_id
            param_value = message.param_value
            parameters[param_id] = param_value
            self.logger.info(f"Parameter: {param_id}, Value: {param_value}")

            # Check if STAT_RUNTIME is encountered
            if param_id == 'STAT_RUNTIME':
                self.logger.info(f"STAT_RUNTIME encountered with value: {param_value}. Exiting...")
                break

        return parameters

    def save_parameters_to_file(self, parameters, save_path):
        """
        Saves the extracted parameters to a JSON file.
        """
        try:
            with open(save_path, 'w') as f:
                json.dump(parameters, f, indent=4)
            self.logger.success(f"Parameters saved to {save_path}")
        except IOError as e:
            self.logger.error(f"Failed to save parameters to file: {e}")

    def run(self):
        # Retrieve configuration options
        save_path = self.config['save_path']

        # Ensure the directory for the save path exists
        save_dir = os.path.dirname(save_path)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)

        # Connect to the Mavlink device (supports both serial and network)
        master = self.connect_drone()

        try:
            # Extract parameters from the device
            parameters = self.extract_parameters(master)

            # Save the parameters to a file
            self.save_parameters_to_file(parameters, save_path)
        finally:
            # Clean up connection
            self.close_connection(master)

