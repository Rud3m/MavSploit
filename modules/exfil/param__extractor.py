from sploitkit import Module, Config, Option
from pymavlink import mavutil
import json
import os

class MavlinkParameterExtractor(Module):
    """Mavlink Parameter Extractor - Extracts and saves all parameters from a Mavlink-enabled device."""

    # Configuration for the module
    config = Config({
        Option(
            name='target_ip',
            description='IP address of the Mavlink device',
            required=True,
        ): "192.168.1.1",  # Default value
        Option(
            name='target_port',
            description='Port number of the Mavlink device',
            required=True,
        ): "14550",  # Default value
        Option(
            name='protocol',
            description='Connection protocol (udp/tcp)',
            required=True,
        ): "udp",  # Default value
        Option(
            name='save_path',
            description='Path to save the extracted parameters (as JSON)',
            required=False,
        ): "./mavlink_parameters.json",  # Default value
    })

    def connect_drone(self, target_ip, target_port, protocol):
        """
        Establish a connection to the drone.
        """
        connection_string = f'{protocol}:{target_ip}:{target_port}'
        master = mavutil.mavlink_connection(connection_string)
        master.wait_heartbeat()
        self.logger.info("Connected to the drone.")
        return master

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
        target_ip = self.config['target_ip']
        target_port = self.config['target_port']
        protocol = self.config['protocol']
        save_path = self.config['save_path']

        # Ensure the directory for the save path exists
        os.makedirs(os.path.dirname(save_path), exist_ok=True)

        # Connect to the Mavlink device
        master = self.connect_drone(target_ip, target_port, protocol)

        # Extract parameters from the device
        parameters = self.extract_parameters(master)

        # Save the parameters to a file
        self.save_parameters_to_file(parameters, save_path)

