from sploitkit import Config, Option
from lib.base import MAVLinkModule
from pymavlink import mavutil
import os
import time
from datetime import datetime

class LogDump(MAVLinkModule):
    """
    Log Dump - Downloads flight logs from MAVLink-enabled devices.

    Connection:
        Supports both network and serial connections:
        - Network: udp:192.168.1.1:14550 or tcp:192.168.1.1:5760
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set connection udp:192.168.1.1:14550
        set log_id 0
        run
    """

    # Inherit connection config from base class and add module-specific options
    config = MAVLinkModule.config + Config({
        Option(
            name='log_id',
            description='Specific log ID to download (0 for all, -1 to list only)',
            required=False,
        ): "0",  # Default value - download all logs
        Option(
            name='output_dir',
            description='Directory to save downloaded logs',
            required=False,
        ): "./mavlink_logs",  # Default value
        Option(
            name='timeout',
            description='Timeout in seconds for log operations',
            required=False,
        ): "30",  # Default value
    })

    def request_log_list(self, master, timeout):
        """
        Request the list of available logs from the device.
        Returns a list of log entries.
        """
        self.logger.info("Requesting log list from device...")

        # Request list of logs
        master.mav.log_request_list_send(
            master.target_system,
            master.target_component,
            0,  # start
            0xFFFF  # end (request all)
        )

        logs = []
        start_time = time.time()
        last_log_id = -1

        while True:
            if time.time() - start_time > timeout:
                self.logger.warning(f"Timeout waiting for log list (received {len(logs)} entries)")
                break

            msg = master.recv_match(type='LOG_ENTRY', blocking=True, timeout=1)

            if msg:
                # Reset timeout on each message
                start_time = time.time()

                log_entry = {
                    'id': msg.id,
                    'num_logs': msg.num_logs,
                    'last_log_num': msg.last_log_num,
                    'time_utc': msg.time_utc,
                    'size': msg.size
                }

                logs.append(log_entry)
                last_log_id = msg.id

                # Check if we've received all logs
                if msg.id == msg.last_log_num:
                    self.logger.success(f"Received complete log list: {len(logs)} logs")
                    break

        return logs

    def download_log(self, master, log_id, log_size, output_dir, timeout):
        """
        Download a specific log from the device.
        """
        self.logger.info(f"Downloading log ID {log_id} (size: {log_size} bytes)...")

        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)

        # Generate filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(output_dir, f"log_{log_id}_{timestamp}.bin")

        # Request log data
        offset = 0
        chunk_size = 90  # MAVLink typical data chunk size
        log_data = bytearray()

        start_time = time.time()
        last_progress = 0

        while offset < log_size:
            if time.time() - start_time > timeout:
                self.logger.error(f"Timeout downloading log {log_id} at offset {offset}/{log_size}")
                return None

            # Request chunk of log data
            count = min(chunk_size, log_size - offset)

            master.mav.log_request_data_send(
                master.target_system,
                master.target_component,
                log_id,
                offset,
                count
            )

            # Wait for LOG_DATA response
            msg = master.recv_match(type='LOG_DATA', blocking=True, timeout=2)

            if msg and msg.id == log_id:
                # Reset timeout on successful receive
                start_time = time.time()

                # Append data
                data_count = min(msg.count, len(msg.data))
                log_data.extend(msg.data[:data_count])
                offset += data_count

                # Progress indicator
                progress = int((offset / log_size) * 100)
                if progress >= last_progress + 10:
                    self.logger.info(f"Progress: {progress}% ({offset}/{log_size} bytes)")
                    last_progress = progress
            else:
                # Retry if no response or wrong log ID
                time.sleep(0.1)

        # Save log to file
        try:
            with open(filename, 'wb') as f:
                f.write(log_data)

            self.logger.success(f"Log {log_id} downloaded: {filename}")
            self.logger.success(f"Size: {len(log_data)} bytes")
            return filename

        except IOError as e:
            self.logger.error(f"Failed to save log file: {e}")
            return None

    def format_log_list(self, logs):
        """
        Format and display the list of available logs.
        """
        separator = "=" * 80
        self.logger.success(f"\n{separator}")
        self.logger.success(f"AVAILABLE LOGS")
        self.logger.success(f"{separator}\n")

        if not logs:
            self.logger.warning("No logs found on device")
            return

        print(f"{'ID':<6} {'Size (bytes)':<15} {'Date/Time (UTC)':<25}")
        print("-" * 80)

        for log in logs:
            log_id = log['id']
            size = log['size']

            # Convert time_utc to readable format if available
            if log['time_utc'] > 0:
                time_str = datetime.utcfromtimestamp(log['time_utc']).strftime('%Y-%m-%d %H:%M:%S')
            else:
                time_str = "Unknown"

            print(f"{log_id:<6} {size:<15} {time_str:<25}")

        print(f"\n{separator}")
        self.logger.info(f"Total logs available: {len(logs)}")
        self.logger.info(f"Total size: {sum(log['size'] for log in logs)} bytes")
        print(f"{separator}\n")

    def run(self):
        # Retrieve configuration options
        log_id = int(self.config['log_id'])
        output_dir = self.config['output_dir']
        timeout = int(self.config['timeout'])

        separator = "=" * 80
        self.logger.info(separator)
        self.logger.info("MAVLink Log Dump")
        self.logger.info(separator)

        try:
            # Connect to the MAVLink device (supports both serial and network)
            master = self.connect_drone()

            # Request log list
            logs = self.request_log_list(master, timeout)

            if not logs:
                self.logger.error("No logs found or failed to retrieve log list")
                return

            # Display log list
            self.format_log_list(logs)

            # Determine which logs to download
            if log_id == -1:
                # List only mode
                self.logger.info("List-only mode (log_id=-1). Set log_id to 0 (all) or specific ID to download.")
                return
            elif log_id == 0:
                # Download all logs
                self.logger.info(f"Downloading all {len(logs)} logs...")
                logs_to_download = logs
            else:
                # Download specific log
                logs_to_download = [log for log in logs if log['id'] == log_id]

                if not logs_to_download:
                    self.logger.error(f"Log ID {log_id} not found")
                    self.logger.info("Available log IDs:")
                    for log in logs:
                        self.logger.info(f"  - ID: {log['id']}, Size: {log['size']} bytes")
                    return

            # Download logs
            downloaded_files = []

            for i, log in enumerate(logs_to_download, 1):
                self.logger.info(f"\n[{i}/{len(logs_to_download)}] Processing log {log['id']}...")

                filename = self.download_log(
                    master,
                    log['id'],
                    log['size'],
                    output_dir,
                    timeout
                )

                if filename:
                    downloaded_files.append(filename)
                else:
                    self.logger.warning(f"Failed to download log {log['id']}")

            # Summary
            self.logger.success(f"\n{separator}")
            self.logger.success("DOWNLOAD SUMMARY")
            self.logger.success(f"{separator}")
            self.logger.success(f"Total logs downloaded: {len(downloaded_files)}/{len(logs_to_download)}")

            if downloaded_files:
                self.logger.success(f"\nDownloaded files:")
                for f in downloaded_files:
                    self.logger.success(f"  - {f}")

            self.logger.success(f"{separator}")

        except Exception as e:
            self.logger.error(f"Error during log dump: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
        finally:
            # Clean up connection
            self.close_connection(master)
