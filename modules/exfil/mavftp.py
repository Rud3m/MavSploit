from sploitkit import Config, Option
from lib.base import MAVLinkModule
from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP, MAVFTPReturn
import os
import sys

class MAVFTPClient(MAVLinkModule):
    """
    MAVFtp Client - Full-featured FTP client for MAVLink devices.

    Supports file upload, download, list, delete, and more operations over MAVLink.

    Connection:
        Supports both network and serial connections:
        - Network: udp:192.168.1.1:14550 or tcp:192.168.1.1:5760
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set connection udp:192.168.1.1:14550
        set mode interactive
        run
    """

    # Inherit connection config from base class and add module-specific options
    config = MAVLinkModule.config + Config({
        Option(
            name='mode',
            description='Mode: interactive or single (single for one operation)',
            required=False,
        ): "interactive",  # Default value
        Option(
            name='operation',
            description='FTP operation: list, get, put, remove, mkdir, rmdir, stat, crc (for single mode)',
            required=False,
        ): "list",  # Default value
        Option(
            name='remote_path',
            description='Remote path on the device (file or directory)',
            required=False,
        ): "/",  # Default value
        Option(
            name='local_path',
            description='Local path for upload/download operations',
            required=False,
        ): "./",  # Default value
        Option(
            name='target_system',
            description='Target system ID (0 for auto-detect)',
            required=False,
        ): "0",  # Default value
        Option(
            name='target_component',
            description='Target component ID (0 for auto-detect)',
            required=False,
        ): "0",  # Default value
    })

    def list_directory(self, ftp, remote_path):
        """
        List files and directories in the remote path.
        """
        self.logger.info(f"Listing directory: {remote_path}")

        try:
            ret = ftp.cmd_list([remote_path])

            if ret.error_code == 0:
                entries = ret.entries

                if not entries:
                    self.logger.warning(f"Directory is empty or does not exist: {remote_path}")
                    return

                separator = "=" * 100
                self.logger.success(f"\n{separator}")
                self.logger.success(f"DIRECTORY LISTING: {remote_path}")
                self.logger.success(f"{separator}\n")

                print(f"{'Type':<6} {'Name':<40} {'Size':<15} {'Modified':<20}")
                print("-" * 100)

                for entry in entries:
                    entry_type = "DIR" if entry.size == 0 and entry.name.endswith('/') else "FILE"
                    name = entry.name
                    size = entry.size if entry_type == "FILE" else "-"

                    print(f"{entry_type:<6} {name:<40} {str(size):<15} {'-':<20}")

                print(f"\n{separator}")
                self.logger.success(f"Total entries: {len(entries)}")
                print(f"{separator}\n")

                return entries

            else:
                self.logger.error(f"Failed to list directory: {ret.display_message()}")
                return None

        except Exception as e:
            self.logger.error(f"Error listing directory: {str(e)}")
            return None

    def download_file(self, ftp, remote_path, local_path):
        """
        Download a file from the remote device.
        """
        self.logger.info(f"Downloading: {remote_path} -> {local_path}")

        try:
            # If local_path is a directory, preserve filename
            if os.path.isdir(local_path):
                filename = os.path.basename(remote_path)
                local_path = os.path.join(local_path, filename)

            # Create directory if needed
            local_dir = os.path.dirname(local_path)
            if local_dir and not os.path.exists(local_dir):
                os.makedirs(local_dir, exist_ok=True)

            ret = ftp.cmd_get([remote_path, local_path])

            if ret.error_code == 0:
                file_size = os.path.getsize(local_path) if os.path.exists(local_path) else 0
                self.logger.success(f"Downloaded successfully: {local_path}")
                self.logger.success(f"Size: {file_size} bytes")
                return True
            else:
                self.logger.error(f"Failed to download file: {ret.display_message()}")
                return False

        except Exception as e:
            self.logger.error(f"Error downloading file: {str(e)}")
            return False

    def upload_file(self, ftp, local_path, remote_path):
        """
        Upload a file to the remote device.
        """
        self.logger.info(f"Uploading: {local_path} -> {remote_path}")

        try:
            if not os.path.exists(local_path):
                self.logger.error(f"Local file does not exist: {local_path}")
                return False

            if not os.path.isfile(local_path):
                self.logger.error(f"Local path is not a file: {local_path}")
                return False

            file_size = os.path.getsize(local_path)
            self.logger.info(f"File size: {file_size} bytes")

            ret = ftp.cmd_put([local_path, remote_path])

            if ret.error_code == 0:
                self.logger.success(f"Uploaded successfully: {remote_path}")
                return True
            else:
                self.logger.error(f"Failed to upload file: {ret.display_message()}")
                return False

        except Exception as e:
            self.logger.error(f"Error uploading file: {str(e)}")
            return False

    def remove_file(self, ftp, remote_path):
        """
        Remove a file from the remote device.
        """
        self.logger.info(f"Removing file: {remote_path}")

        try:
            ret = ftp.cmd_remove([remote_path])

            if ret.error_code == 0:
                self.logger.success(f"File removed: {remote_path}")
                return True
            else:
                self.logger.error(f"Failed to remove file: {ret.display_message()}")
                return False

        except Exception as e:
            self.logger.error(f"Error removing file: {str(e)}")
            return False

    def create_directory(self, ftp, remote_path):
        """
        Create a directory on the remote device.
        """
        self.logger.info(f"Creating directory: {remote_path}")

        try:
            ret = ftp.cmd_mkdir([remote_path])

            if ret.error_code == 0:
                self.logger.success(f"Directory created: {remote_path}")
                return True
            else:
                self.logger.error(f"Failed to create directory: {ret.display_message()}")
                return False

        except Exception as e:
            self.logger.error(f"Error creating directory: {str(e)}")
            return False

    def remove_directory(self, ftp, remote_path):
        """
        Remove a directory from the remote device.
        """
        self.logger.info(f"Removing directory: {remote_path}")

        try:
            ret = ftp.cmd_rmdir([remote_path])

            if ret.error_code == 0:
                self.logger.success(f"Directory removed: {remote_path}")
                return True
            else:
                self.logger.error(f"Failed to remove directory: {ret.display_message()}")
                return False

        except Exception as e:
            self.logger.error(f"Error removing directory: {str(e)}")
            return False

    def get_file_info(self, ftp, remote_path):
        """
        Get information about a file on the remote device.
        """
        self.logger.info(f"Getting file info: {remote_path}")

        try:
            # Use list command to get file info
            ret = ftp.cmd_list([remote_path])

            if ret.error_code == 0 and ret.entries:
                entry = ret.entries[0]

                separator = "=" * 80
                self.logger.success(f"\n{separator}")
                self.logger.success(f"FILE INFORMATION: {remote_path}")
                self.logger.success(f"{separator}\n")

                print(f"Name:     {entry.name}")
                print(f"Size:     {entry.size} bytes")
                print(f"Type:     {'Directory' if entry.name.endswith('/') else 'File'}")

                print(f"\n{separator}\n")
                return entry

            else:
                self.logger.error(f"Failed to get file info: {ret.display_message()}")
                return None

        except Exception as e:
            self.logger.error(f"Error getting file info: {str(e)}")
            return None

    def calculate_crc(self, ftp, remote_path):
        """
        Calculate CRC32 checksum of a remote file.
        """
        self.logger.info(f"Calculating CRC for: {remote_path}")

        try:
            ret = ftp.cmd_crc([remote_path])

            if ret.error_code == 0:
                self.logger.success(f"CRC32: 0x{ret.crc:08X}")
                return ret.crc
            else:
                self.logger.error(f"Failed to calculate CRC: {ret.display_message()}")
                return None

        except Exception as e:
            self.logger.error(f"Error calculating CRC: {str(e)}")
            return None

    def print_help(self):
        """
        Print available commands in interactive mode.
        """
        print("\nAvailable Commands:")
        print("=" * 80)
        print("  ls [path]              - List directory contents")
        print("  cd <path>              - Change remote directory")
        print("  pwd                    - Print remote working directory")
        print("  lcd <path>             - Change local directory")
        print("  lpwd                   - Print local working directory")
        print("  get <remote> [local]   - Download file from device")
        print("  put <local> <remote>   - Upload file to device")
        print("  rm <remote>            - Delete remote file")
        print("  mkdir <remote>         - Create remote directory")
        print("  rmdir <remote>         - Remove remote directory")
        print("  stat <remote>          - Get file information")
        print("  crc <remote>           - Calculate file CRC32")
        print("  help                   - Show this help message")
        print("  exit / quit            - Exit interactive mode")
        print("=" * 80)

    def interactive_mode(self, ftp):
        """
        Run MAVFtp in interactive mode with a command prompt.
        """
        # Track current directories
        remote_cwd = "/"
        local_cwd = os.getcwd()

        separator = "=" * 80
        self.logger.success(f"\n{separator}")
        self.logger.success("MAVLink FTP Interactive Mode")
        self.logger.success(f"{separator}")
        self.logger.info("Type 'help' for available commands, 'exit' to quit")
        self.logger.success(f"{separator}\n")

        try:
            while True:
                try:
                    # Display prompt
                    prompt = f"mavftp:{remote_cwd}> "
                    command_line = input(prompt).strip()

                    if not command_line:
                        continue

                    # Parse command
                    parts = command_line.split()
                    cmd = parts[0].lower()
                    args = parts[1:]

                    # Handle commands
                    if cmd in ['exit', 'quit', 'q']:
                        self.logger.info("Exiting interactive mode...")
                        break

                    elif cmd in ['help', '?']:
                        self.print_help()

                    elif cmd in ['ls', 'list', 'dir']:
                        path = args[0] if args else remote_cwd
                        # Handle relative paths
                        if not path.startswith('/'):
                            path = os.path.join(remote_cwd, path).replace('\\', '/')
                        self.list_directory(ftp, path)

                    elif cmd == 'cd':
                        if not args:
                            print("Usage: cd <path>")
                            continue

                        new_path = args[0]

                        # Handle special cases
                        if new_path == '/':
                            remote_cwd = '/'
                        elif new_path == '..':
                            # Go up one directory
                            remote_cwd = os.path.dirname(remote_cwd.rstrip('/')).replace('\\', '/')
                            if not remote_cwd:
                                remote_cwd = '/'
                        elif new_path.startswith('/'):
                            # Absolute path
                            remote_cwd = new_path
                        else:
                            # Relative path
                            remote_cwd = os.path.join(remote_cwd, new_path).replace('\\', '/')

                        # Normalize path
                        if not remote_cwd.startswith('/'):
                            remote_cwd = '/' + remote_cwd

                        print(f"Remote directory: {remote_cwd}")

                    elif cmd == 'pwd':
                        print(f"Remote directory: {remote_cwd}")

                    elif cmd == 'lcd':
                        if not args:
                            print("Usage: lcd <path>")
                            continue

                        new_path = args[0]

                        try:
                            # Expand user path
                            new_path = os.path.expanduser(new_path)

                            if os.path.isdir(new_path):
                                os.chdir(new_path)
                                local_cwd = os.getcwd()
                                print(f"Local directory: {local_cwd}")
                            else:
                                print(f"Local directory does not exist: {new_path}")
                        except Exception as e:
                            print(f"Error changing local directory: {e}")

                    elif cmd == 'lpwd':
                        print(f"Local directory: {local_cwd}")

                    elif cmd in ['get', 'download']:
                        if not args:
                            print("Usage: get <remote_file> [local_file]")
                            continue

                        remote_file = args[0]
                        local_file = args[1] if len(args) > 1 else os.path.basename(remote_file)

                        # Handle relative remote path
                        if not remote_file.startswith('/'):
                            remote_file = os.path.join(remote_cwd, remote_file).replace('\\', '/')

                        # Handle relative local path
                        if not os.path.isabs(local_file):
                            local_file = os.path.join(local_cwd, local_file)

                        self.download_file(ftp, remote_file, local_file)

                    elif cmd in ['put', 'upload']:
                        if len(args) < 2:
                            print("Usage: put <local_file> <remote_file>")
                            continue

                        local_file = args[0]
                        remote_file = args[1]

                        # Handle relative local path
                        if not os.path.isabs(local_file):
                            local_file = os.path.join(local_cwd, local_file)

                        # Handle relative remote path
                        if not remote_file.startswith('/'):
                            remote_file = os.path.join(remote_cwd, remote_file).replace('\\', '/')

                        self.upload_file(ftp, local_file, remote_file)

                    elif cmd in ['rm', 'remove', 'delete', 'del']:
                        if not args:
                            print("Usage: rm <remote_file>")
                            continue

                        remote_file = args[0]

                        # Handle relative path
                        if not remote_file.startswith('/'):
                            remote_file = os.path.join(remote_cwd, remote_file).replace('\\', '/')

                        self.remove_file(ftp, remote_file)

                    elif cmd == 'mkdir':
                        if not args:
                            print("Usage: mkdir <remote_dir>")
                            continue

                        remote_dir = args[0]

                        # Handle relative path
                        if not remote_dir.startswith('/'):
                            remote_dir = os.path.join(remote_cwd, remote_dir).replace('\\', '/')

                        self.create_directory(ftp, remote_dir)

                    elif cmd == 'rmdir':
                        if not args:
                            print("Usage: rmdir <remote_dir>")
                            continue

                        remote_dir = args[0]

                        # Handle relative path
                        if not remote_dir.startswith('/'):
                            remote_dir = os.path.join(remote_cwd, remote_dir).replace('\\', '/')

                        self.remove_directory(ftp, remote_dir)

                    elif cmd in ['stat', 'info']:
                        if not args:
                            print("Usage: stat <remote_file>")
                            continue

                        remote_file = args[0]

                        # Handle relative path
                        if not remote_file.startswith('/'):
                            remote_file = os.path.join(remote_cwd, remote_file).replace('\\', '/')

                        self.get_file_info(ftp, remote_file)

                    elif cmd in ['crc', 'checksum']:
                        if not args:
                            print("Usage: crc <remote_file>")
                            continue

                        remote_file = args[0]

                        # Handle relative path
                        if not remote_file.startswith('/'):
                            remote_file = os.path.join(remote_cwd, remote_file).replace('\\', '/')

                        self.calculate_crc(ftp, remote_file)

                    else:
                        print(f"Unknown command: {cmd}")
                        print("Type 'help' for available commands")

                except EOFError:
                    print("\n")
                    self.logger.info("EOF detected, exiting...")
                    break

                except KeyboardInterrupt:
                    print("\n")
                    self.logger.info("Interrupted, exiting...")
                    break

        except Exception as e:
            self.logger.error(f"Error in interactive mode: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())

        print("")
        self.logger.info("Interactive session ended.")

    def run(self):
        # Retrieve configuration options
        mode = self.config['mode'].lower()
        operation = self.config['operation'].lower()
        remote_path = self.config['remote_path']
        local_path = self.config['local_path']
        target_system = int(self.config['target_system'])
        target_component = int(self.config['target_component'])

        separator = "=" * 80
        self.logger.info(separator)
        self.logger.info("MAVLink FTP Client")
        self.logger.info(separator)

        try:
            # Connect to the MAVLink device (supports both serial and network)
            master = self.connect_drone()

            # Override target system/component if specified
            if target_system > 0:
                master.target_system = target_system
            if target_component > 0:
                master.target_component = target_component

            # Initialize MAVFtp
            self.logger.info("Initializing MAVFtp...")
            ftp = MAVFTP(master, target_system=master.target_system, target_component=master.target_component)

            # Check mode
            if mode == "interactive":
                # Run interactive mode
                self.interactive_mode(ftp)
            elif mode == "single":
                # Execute single operation
                self._execute_single_operation(ftp, operation, remote_path, local_path)
            else:
                self.logger.error(f"Unknown mode: {mode}")
                self.logger.info("Supported modes: interactive, single")

        except Exception as e:
            self.logger.error(f"Error during MAVFtp operation: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
        finally:
            # Clean up connection
            self.close_connection(master)

    def _execute_single_operation(self, ftp, operation, remote_path, local_path):
        """
        Execute a single FTP operation (for single mode).
        """
        # Execute the requested operation
        if operation == "list" or operation == "ls":
            self.list_directory(ftp, remote_path)

        elif operation == "get" or operation == "download":
            if not remote_path or remote_path == "/":
                self.logger.error("remote_path must be specified for download operation")
                return

            self.download_file(ftp, remote_path, local_path)

        elif operation == "put" or operation == "upload":
            if not local_path or local_path == "./":
                self.logger.error("local_path must be specified for upload operation")
                return

            if not remote_path or remote_path == "/":
                self.logger.error("remote_path must be specified for upload operation")
                return

            self.upload_file(ftp, local_path, remote_path)

        elif operation == "remove" or operation == "rm" or operation == "delete":
            if not remote_path or remote_path == "/":
                self.logger.error("remote_path must be specified for remove operation")
                return

            self.remove_file(ftp, remote_path)

        elif operation == "mkdir":
            if not remote_path or remote_path == "/":
                self.logger.error("remote_path must be specified for mkdir operation")
                return

            self.create_directory(ftp, remote_path)

        elif operation == "rmdir":
            if not remote_path or remote_path == "/":
                self.logger.error("remote_path must be specified for rmdir operation")
                return

            self.remove_directory(ftp, remote_path)

        elif operation == "stat" or operation == "info":
            if not remote_path or remote_path == "/":
                self.logger.error("remote_path must be specified for stat operation")
                return

            self.get_file_info(ftp, remote_path)

        elif operation == "crc" or operation == "checksum":
            if not remote_path or remote_path == "/":
                self.logger.error("remote_path must be specified for crc operation")
                return

            self.calculate_crc(ftp, remote_path)

        else:
            self.logger.error(f"Unknown operation: {operation}")
            self.logger.info("Supported operations: list, get, put, remove, mkdir, rmdir, stat, crc")
