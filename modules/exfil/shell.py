from sploitkit import Config, Option
from lib.base import MAVLinkModule
from pymavlink import mavutil
import select
import sys
import termios
import tty
import time

class MAVLinkShell(MAVLinkModule):
    """
    MAVLink Shell - Interactive shell access to MAVLink devices supporting SERIAL_CONTROL (PX4/NuttX).

    Connection:
        Supports both network and serial connections:
        - Network: udp:10.13.0.2:14550 or tcp:10.13.0.2:5760
        - Serial: /dev/ttyUSB0 (Linux) or COM3 (Windows)

    Usage:
        set connection udp:10.13.0.2:14550
        set mode interactive
        run
    """

    # Inherit connection config from base class and add module-specific options
    config = MAVLinkModule.config + Config({
        Option(
            name='mode',
            description='Shell mode: interactive or command',
            required=False,
        ): "interactive",  # Default value
        Option(
            name='command',
            description='Single command to execute (when mode=command)',
            required=False,
        ): "id",  # Default value
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

    def send_command(self, master, command):
        """
        Send a command to the shell using SERIAL_CONTROL.
        """
        # Ensure command ends with newline
        if not command.endswith('\n'):
            command += '\n'

        # Convert command to bytes
        cmd_bytes = command.encode('utf-8')

        # MAVLink SERIAL_CONTROL parameters
        device = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
        flags = (mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND |
                 mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE)
        timeout = 0

        # Send command in chunks (max 70 bytes per message)
        chunk_size = 70
        for i in range(0, len(cmd_bytes), chunk_size):
            chunk = cmd_bytes[i:i + chunk_size]

            # Pad to 70 bytes with zeros
            data = list(chunk) + [0] * (70 - len(chunk))

            master.mav.serial_control_send(
                device,
                flags,
                timeout,
                0,  # baudrate (not used for shell)
                len(chunk),
                data
            )

            # Small delay between chunks
            time.sleep(0.01)

    def receive_output(self, master, timeout=1.0):
        """
        Receive output from the shell.
        """
        output = b''
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            msg = master.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=0.1)

            if msg and msg.device == mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL:
                # Extract data from the message
                count = msg.count
                if count > 0:
                    data = bytes(msg.data[:count])
                    output += data
                    # Reset timeout on receiving data
                    start_time = time.time()

        return output.decode('utf-8', errors='replace')

    def execute_single_command(self, master, command):
        """
        Execute a single command and return the output.
        """
        self.logger.info(f"Executing command: {command}")

        # Send command
        self.send_command(master, command)

        # Wait a bit for execution
        time.sleep(0.5)

        # Receive output
        output = self.receive_output(master, timeout=2.0)

        if output:
            separator = "=" * 80
            self.logger.success(f"\n{separator}")
            self.logger.success("COMMAND OUTPUT")
            self.logger.success(f"{separator}\n")
            print(output)
            print(f"{separator}\n")
        else:
            self.logger.warning("No output received (command may not be supported or shell not available)")

        return output

    def interactive_shell(self, master):
        """
        Start an interactive shell session.
        """
        separator = "=" * 80
        self.logger.success(f"\n{separator}")
        self.logger.success("MAVLink Interactive Shell")
        self.logger.success(f"{separator}")
        self.logger.info("Type 'exit' or press Ctrl+C to quit")
        self.logger.info("Note: Not all devices support interactive shell (mainly PX4/NuttX)")
        self.logger.success(f"{separator}\n")

        # Save original terminal settings
        old_settings = None
        try:
            old_settings = termios.tcgetattr(sys.stdin)
        except:
            self.logger.warning("Could not get terminal settings. Using simple mode.")

        try:
            if old_settings:
                # Set terminal to raw mode for better interactivity
                tty.setraw(sys.stdin.fileno())

            buffer = ""

            while True:
                # Check for input from user
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    char = sys.stdin.read(1)

                    if char:
                        # Handle special characters
                        if char == '\x03':  # Ctrl+C
                            break
                        elif char == '\r':  # Enter
                            print('\r\n', end='', flush=True)

                            # Send command
                            if buffer.strip() == 'exit':
                                break

                            if buffer.strip():
                                self.send_command(master, buffer)

                            buffer = ""
                        elif char == '\x7f':  # Backspace
                            if buffer:
                                buffer = buffer[:-1]
                                print('\b \b', end='', flush=True)
                        else:
                            buffer += char
                            print(char, end='', flush=True)

                # Check for output from device
                msg = master.recv_match(type='SERIAL_CONTROL', blocking=False)

                if msg and msg.device == mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL:
                    count = msg.count
                    if count > 0:
                        data = bytes(msg.data[:count])
                        try:
                            text = data.decode('utf-8', errors='replace')
                            print(text, end='', flush=True)
                        except:
                            pass

        except KeyboardInterrupt:
            self.logger.info("\n\nShell session interrupted.")
        except Exception as e:
            self.logger.error(f"\nError in shell session: {str(e)}")
        finally:
            # Restore terminal settings
            if old_settings:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                except:
                    pass

            print("\n")
            self.logger.info("Shell session ended.")

    def simple_shell(self, master):
        """
        Simple line-based shell for systems without terminal support.
        """
        separator = "=" * 80
        self.logger.success(f"\n{separator}")
        self.logger.success("MAVLink Simple Shell")
        self.logger.success(f"{separator}")
        self.logger.info("Type 'exit' or press Ctrl+C to quit")
        self.logger.info("Enter commands and press Enter to execute")
        self.logger.success(f"{separator}\n")

        try:
            while True:
                try:
                    # Get command from user
                    command = input("mav> ")

                    if command.strip().lower() == 'exit':
                        break

                    if command.strip():
                        # Send command
                        self.send_command(master, command)

                        # Wait for output
                        time.sleep(0.5)
                        output = self.receive_output(master, timeout=2.0)

                        if output:
                            print(output)

                except EOFError:
                    break

        except KeyboardInterrupt:
            self.logger.info("\n\nShell session interrupted.")
        except Exception as e:
            self.logger.error(f"\nError in shell session: {str(e)}")

        print("")
        self.logger.info("Shell session ended.")

    def run(self):
        # Retrieve configuration options
        mode = self.config['mode'].lower()
        command = self.config['command']
        target_system = int(self.config['target_system'])
        target_component = int(self.config['target_component'])

        separator = "=" * 80
        self.logger.info(separator)
        self.logger.info("MAVLink Shell")
        self.logger.info(separator)

        try:
            # Connect to the MAVLink device (supports both serial and network)
            master = self.connect_drone()

            # Override target system/component if specified
            if target_system > 0:
                master.target_system = target_system
            if target_component > 0:
                master.target_component = target_component

            # Execute based on mode
            if mode == "command":
                if not command:
                    self.logger.error("command parameter must be set when mode=command")
                    return

                self.execute_single_command(master, command)

            elif mode == "interactive":
                # Try interactive shell, fall back to simple if terminal not available
                try:
                    termios.tcgetattr(sys.stdin)
                    self.interactive_shell(master)
                except:
                    self.logger.warning("Terminal not available, using simple shell mode")
                    self.simple_shell(master)

            elif mode == "simple":
                self.simple_shell(master)

            else:
                self.logger.error(f"Unknown mode: {mode}")
                self.logger.info("Supported modes: interactive, simple, command")

        except Exception as e:
            self.logger.error(f"Error during shell operation: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
        finally:
            # Clean up connection
            self.close_connection(master)
