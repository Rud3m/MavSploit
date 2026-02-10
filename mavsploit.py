#!/usr/bin/python3
import os
import sys
from sploitkit import FrameworkConsole
from tinyscript import *

class MySploitConsole(FrameworkConsole):
    # TODO: set your console attributes
    sources = {'banners': "banners"}
    pass

def check_sudo():
    if os.geteuid() != 0:
        sys.stderr.write("Error: This script must be run as root (use sudo).\n")
        sys.exit(1)

if __name__ == '__main__':
    # Check if the script is run as sudo
    check_sudo()

    # Argument parser setup
    parser.add_argument("-d", "--dev", action="store_true", help="enable development mode")
    parser.add_argument("-r", "--rcfile", type=ts.file_exists, help="execute commands from a rcfile")

    initialize(exit_at_interrupt=False)
    c = MySploitConsole(
        "MAVSploit",
        # TODO: configure your console settings
        dev=args.dev,
        debug=args.verbose,
    )
    c.rcfile(args.rcfile) if args.rcfile else c.start()
