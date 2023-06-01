#!/usr/bin/env python3
try:
    from pymavlink import mavutil
    import serial
except Exception as e:
    print("Failed to import pymavlink and/or serial.")
    print("You may need to install it with 'pip3 install pymavlink pyserial'")
    quit(-1)

import time
import os
os.environ["MAVLINK_DIALECT"] = "rocket"
clear = lambda: os.system('clear')

# main()
if __name__ == '__main__':
    mavutil.set_dialect("rocket")
    dict = {}
    baudrate = 57600
    print("Connecting...")
    serial_list = mavutil.auto_detect_serial(
        preferred_list=['*FTDI*', "*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*',
                        "*Gumstix*"])
    con = mavutil.mavlink_connection(str(serial_list[0]), baud=baudrate, autoreconnect=True, dialect="rocket")
    print("done.")
    print("Waiting for heartbeat...")
    con.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    try:
        con.wait_heartbeat()
    except serial.serialutil.SerialException:
        pass
    print("done.")

    clock1 = time.time()
    bad_data = 0
    ok_data = 0
    msg = None
    while True:
        clock2 = time.time()
        try:
            msg = con.recv_match(blocking=True, timeout=1000)
        except serial.serialutil.SerialException:
            serial_list = mavutil.auto_detect_serial(
                preferred_list=['*FTDI*', "*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*',
                                "*Gumstix*"])
            con = mavutil.mavlink_connection(str(serial_list[0]), baud=baudrate, autoreconnect=True)
        """
        if msg.get_type() == "UNKNOWN_68":
            print(type(msg))
            print(msg.get_type())
            print(msg.data)
        """
        type = msg.get_type()
        # print(msg)
        if type is not "BAD_DATA":
            dict[type] = msg
            ok_data += 1
        else:
            bad_data += 1

        if clock2 - clock1 > 0.3:
            clear()
            for a in sorted(dict):
                print(a)
            print("BAD_DATA:", bad_data, "/", ok_data + bad_data)
            clock1 = time.time()

