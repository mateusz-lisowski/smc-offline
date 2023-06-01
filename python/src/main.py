import serial.tools.list_ports
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client import InfluxDBClient, Point
from pymavlink import mavutil
import datetime
import time
import math
import os


# RADIO: /dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30EZ7ND-if00-port0
# USB-PX: /dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00
# RADIO: /dev/serial/usb-FTDI_FT231X_USB_UART_D30EZEGB-if00-port0

SERIAL_PORT = '/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00'
TOKEN = os.environ.get('INFLUXDB_TOKEN')
IP = os.environ.get('IP_ADDRESS')
PORT = os.environ.get('INFLUXDB_PORT')
ORG = os.environ.get('ORGANISATION')
BUCKET = os.environ.get('BUCKET_NAME')
print("Config:", TOKEN, IP, PORT, ORG, BUCKET)

mavcon = None
client = None

max_retries = 5
retry_interval = 3  # seconds
retry_count = 0
is_mavlink_connected = False
is_influxdb_connected = False
baudrate = 57600

bytes_sent = 0
bytes_recv = 0
errors_recv = 0

serial_list = mavutil.auto_detect_serial(
    preferred_list=['*CP210X*', '*FTDI*', "*Arduino_Mega_2560*",
                    "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*', "*Gumstix*"]
)


def int2dict(num, tag):
    binary_string = bin(num)[2:]
    binary_dict = {f"{tag}_{i}": int(bit)
                   for i, bit in enumerate(binary_string[::-1])}
    return binary_dict


if __name__ == "__main__":

    # print used serial ports
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))

    # Attempt to establish InfluxDB connection
    while not is_influxdb_connected and retry_count < max_retries:
        try:
            print(
                f"Attempting to connect (Retry {retry_count + 1} of {max_retries})...")
            client = InfluxDBClient(
                url=f"http://{IP}:{PORT}", token=TOKEN, org=ORG)
            is_influxdb_connected = True
            retry_count = 0
            print("InfluxDB connection established")
        except Exception as e:
            print(f"Failed to connect: {e}")
            retry_count += 1
            time.sleep(retry_interval)

    if not is_influxdb_connected:
        print(
            f"Failed to establish InfluxDB connection after {max_retries} retries.")

    # Attempt to establish MAVLink connection
    while not is_mavlink_connected and retry_count < max_retries:
        try:
            print(
                f"Attempting to connect (Retry {retry_count + 1} of {max_retries})...")
            # Serial port is hard-coded cause otherwise doesn't work ...
            # mavcon = mavutil.mavlink_connection(str(serial_list[0]), baud=baudrate, autoreconnect=True)
            mavcon = mavutil.mavlink_connection(
                SERIAL_PORT, baud=baudrate, autoreconnect=True, dialect="rocket")

            print("Waiting for heartbeat...")
            mavcon.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            try:
                mavcon.wait_heartbeat()
            except serial.serialutil.SerialException:
                print("Error occured when awaiting for heartbeat!")

            print(f"MAVLink connection established!")
            is_mavlink_connected = True
        except Exception as e:
            print(f"Failed to connect: {e}")
            retry_count += 1
            time.sleep(retry_interval)

    if not is_mavlink_connected:
        print(
            f"Failed to establish MAVLink connection after {max_retries} retries.")
        exit()

    write_api = client.write_api(write_options=SYNCHRONOUS)

    # now there is about 2 second delay
    t1 = time.time()
    while True:

        # receive and write telemetry
        while True:
            try:
                msg = mavcon.recv_match(blocking=True)
            except Exception as e:
                print(f"Lost connection with Pixhawk: {e}")
                break

            dict_msg = msg.to_dict()
            title = dict_msg['mavpackettype']

            if title == "SIMBA_ACTUATOR_STATUS":
                act_values = int2dict(dict_msg['values'], "act_node")
                act_errors = int2dict(dict_msg['errors'], "act_node_err")
                dict_msg.update(act_values)
                dict_msg.update(act_errors)

            elif title == "SIMBA_HEARTBEATS":
                heartbeats = int2dict(dict_msg['nodes_status'], "node_hbeat")
                resets = int2dict(dict_msg['nodes_reset'], "node_reset")
                dict_msg.update(heartbeats)
                dict_msg.update(resets)

            elif title == "SIMBA_TEMPERATURE_STATUS":
                temp_health = int2dict(
                    dict_msg['sensors_error'], "temp_health")
                dict_msg.update(temp_health)

            elif title == "GPS_RAW_INT":
                dict_msg['lat'] = round(dict_msg['lat'] / 10000000, 6)
                dict_msg['lon'] = round(dict_msg['lat'] / 10000000, 6)
                dict_msg['alt'] = round(dict_msg['lat'] / 1000, 3)

            elif title == "GLOBAL_POSITION_INT":
                dict_msg['alt'] = round(dict_msg['lat'] / 1000, 3)
                dict_msg['relative_alt'] = round(dict_msg['lat'] / 1000, 3)

            elif title == "ODOMETRY" or title == "LOCAL_POSITION_NED":
                vx = dict_msg['vx']
                vy = dict_msg['vy']
                dict_msg.update({'horizontal_speed', math.sqrt(vx**2 + vy**2)})

            msg_iter = iter(dict_msg.items())
            next(msg_iter)
            for key, value in msg_iter:
                try:
                    write_api.write(BUCKET, ORG, Point(title).field(
                        key, value).time(time.time_ns()))
                except Exception as e:
                    pass

            t2 = time.time()
            if t2 - t1 > 1.0:
                try:
                    # kB/s
                    write_api.write(BUCKET, ORG, Point("BANDWITH")
                                    .field("bytes_recv", 0.001*(mavcon.mav.total_bytes_received-bytes_recv)/(t2-t1))
                                    .time(time.time_ns()))
                    write_api.write(BUCKET, ORG, Point("BANDWITH")
                                    .field("bytes_sent", 0.001*(mavcon.mav.total_bytes_sent-bytes_sent)/(t2-t1))
                                    .time(time.time_ns()))
                    # write_api.write(BUCKET, ORG, Point("BANDWITH")\
                    #                 .field("errors_per_second", mavcon.mav.total_receive_errors-errors_recv/(t2-t1))\
                    #                 .time(time.time_ns()))
                    # write_api.write(BUCKET, ORG, Point("BANDWITH")\
                    #                 .field("total_errors", mavcon.mav.total_receive_errors).time(time.time_ns()))
                except Exception as e:
                    print(e)
                    write_api.write(BUCKET, ORG, Point("BANDWITH").field(
                        "bytes_recv", 0.0).time(time.time_ns()))
                    write_api.write(BUCKET, ORG, Point("BANDWITH").field(
                        "bytes_sent", 0.0).time(time.time_ns()))

                bytes_sent = mavcon.mav.total_bytes_sent
                bytes_recv = mavcon.mav.total_bytes_received
                errors_recv = mavcon.mav.total_receive_errors
                t1 = t2

        # reconnection
        while True:
            ports = serial.tools.list_ports.comports()
            for port, desc, hwid in sorted(ports):
                print("{}: {} [{}]".format(port, desc, hwid))
            try:
                mavcon = mavutil.mavlink_connection(
                    SERIAL_PORT, baud=baudrate, autoreconnect=True)
                print("Waiting for heartbeat...")
                mavcon.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                          mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                try:
                    mavcon.wait_heartbeat()
                except serial.serialutil.SerialException:
                    print("Error occured when awaiting for heartbeat!")

                print(f"MAVLink connection established!")
                break
            except Exception as e:
                print(f"Failed to reconnect: {e}")
                time.sleep(retry_interval)
