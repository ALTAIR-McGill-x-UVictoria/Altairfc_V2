import pymavlink


def mavlink_connection():
    master = pymavlink.mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

    master.wait_heartbeat()
    print("Heartbeat received")