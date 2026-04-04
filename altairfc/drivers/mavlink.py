from pymavlink import mavutil

def mavlink_connection():
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

    master.wait_heartbeat()
    print("Heartbeat received")

    while True:
        msg = master.recv_match(type='ATTITUDE', blocking=True)
        if msg is not None:
            print(msg)
