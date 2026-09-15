from pymavlink import mavutil
import guided_functions as gf

LAT = 55.7558123  # degrees
LON = 37.6173456  # degrees
ALT = 150.5  # metres


def main():
    connection_string = "udpin:127.0.0.1:14551" # SITL
    # connection_string = "tcp:127.0.0.1:5602"  # Pi
    master = mavutil.mavlink_connection(connection_string)

    print("Waiting for heartbeat...")
    print(master.wait_heartbeat(timeout=10))
    print("OK!")

    gf.set_global_origin(master, LAT, LON, ALT)

    gf.print_global_origin(master)


if __name__ == "__main__":
    main()
