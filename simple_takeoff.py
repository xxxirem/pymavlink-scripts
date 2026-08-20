from pymavlink import mavutil
import guided_functions as gf

CONNECTION_STRING = "udpin:127.0.0.1:14551"
# CONNECTION_STRING = "tcp:127.0.0.1:5602"
LAT = 55.7558123  # degrees
LON = 37.6173456  # degrees
ALT = 150.5  # metres


def main():
    master = mavutil.mavlink_connection(CONNECTION_STRING)

    if not gf.check_connection(master):
        return 1

    gf.print_status(master)

    # set_home(master)
    gf.set_global_origin(master, LAT, LON, ALT)
    gf.set_mode(master, "GUIDED")
    gf.arm(master)
    gf.takeoff(master, 1.5)
    gf.land(master)
    gf.disarm(master)

if __name__ == "__main__":
    main()