import time
from pymavlink import mavutil
from fly_to_my import set_ahrs_origin, takeoff, land

# CONNECTION_STRING = "udpin:127.0.0.1:14551"
CONNECTION_STRING = "tcp:127.0.0.1:5602"
MAV_CMD_DO_SET_GLOBAL_ORIGIN = 611
EPSILON = 0.15


def check_connection(master_instance, timeout=5):
    start_time = time.time()
    while time.time() - start_time <= timeout:
        master_instance.wait_heartbeat(timeout=1.0)
        if master_instance.target_system != 0:
            print(
                f"[Connected] System ID: {master_instance.target_system}, Component ID: {master_instance.target_component}"
            )
            return True
        else:
            print(
                f"[Wrong ID] System ID: {master_instance.target_system} Trying again..."
            )
    print("[Timeout] Connection failed")
    return False


def print_status(master_instance):
    print(master_instance.wait_heartbeat())


def recv_ack(master_instance, command, timeout=3):
    cmd_ack_flags = {
        "0": "Accepted",
        "1": "Temporarily Rejected",
        "2": "Denied",
        "3": "Unsupported",
        "4": "Failed",
        "5": "In Progress",
        "6": "Cancelled",
        "7": "CMD_LONG Only",
        "8": "CMD_INT Only",
        "9": "CMD Unsupported MAV_FRAME",
        "10": "Not In Control",
    }

    ack = master_instance.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
    if ack and ack.command == command:
        if ack.result == 0:
            print(
                f"[Accepted] COMMAND_ACK.result: {ack.result} ({cmd_ack_flags[str(ack.result)]})"
            )
        else:
            print(
                f"[Not Accepted] COMMAND_ACK.result: {ack.result} ({cmd_ack_flags[str(ack.result)]})"
            )


def set_mode(master_instance, mode_name):
    mode_id = master_instance.mode_mapping().get(mode_name)
    if mode_id is None:
        return False
    print(f"[Set Mode] Command SET_MODE ({mode_name}) send.")
    while master_instance.flightmode != mode_name:
        master_instance.mav.set_mode_send(
            master_instance.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        master_instance.wait_heartbeat(timeout=1)
    return True


def arm(master_instance):
    master_instance.mav.command_long_send(
        master_instance.target_system,
        master_instance.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    print("[Arm] Command COMMAND_LONG (MAV_CMD_COMPONENT_ARM_DISARM) send.")
    recv_ack(master_instance, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    while not master_instance.motors_armed():
        master_instance.wait_heartbeat(timeout=1)


def disarm(master_instance):
    master_instance.mav.command_long_send(
        master_instance.target_system,
        master_instance.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    print("[Disarm] Command COMMAND_LONG (MAV_CMD_COMPONENT_ARM_DISARM) send.")
    recv_ack(master_instance, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    while master_instance.motors_armed():
        master_instance.wait_heartbeat(timeout=1)


def print_loc_pos(master_instance, duration=5.0):
    start_time = time.time()
    while time.time() - start_time < duration:
        msg = master_instance.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=1
        )

        if msg is None:
            print("[LOC_POS_NED] Waiting for LOCAL_POSITION_NED message...")
            continue

        print(
            f"Pos: [{msg.x:7.2f}, {msg.y:7.2f}, {msg.z:7.2f}] m | "
            f"Vel: [{msg.vx:6.2f}, {msg.vy:6.2f}, {msg.vz:6.2f}] m/s"
        )


def main():
    master = mavutil.mavlink_connection(CONNECTION_STRING)

    if not check_connection(master):
        return 1

    print_status(master)
    print("Before ahrs origin (MAV_CMD_DO_SET_HOME) set.\nLocal position NED:")
    print_loc_pos(master, 3)


    print("After ahrs origin (MAV_CMD_DO_SET_HOME) set.\nLocal position NED:")
    set_ahrs_origin(master)
    set_mode(master, "GUIDED")
    print_loc_pos(master, 3)
    # arm(master)
    # takeoff(master)
    # land(master)

if __name__ == "__main__":
    main()
