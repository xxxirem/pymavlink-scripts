import time
from pymavlink import mavutil

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


def set_home(master_instance):
    master_instance.mav.command_long_send(
        master_instance.target_system,
        master_instance.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        1,
        0.01,
        0.01,
        0,
        0,
        0,
        0,
    )
    print("[Set Home] Command COMMAND_INT (MAV_CMD_DO_SET_HOME) send.")
    recv_ack(master_instance, mavutil.mavlink.MAV_CMD_DO_SET_HOME)


def set_global_origin(master_instance, lat, lon, alt):
    """Sets EKF Origin required for navigation in local coordinates (LOCAL_POSITION_NED)"""
    # convertation
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)

    MAV_CMD_DO_SET_GLOBAL_ORIGIN = 611

    master_instance.mav.command_int_send(
        master_instance.target_system,
        master_instance.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        MAV_CMD_DO_SET_GLOBAL_ORIGIN,
        0,
        0,
        0,
        0,
        0,
        0,
        lat_int,  # X: Latitude (int32, deg * 1e7)
        lon_int,  # Y: Longitude (int32, deg * 1e7)
        alt,  # Z: Altitude MSL (float, meters)
    )
    print(
        f"[Set Global Origin] Command (MAV_CMD_DO_SET_GLOBAL_ORIGIN) send. LAT: {lat}, LON: {lon}, ALT: {alt}"
    )
    recv_ack(master_instance, MAV_CMD_DO_SET_GLOBAL_ORIGIN)


def print_global_origin(master_instance):
    """Prints Global (EKF) origin"""
    MSG_GPS_GLOBAL_ORIGIN = 49

    master_instance.mav.command_long_send(
        master_instance.target_system,
        master_instance.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command ID (512)
        0,  # Confirmation
        MSG_GPS_GLOBAL_ORIGIN,  # Param 1: ID запрашиваемого сообщения (49)
        0,
        0,
        0,
        0,
        0,  # Param 2-6: Reserved (0)
        0,  # Param 7: Response Target (0)
    )
    print("[Get Origin] MAV_CMD_REQUEST_MESSAGE (MSG_GPS_GLOBAL_ORIGIN) send.")
    recv_ack(master_instance, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE)
    msg = master_instance.recv_match(type="GPS_GLOBAL_ORIGIN", blocking=True, timeout=3)

    if msg:
        lat = msg.latitude / 1e7
        lon = msg.longitude / 1e7
        alt = msg.altitude / 1000.0  # Из миллиметров в метры

        print("\n[Success] EKF Origin coordinates:")
        print(f"  Latitude:  {lat:.7f}")
        print(f"  Longitude: {lon:.7f}")
        print(f"  Altitude:  {alt:.2f} m (MSL)")
    else:
        print("\n[Timeout] GPS_GLOBAL_ORIGIN message wasn't recieved.")


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


def takeoff(master_instance, target_alt=1.5):
    master_instance.mav.command_long_send(
        master_instance.target_system,
        master_instance.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        target_alt,
    )
    print(f"[Takeoff] Command (MAV_CMD_NAV_TAKEOFF) send. Target Alt = {target_alt}")
    recv_ack(master_instance, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    while True:
        msg = master_instance.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=1
        )

        if msg is None:
            print("[Takeoff] Waiting for LOCAL_POSITION_NED message...")
            continue

        current_alt = -msg.z

        print(
            f"Pos: [{msg.x:7.2f}, {msg.y:7.2f}, {msg.z:7.2f}] m | "
            f"Vel: [{msg.vx:6.2f}, {msg.vy:6.2f}, {msg.vz:6.2f}] m/s"
        )

        if abs(current_alt - target_alt) <= EPSILON:
            print(f"[Takeoff] Target altitude reached: {current_alt:.2f} m")
            break
    return True


def move_relative(master_instance, dx, dy, dz):
    """Задает положение по относительным координатам, положительное значение dz - вниз"""
    type_mask = 0b0000101111000000
    init_x, init_y, init_z = None, None, None
    print("[Move Relative] Getting initial position...")
    while True:
        msg = master_instance.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=1
        )

        if msg is None:
            print("[Move Relative] Waiting for LOCAL_POSITION_NED message...")
            continue

        init_x, init_y, init_z = msg.x, msg.y, msg.z
        print(
            f"[Move Relative] Init position (XYZ): [{init_x:8.3f}, {init_y:8.3f}, {init_z:8.3f}] m"
        )
        break
    tar_x, tar_y, tar_z = init_x + dx, init_y + dy, init_z + dz

    print("[Move Relative] Command SET_POSITION_TARGET_LOCAL_NED send.")
    master_instance.mav.set_position_target_local_ned_send(
        0,
        master_instance.target_system,
        master_instance.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        type_mask,
        dx,
        dy,
        dz,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    while True:
        msg = master_instance.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=1
        )

        if msg is None:
            print("[Move Relative] Waiting for LOCAL_POSITION_NED message...")
            continue

        print(
            f"Pos: [{msg.x:7.2f}, {msg.y:7.2f}, {msg.z:7.2f}] m | "
            f"Vel: [{msg.vx:6.2f}, {msg.vy:6.2f}, {msg.vz:6.2f}] m/s"
        )

        if (
            abs(tar_x - msg.x) <= EPSILON
            and abs(tar_y - msg.y) <= EPSILON
            and abs(tar_z - msg.z) <= EPSILON
        ):
            print(
                f"[Move Relative] Target position reached: {msg.x:8.3f}, {msg.y:8.3f}, {msg.z:8.3f} m"
            )
            break
    return True


def land(master_instance):
    print("[Land] Command SET_MODE (LAND) send.")
    set_mode(master_instance, "LAND")
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
