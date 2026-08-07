import time
from pymavlink import mavutil

connection_string = 'udpin:127.0.0.1:14550'
print(f"Connecting to vehicle on: {connection_string}")
master = mavutil.mavlink_connection(connection_string)

master.wait_heartbeat()
print(f"Connected! (System ID: {master.target_system})")


def set_mode(mode_name):
    """Смена режима с обязательным вычитыванием Heartbeat"""
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"Unknown mode: {mode_name}")
        return False

    print(f"Setting mode to {mode_name}...")

    start_time = time.time()
    while time.time() - start_time < 10:  # Таймаут 10 секунд
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

        # ВАЖНО: считываем Heartbeat, чтобы обновить master.flightmode
        master.wait_heartbeat()

        if master.flightmode == mode_name:
            print(f"Mode set to {mode_name} confirmed.")
            return True
        time.sleep(0.5)

    print(f"Failed to set mode {mode_name}")
    return False


def arm_vehicle():
    """Арминг с обновлением состояния motors_armed"""
    print("Arming motors...")

    start_time = time.time()
    while time.time() - start_time < 15:  # Таймаут 15 секунд
        # Вычитываем накопившиеся текстовые сообщения от полетника (PreArm errors)
        msg = master.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            print(f"  ArduPilot: {msg.text}")

        # ВАЖНО: считываем Heartbeat, чтобы обновить master.motors_armed()
        master.wait_heartbeat()

        if master.motors_armed():
            print(">>> VEHICLE IS ARMED! <<<")
            return True

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)

    print("Arming failed!")
    return False


def disarm_vehicle():
    """Дисарминг с обновлением состояния motors_armed"""
    print("Disarming motors...")

    start_time = time.time()
    while time.time() - start_time < 10:  # Таймаут 10 секунд
        master.wait_heartbeat()

        if not master.motors_armed():
            print(">>> VEHICLE IS DISARMED! <<<")
            return True

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)

    print("Disarming failed!")
    return False


if __name__ == "__main__":
    if set_mode("GUIDED"):
        if arm_vehicle():
            print("Holding ARMED state for 5 seconds...")
            time.sleep(3)
            disarm_vehicle()