import time
from pymavlink import mavutil

### CONNECTION

connection_string = "udpin:127.0.0.1:14551" # SITL
# connection_string = "tcp:127.0.0.1:5602" # Pi
master = mavutil.mavlink_connection(connection_string)

print("Waiting for heartbeat...")
print(master.wait_heartbeat(timeout=10))
print("OK!")

### SETTING AHRS ORIGIN

latitude = 55.7558123   # degrees
longitude = 37.6173456  # degrees
altitude = 150.5        # metres

MAV_CMD_DO_SET_GLOBAL_ORIGIN = 611


lat_int = int(latitude * 1e7)
lon_int = int(longitude * 1e7)

master.mav.command_int_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_FRAME_GLOBAL,
    MAV_CMD_DO_SET_GLOBAL_ORIGIN,
    0,
    0,
    0,
    0, 0, 0,
    lat_int,
    lon_int,
    altitude
)
print("Команда COMMAND_INT (MAV_CMD_DO_SET_GLOBAL_ORIGIN) отправлена.")

# Проверка установки AHRS ORIGIN
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
if ack and ack.command == MAV_CMD_DO_SET_GLOBAL_ORIGIN:
    print(f"COMMAND_ACK получен: {ack.result} (0 = ACCEPTED)")

MSG_GPS_GLOBAL_ORIGIN = 49

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command ID (512)
    0,
    MSG_GPS_GLOBAL_ORIGIN,
    0, 0, 0, 0, 0, 0,
)
print("Запрос MAV_CMD_REQUEST_MESSAGE (ID 49) отправлен...")

msg = master.recv_match(type="GPS_GLOBAL_ORIGIN", blocking=True, timeout=5)

if msg:

    lat = msg.latitude / 1e7
    lon = msg.longitude / 1e7
    alt = msg.altitude / 1000.0  # Из миллиметров в метры

    print("\n[УСПЕХ] Координаты EKF Origin получены:")
    print(f"  Latitude:  {lat:.7f}")
    print(f"  Longitude: {lon:.7f}")
    print(f"  Altitude:  {alt:.2f} м (MSL)")
else:
    print("\n[ТАЙМАУТ] Ответ GPS_GLOBAL_ORIGIN не получен.")


### Переключение в GUIDED !

mode_id = master.mode_mapping().get("GUIDED")
print("Включение режима GUIDED...")
MAV_CMD_DO_SET_MODE = 176
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    MAV_CMD_DO_SET_MODE,
    0,
    1,  # Param 1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    mode_id,
    0,
    0,
    0,
    0,
    0,
)

if (log:= master.wait_heartbeat(timeout=3).custom_mode == mode_id):
    print("Режим GUIDED включен")
else:
    print("Ошибка!")
    print(log)

### ARM

print("> ARMING MOTORS")
master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

start_time = time.time()
while time.time() - start_time < 10:
    msg = master.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        print(f"  ArduPilot: {msg.text}")

    master.wait_heartbeat()

    if master.motors_armed():
        print(">>> VEHICLE IS ARMED! <<<")
        break


### TAKEOFF

MAV_CMD_NAV_TAKEOFF_LOCAL = 24
target_alt = 2

print("Отправка Takeoff команды...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    MAV_CMD_NAV_TAKEOFF_LOCAL,
    0,      # confirmation
    0,      # p1: pitch (rad)
    0,
    1,    # p3: Ascend rate (m/s)
    0,      # p4: yaw (rad)
    0,      # x (m)
    0,      # y (m)
    target_alt,      # z (m)
)

print(f"Запрос MAV_CMD_NAV_TAKEOFF_LOCAL ({target_alt} meters) отправлен...")

print(f"Ожидание COMMAND_ACK...")
msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)

if msg:
    if msg.command == MAV_CMD_NAV_TAKEOFF_LOCAL:
        print(f"COMMAND_ACK.result: {msg.result}")
else:
    print("\n[ТАЙМАУТ] Ответ не получен.")