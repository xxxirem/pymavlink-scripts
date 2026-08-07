import time
from pymavlink import mavutil

# connection_string = "udpin:127.0.0.1:14551" # SITL
connection_string = "tcp:127.0.0.1:5602" # Pi
master = mavutil.mavlink_connection(connection_string)

print("Waiting for heartbeat...")
print(master.wait_heartbeat(timeout=10))
print("OK!")

latitude = 55.7558123   # degrees
longitude = 37.6173456  # degrees
altitude = 150.5        # metres

MAV_CMD_DO_SET_GLOBAL_ORIGIN = 611

# for COMMAND_INT
lat_int = int(latitude * 1e7)
lon_int = int(longitude * 1e7)

master.mav.command_int_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_FRAME_GLOBAL,
    MAV_CMD_DO_SET_GLOBAL_ORIGIN,  # 611
    0,
    0,
    0,
    0, 0, 0,
    lat_int,                                       # X: Latitude (int32, deg * 1e7)
    lon_int,                                       # Y: Longitude (int32, deg * 1e7)
    altitude                                       # Z: Altitude MSL (float, meters)
)
print("Команда COMMAND_INT (MAV_CMD_DO_SET_GLOBAL_ORIGIN) отправлена.")

ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
if ack and ack.command == MAV_CMD_DO_SET_GLOBAL_ORIGIN:
    print(f"COMMAND_ACK получен: {ack.result} (0 = ACCEPTED)")

MSG_GPS_GLOBAL_ORIGIN = 49

master.mav.command_long_send(
    master.target_system,
    master.target_component,
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