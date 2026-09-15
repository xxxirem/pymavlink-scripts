import sys
from pymavlink import mavutil

# connection_string = "udpin:127.0.0.1:14551" # SITL
connection_string = "tcp:127.0.0.1:5602" # Pi
master = mavutil.mavlink_connection(connection_string)

print("Waiting for heartbeat...")
print(master.wait_heartbeat(timeout=10))
print("OK!")


master.mav.command_long_send(
    master.target_system,                     # Target system ID
    master.target_component,                  # Target component ID
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command ID (512)
    0,                                        # Confirmation
    148,                                      # Param 1: Requested Message ID (AUTOPILOT_VERSION)
    0, 0, 0, 0, 0, 0                          # Param 2-7 (Unused)
)

msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=5)

if msg:
    print("\nCapabilities received:")
    print(msg)
    print(f"\nCapabilities Bitmask: {msg.capabilities}")
else:
    print("\nRequest timed out or no response received.")


caps = msg.capabilities
flags = {
    "MISSION_FLOAT": mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT,
    "PARAM_FLOAT": mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT,
    "MISSION_INT": mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_INT,
    "COMMAND_INT": mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_COMMAND_INT,
    "FTP": mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_FTP,
    "SET_ATTITUDE_TARGET": mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET,
    "SET_POSITION_TARGET_LOCAL_NED": mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED,
    "SET_POSITION_TARGET_GLOBAL_INT": mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT,
    "TERRAIN": mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_TERRAIN,
    "MAVLINK2": mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MAVLINK2,
    "MISSION_FENCE": mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE,
    "MISSION_RALLY": mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY,
}

print("\nSupported Protocol Capabilities:")
for name, flag in flags.items():
    if caps & flag:
        print(f"  [X] {name}")