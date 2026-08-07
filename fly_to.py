import sys
import time
from pymavlink import mavutil

CONNECTION_STRING = "udpin:127.0.0.1:14551" # SITL
# CONNECTION_STRING = "tcp:127.0.0.1:5602" # Pi

TARGET_ALTITUDE = 3.0  # Высота первого взлета (метры)
MAX_CLIMB_RATE = 1.5 # Макс. скорость подъема (м/с)
MOVE_SPEED = 0.9  # Ограничение скорости (м/с)

print(f"[INIT] Подключение к полетному контроллеру: {CONNECTION_STRING}")
master = mavutil.mavlink_connection(CONNECTION_STRING)



# 1. Ждем НАСТОЯЩИЙ Heartbeat от автопилота (пропускаем System ID = 0)
def connect_to_vehicle(timeout=15):
    print("[INIT] Ожидание сигнала Heartbeat от автопилота...")
    start_time = time.time()
    while time.time() - start_time < timeout:
        master.wait_heartbeat(timeout=1)
        if master.target_system != 0:
            print(
                f"[INIT] Успешно подключено! (System ID: {master.target_system}, Component ID: {master.target_component})"
            )
            return True
        print("[INIT] Получен служебный heartbeat (System 0), ждем дальше...")
    return False


if not connect_to_vehicle():
    print(" [ERROR] Не удалось подключиться к автопилоту (System ID = 0)!")
    sys.exit(1)


def get_target_sys():
    """Возвращает корректный System ID (защита от 0)"""
    return master.target_system if master.target_system != 0 else 1


def get_target_comp():
    """Возвращает корректный Component ID (защита от 0)"""
    return (
        master.target_component
        if master.target_component != 0
        else mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
    )


def print_statustext():
    """Вычитывает сообщения от ArduPilot"""
    while True:
        msg = master.recv_match(type="STATUSTEXT", blocking=False)
        if not msg:
            break
        print(f"  [ArduPilot]: {msg.text}")


def set_mode(mode_name, timeout=10):
    """Переключение режима полета"""
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f" [ERROR] Неизвестный режим: {mode_name}")
        return False

    print(f"[MODE] Запрос переключения в режим {mode_name}...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        master.mav.set_mode_send(
            get_target_sys(),
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        master.wait_heartbeat(timeout=1)

        if master.flightmode == mode_name:
            print(f"[MODE] Режим {mode_name} успешно подтвержден!")
            return True
        print_statustext()
        time.sleep(0.5)

    print(f" [ERROR] Не удалось перейти в режим {mode_name}")
    return False


def set_home_position():
    """Фиксация точки Home"""
    print("[HOME] Фиксация Home Position в текущих координатах EKF...")
    master.mav.command_long_send(
        get_target_sys(),
        get_target_comp(),
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        1,  # 1 = использовать текущую позицию
        0,
        0,
        0,
        0,
        0,
        0,
    )
    time.sleep(1)
    print_statustext()


def wait_for_ekf_ready(timeout=20):
    """Проверка сходимости EKF3"""
    print("[EKF] Проверка готовности системы позиционирования EKF3...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        msg = master.recv_match(
            type="EKF_STATUS_REPORT", blocking=True, timeout=1
        )
        print_statustext()

        if msg:
            flags = msg.flags
            pos_horiz_ok = bool(flags & (16 | 32))
            pos_vert_ok = bool(flags & 4)

            if pos_horiz_ok and pos_vert_ok:
                print(f"[EKF] EKF готов к полету! (Флаги: {flags})")
                return True

        time.sleep(0.5)

    print("⚠️ [EKF] Превышено время ожидания EKF! Пробуем продолжать...")
    return False


def set_horizontal_speed(speed_mps):
    """Установка скорости горизонтального перемещения"""
    print(f"[SPEED] Установка скорости: {speed_mps} м/с...")
    master.mav.command_long_send(
        get_target_sys(),
        get_target_comp(),
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        1,  # 1 = Groundspeed
        speed_mps,  # Скорость в м/с
        -1,  # Throttle (без изменений)
        0,
        0,
        0,
        0,  # Зарезервировано
    )


def send_cmd_with_ack(command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, timeout=3):
    """Отправка команды с гарантированной проверкой ACK"""
    target_sys = get_target_sys()
    target_comp = get_target_comp()

    # Очищаем старые ACK
    while master.recv_match(type="COMMAND_ACK", blocking=False):
        pass

    master.mav.command_long_send(
        target_sys, target_comp, command, 0, p1, p2, p3, p4, p5, p6, p7
    )

    start_time = time.time()
    while time.time() - start_time < timeout:
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack and ack.command == command:
            return ack.result

    return None


def arm_vehicle(timeout=15):
    """Арминг моторов"""
    print("[ARM] Отправка команды ARM...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        master.wait_heartbeat(timeout=1)

        if master.motors_armed():
            print("\n>>> [SUCCESS] ДРОН ЗААРМЛЕН! <<<")
            return True

        master.mav.command_long_send(
            get_target_sys(),
            get_target_comp(),
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
        print_statustext()
        time.sleep(1)

    print("\n [ERROR] Ошибка арминга!")
    return False


def disarm_immediately():
    """Аварийный дисарм"""
    print("\n [EMERGENCY DISARM] ВЫКЛЮЧЕНИЕ МОТОРОВ! ")
    for _ in range(5):
        master.mav.command_long_send(
            get_target_sys(),
            get_target_comp(),
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
        time.sleep(0.05)


def safe_takeoff_with_ack(target_alt=1.0):
    """Безопасный взлет"""
    print(f"[TAKEOFF] Отправка команды взлета на {target_alt} м...")

    res = send_cmd_with_ack(
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=target_alt, timeout=5
    )

    if res == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("[TAKEOFF] Команда ПРИНЯТА ArduPilot! Моторы набирают обороты...")
    elif res is not None:
        print(f" [TAKEOFF REJECTED] Отклонено! Код ответа: {res}")
        print_statustext()
        return False
    else:
        print(" [TAKEOFF ERROR] Таймаут! ArduPilot не прислал ACK.")
        print_statustext()
        # return False

    print("[MONITOR] Контроль скорости подъема...")
    start_time = time.time()

    while time.time() - start_time < 8:
        pos_msg = master.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=0.2
        )
        print_statustext()

        if pos_msg:
            current_alt = -pos_msg.z
            vz = -pos_msg.vz
            print(
                f"  Высота: {current_alt:.2f} м | Скорость подъема: {vz:.2f} м/с"
            )

            if vz > MAX_CLIMB_RATE:
                print(f" [SAFETY TRIP] Превышение скорости подъема ({vz:.2f} м/с)!")
                disarm_immediately()
                return False

            if current_alt >= (target_alt - 0.15):
                print(f" [TAKEOFF] Высота {target_alt} м успешно достигнута!")
                return True

        time.sleep(0.1)

    return True


def move_relative(dx, dy, dz, duration=4):
    """Смещение относительно носа дрона"""
    type_mask = 0b0000101111000000  # Игнорируем скорости/ускорения

    start_time = time.time()
    while time.time() - start_time < duration:
        master.mav.set_position_target_local_ned_send(
            0,
            get_target_sys(),
            get_target_comp(),
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
        time.sleep(0.2)


def land_and_disarm():
    """Посадка"""
    print("\n[LAND] Выполняем посадку (режим LAND)...")
    set_mode("LAND")

    start_time = time.time()
    while master.motors_armed() and (time.time() - start_time < 15):
        master.wait_heartbeat(timeout=1)
        time.sleep(0.5)

    print(">>> [SUCCESS] ДРОН УСПЕШНО СЕЛ И ДИСАРМЛЕН! <<<")


# ==============================================================================
# ОСНОВНОЙ СЦЕНАРИЙ
# ==============================================================================
if __name__ == "__main__":
    print_statustext()

    if not set_mode("GUIDED"):
        sys.exit(1)

    set_home_position()
    wait_for_ekf_ready()

    if arm_vehicle():
        set_horizontal_speed(MOVE_SPEED)

        if safe_takeoff_with_ack(target_alt=TARGET_ALTITUDE):
            print("\n--- НАЧАЛО АВТОНОМНЫХ МАНЕВРОВ ---")

            print("\n[MOVE] Движение на 1.0 метр ВПЕРЕД...")
            move_relative(dx=1.0, dy=0.0, dz=0.0, duration=4)

            print("\n[MOVE] Движение на 0.5 метра ВПРАВО...")
            move_relative(dx=0.0, dy=0.5, dz=0.0, duration=3)

            time.sleep(2)

        land_and_disarm()