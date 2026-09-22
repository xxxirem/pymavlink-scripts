import glob
import os
import cv2
import numpy as np

# --- НАСТРОЙКИ ---
# Указываем количество ВНУТРЕННИХ углов сетки (пересечений линий)
# Если на экране шахматка 9x7 клеток, то внутренних углов будет 8x6.
BOARD_SIZE = (8, 5)

# Размер одного квадрата шахматки на экране телефона в миллиметрах или метрах.
# Для калибровки пропорций это не критично (можно оставить 1.0),
# но для реального измерения расстояний укажите точный размер (например, 15 мм = 0.015 м).
SQUARE_SIZE = 0.010

# Путь к изображениям
IMAGES_PATH = "./photos/*.jpg"

# Критерии субпиксельной уточняющей аппроксимации углов
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Подготовка 3D-точек шаблона (0,0,0), (1,0,0), (2,0,0) ...
objp = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0 : BOARD_SIZE[0], 0 : BOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# Массивы для хранения 3D-точек объекта и 2D-точек изображения
objpoints = []  # 3D точки в реальном мире
imgpoints = []  # 2D точки на плоскости изображения

images = glob.glob(IMAGES_PATH)
if not images:
    print("Изображения не найдены! Проверьте путь.")
    exit()

img_size = None

print(f"Обработка {len(images)} снимков...")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_size = gray.shape[::-1]

    # Поиск углов шахматной доски
    ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)

    if ret:
        objpoints.append(objp)

        # Уточнение координат углов с субпиксельной точностью
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        print(f"[+] Углы найдены: {os.path.basename(fname)}")
    else:
        print(f"[-] Не удалось найти углы: {os.path.basename(fname)}")

if len(objpoints) < 5:
    print(
        "Ошибка: Слишком мало успешных кадров для калибровки (нужно минимум 5-10)."
    )
    exit()

# Калибровка камеры
print("\nВычисление параметров калибровки...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_size, None, None
)

print(f"Ошибка репроекции (RMS Error): {ret:.4f} px (отлично, если < 0.5)")

# Сохранение результатов
np.savez("calibration_data.npz", mtx=mtx, dist=dist)
print("Результаты сохранены в файл 'calibration_data.npz'")