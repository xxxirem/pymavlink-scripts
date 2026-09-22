import glob
import os
import cv2
import numpy as np

# --- НАСТРОЙКИ ---
BOARD_SIZE = (7, 4)
SQUARE_SIZE = 0.010
IMAGES_PATH = "./photos/*.jpg"

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Подготовка 3D-точек шаблона
objp = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0 : BOARD_SIZE[0], 0 : BOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []

images = glob.glob(IMAGES_PATH)
if not images:
    print("Изображения не найдены!")
    exit()

img_size = None
print(f"Обработка {len(images)} снимков...")

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_size = gray.shape[::-1]

    # Основной детектор SB
    ret, corners = cv2.findChessboardCornersSB(
        gray,
        BOARD_SIZE,
        flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY
    )

    # Запасной классический метод
    if not ret:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, flags)
        if ret:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        print(f"[+] Углы найдены: {os.path.basename(fname)}")
    else:
        print(f"[-] Углы НЕ найдены: {os.path.basename(fname)}")

if len(objpoints) < 5:
    print("\nОшибка: Слишком мало успешных кадров для калибровки (нужно минимум 5).")
    exit()

# Вычисление параметров калибровки
print("\nВычисление параметров калибровки...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_size, None, None
)

print(f"Ошибка репроекции (RMS Error): {ret:.4f} px")

# Сохранение параметров в camera.yaml
fs = cv2.FileStorage("camera.yaml", cv2.FILE_STORAGE_WRITE)
fs.write("camera_matrix", mtx)
fs.write("distortion_coefficients", dist)
fs.write("image_width", img_size[0])
fs.write("image_height", img_size[1])
fs.write("rms_error", float(ret))
fs.release()

print("Результаты успешно сохранены в файл 'camera.yaml'")