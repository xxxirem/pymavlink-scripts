import cv2
import os
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__)) + '/'

COLS = 8
ROWS = 5

SQUARE_SIZE_PX = 200

MARGIN_PX = 100

width = COLS * SQUARE_SIZE_PX + 2 * MARGIN_PX
height = ROWS * SQUARE_SIZE_PX + 2 * MARGIN_PX

image = np.full((height, width), 255, dtype=np.uint8)

for r in range(ROWS):
    for c in range(COLS):
        if (r + c) % 2 == 1:
            x1 = MARGIN_PX + c * SQUARE_SIZE_PX
            y1 = MARGIN_PX + r * SQUARE_SIZE_PX
            x2 = x1 + SQUARE_SIZE_PX
            y2 = y1 + SQUARE_SIZE_PX

            image[y1:y2, x1:x2] = 0

output_filename = f"{BASE_DIR}chessboard_{COLS}x{ROWS}.png"
cv2.imwrite(output_filename, image)
print(f"Изображение успешно сохранено как '{output_filename}' ({width}x{height} px).")
