import cv2

# Загрузка
fs = cv2.FileStorage("camera.yaml", cv2.FILE_STORAGE_READ)
mtx = fs.getNode("camera_matrix").mat()
dist = fs.getNode("distortion_coefficients").mat()
fs.release()

img = cv2.imread("photos/photo_20260921_145109.jpg")
undistorted_img = cv2.undistort(img, mtx, dist, None, mtx)

cv2.imshow("Original vs Undistorted", cv2.hconcat([img, undistorted_img]))
cv2.waitKey(0)