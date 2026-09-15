import io
import time
import cv2
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from picamera2 import Picamera2

class MarkerDataStore:
    def __init__(self):
        self.detected_ids = []
        self.lock = threading.Lock()

    def update_ids(self, new_ids):
        with self.lock:
            if set(self.detected_ids) != set(new_ids):
                if new_ids:
                    print(f"[INFO SERVER] Метки: {new_ids}")
                else:
                    print("[INFO SERVER] Метки потеряны")
            self.detected_ids = new_ids

    def get_json(self):
        with self.lock:
            return json.dumps({
                "timestamp": time.time(),
                "count": len(self.detected_ids),
                "markers": self.detected_ids
            })

marker_store = MarkerDataStore()

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

class StreamingOutput:
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, frame):
        with self.condition:
            self.frame = frame
            self.condition.notify_all()

output = StreamingOutput()

class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ('/', '/index.html', '/stream'):
            content = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>Pi Zero 2 W - ArUco Fast</title>
                <style>
                    body { background: #121212; color: #fff; text-align: center; font-family: monospace; }
                    img { max-width: 100%; border: 2px solid #00ff88; border-radius: 8px; }
                    #info { margin-top: 15px; font-size: 1.2rem; color: #00ff88; }
                </style>
            </head>
            <body>
                <h2>Optimized ArUco Detector</h2>
                <img src="/video.mjpg" width="640" height="480" />
                <div id="info">Ожидание данных...</div>
                <script>
                    setInterval(async () => {
                        try {
                            let res = await fetch('/api/markers');
                            let data = await res.json();
                            document.getElementById('info').innerText =
                                `Найденные метки: [ ${data.markers.join(', ')} ]`;
                        } catch(e) {}
                    }, 500);
                </script>
            </body>
            </html>
            """.encode('utf-8')

            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        elif self.path == '/api/markers':
            json_data = marker_store.get_json().encode('utf-8')
            self.send_response(200)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(json_data)))
            self.end_headers()
            self.wfile.write(json_data)

        elif self.path == '/video.mjpg':
            self.send_response(200)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()

            try:
                while True:
                    with output.condition:
                        if not output.condition.wait(timeout=1.0):
                            continue
                        frame = output.frame

                    if frame is None:
                        continue

                    # Сжатие с качеством 50 вместо 60 — дает дополнительную экономию CPU
                    ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
                    if not ret:
                        continue

                    jpeg_bytes = jpeg.tobytes()
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(jpeg_bytes)))
                    self.end_headers()
                    self.wfile.write(jpeg_bytes)
                    self.wfile.write(b'\r\n')
            except Exception:
                pass
        else:
            self.send_error(404)
            self.end_headers()

def camera_worker():
    print("[CAM] Инициализация оптимизированной камеры...")
    picam2 = Picamera2()

    # 640x480 — хорошая картинка для стрима
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(config)
    picam2.start()

    # Быстрые параметры ArUco
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()

        # --- ОПТИМИЗАЦИЯ ПАРАМЕТРОВ ДЕТЕКТОРА ---
        aruco_params.adaptiveThreshWinSizeStep = 10 # Увеличиваем шаг окна (по умолчанию 3)
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE # Отключаем субпиксельное уточнение

        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        legacy_mode = False
    except AttributeError:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters_create()
        aruco_params.adaptiveThreshWinSizeStep = 10
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        legacy_mode = True

    frame_count = 0
    last_corners = []
    last_ids = []

    while True:
        try:
            frame = picam2.capture_array()
            frame_count += 1

            # ДЕТЕКЦИЯ ВЫПОЛНЯЕТСЯ ТОЛЬКО КАЖДЫЙ 3-й КАДР
            if frame_count % 3 == 0:
                # 1. Сжимаем кадр до 320x240 для быстрого сканирования
                small_frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_NEAREST)
                gray_small = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)

                # 2. Поиск меток на уменьшенном кадре
                if not legacy_mode:
                    corners, ids, _ = detector.detectMarkers(gray_small)
                else:
                    corners, ids, _ = cv2.aruco.detectMarkers(gray_small, aruco_dict, parameters=aruco_params)

                current_ids = []
                if ids is not None:
                    current_ids = [int(x) for x in ids.flatten()]
                    # Умножаем координаты обратно на 2 для масштаба 640x480
                    last_corners = [c * 2.0 for c in corners]
                    last_ids = ids
                else:
                    last_corners = []
                    last_ids = None

                marker_store.update_ids(current_ids)

            # Отрисовка меток на оригинальном кадре (каждый кадр для плавности)
            if last_ids is not None and len(last_corners) > 0:
                cv2.aruco.drawDetectedMarkers(frame, last_corners, last_ids)
                for i, marker_id in enumerate(last_ids.flatten()):
                    c = last_corners[i][0]
                    cx, cy = int((c[0][0] + c[2][0]) / 2), int((c[0][1] + c[2][1]) / 2)
                    cv2.putText(frame, f"ID: {marker_id}", (cx - 20, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            output.write(frame)

            # Небольшой микро-отдых для CPU
            time.sleep(0.005)

        except Exception as e:
            print(f"[CAM ERROR] {e}")
            time.sleep(0.05)

    picam2.stop()

def main():
    cam_thread = threading.Thread(target=camera_worker, daemon=True)
    cam_thread.start()

    server_address = ('', 8080)
    httpd = ThreadedHTTPServer(server_address, StreamingHandler)
    print("[HTTP] Оптимизированный сервер запущен на 8080...")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()

if __name__ == '__main__':
    main()