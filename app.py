# -*- coding: utf-8 -*-
import cv2
import time
import json
import threading
import RPi.GPIO as GPIO
import base64
import os
from flask import Flask, render_template, Response
from flask_sock import Sock

# --- CẤU HÌNH ---
CAMERA_INDEX = 0
CONFIG_FILE = 'config.json'

# --- CẤU HÌNH GPIO ---
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
RELAY_PINS = { 
    0: {'push': 11, 'pull': 12},
    1: {'push': 13, 'pull': 8},
    2: {'push': 15, 'pull': 7}
}
SENSOR_PINS = { 0: 5, 1: 29, 2: 31 }

for pins in RELAY_PINS.values():
    GPIO.setup(pins['push'], GPIO.OUT)
    GPIO.setup(pins['pull'], GPIO.OUT)
for pin in SENSOR_PINS.values(): 
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# --- BIẾN TRẠNG THÁI TRUNG TÂM ---
system_state = {
    "lanes": [
        {"name": "Loại 1", "status": "Sẵn sàng", "count": 0},
        {"name": "Loại 2", "status": "Sẵn sàng", "count": 0},
        {"name": "Loại 3", "status": "Sẵn sàng", "count": 0}
    ],
    "timing_config": {"cycle_delay": 0.3}
}
state_lock = threading.Lock()
main_loop_running = True
latest_frame = None
frame_lock = threading.Lock()

# --- HÀM LƯU/TẢI CẤU HÌNH CỤC BỘ ---
def load_local_config():
    global system_state
    default_delay = 0.3
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                config = json.load(f)
                delay = config.get('timing_config', {}).get('cycle_delay', default_delay)
                with state_lock:
                    system_state['timing_config']['cycle_delay'] = delay
                print(f"Loaded local config: cycle_delay = {delay}")
        except (json.JSONDecodeError, IOError):
            print(f"Error reading {CONFIG_FILE}, using default {default_delay}s.")
            with state_lock: system_state['timing_config']['cycle_delay'] = default_delay
    else:
        print(f"{CONFIG_FILE} not found, using default {default_delay}s.")
        with state_lock: system_state['timing_config']['cycle_delay'] = default_delay

# --- HÀM ĐIỀU KHIỂN ---
def reset_all_relays_to_default():
    print("[GPIO] Resetting all relays to default state (PULL ON).")
    for lane_pins in RELAY_PINS.values():
        GPIO.output(lane_pins['push'], GPIO.HIGH)
        GPIO.output(lane_pins['pull'], GPIO.LOW)

# --- CÁC LUỒNG XỬ LÝ SONG SONG ---
def camera_capture_thread():
    global latest_frame
    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        print("[ERROR] Cannot open camera.")
        return
    while main_loop_running:
        ret, frame = camera.read()
        if ret:
            with frame_lock:
                latest_frame = frame.copy()
        time.sleep(1/30) # Chụp ở ~30 FPS
    camera.release()

def sorting_process(lane_index):
    if system_state["lanes"][lane_index]["status"] not in ["Sẵn sàng", "Đang chờ vật..."]: return
    with state_lock:
        delay = system_state['timing_config']['cycle_delay']
        log_name = system_state['lanes'][lane_index]['name']
        system_state["lanes"][lane_index]["status"] = "Đang phân loại..."
    broadcast_log({"log_type": "info", "message": f"Bắt đầu chu trình cho {log_name}"})
    try:
        pull_pin, push_pin = RELAY_PINS[lane_index]['pull'], RELAY_PINS[lane_index]['push']
        GPIO.output(pull_pin, GPIO.HIGH); time.sleep(0.2)
        GPIO.output(push_pin, GPIO.LOW); time.sleep(delay)
        GPIO.output(push_pin, GPIO.HIGH); time.sleep(0.2)
        GPIO.output(pull_pin, GPIO.LOW)
    finally:
        with state_lock:
            lane_info = system_state["lanes"][lane_index]
            lane_info["status"] = "Sẵn sàng"
            lane_info["count"] += 1
            broadcast_log({"log_type": "sort", "name": log_name, "count": lane_info['count']})
    broadcast_log({"log_type": "info", "message": f"Hoàn tất chu trình cho {log_name}"})

def qr_detection_loop():
    detector = cv2.QRCodeDetector()
    last_qr_data, last_qr_time = "", 0
    LANE_MAP = {"LOAI1": 0, "LOAI2": 1, "LOAI3": 2}
    while main_loop_running:
        current_frame = None
        with frame_lock:
            if latest_frame is not None:
                current_frame = latest_frame.copy()
        if current_frame is None:
            time.sleep(0.2); continue
        try:
            data, _, _ = detector.detectAndDecode(current_frame)
        except cv2.error:
            data = None; time.sleep(0.25); continue
        
        if data and (data != last_qr_data or time.time() - last_qr_time > 3):
            last_qr_data, last_qr_time = data, time.time()
            data_upper = data.upper().strip()
            if data_upper in LANE_MAP:
                lane_index = LANE_MAP[data_upper]
                if system_state["lanes"][lane_index]["status"] == "Sẵn sàng":
                    broadcast_log({"log_type": "qr", "data": data_upper})
                    with state_lock: system_state["lanes"][lane_index]["status"] = "Đang chờ vật..."
                    timeout = time.time() + 15
                    while time.time() < timeout:
                        if GPIO.input(SENSOR_PINS[lane_index]) == 0:
                            threading.Thread(target=sorting_process, args=(lane_index,), daemon=True).start()
                            break
                        time.sleep(0.05)
                    else:
                        with state_lock: system_state["lanes"][lane_index]["status"] = "Sẵn sàng"
            elif data_upper == "NG": broadcast_log({"log_type": "ng_product", "data": data_upper})
            else: broadcast_log({"log_type": "unknown_qr", "data": data_upper})
        time.sleep(0.25)

# --- FLASK WEB SERVER ---
app = Flask(__name__)
sock = Sock(app)
connected_clients = set()

def broadcast_state():
    while main_loop_running:
        with state_lock:
            # Lấy trạng thái GPIO real-time
            for i in range(3):
                system_state['lanes'][i]['sensor'] = GPIO.input(SENSOR_PINS[i])
                system_state['lanes'][i]['relay_grab'] = 1 if GPIO.input(RELAY_PINS[i]['pull']) == GPIO.LOW else 0
                system_state['lanes'][i]['relay_push'] = 1 if GPIO.input(RELAY_PINS[i]['push']) == GPIO.LOW else 0
            
            msg = json.dumps({"type": "state_update", "state": system_state})
            for client in list(connected_clients):
                try: client.send(msg)
                except: connected_clients.remove(client)
        time.sleep(0.5)

def broadcast_log(log_data):
    log_data['timestamp'] = time.strftime('%H:%M:%S')
    msg = json.dumps({"type": "log", **log_data})
    for client in list(connected_clients):
        try: client.send(msg)
        except: connected_clients.remove(client)

def generate_frames():
    while main_loop_running:
        with frame_lock:
            if latest_frame is None:
                time.sleep(0.1)
                continue
            frame = latest_frame.copy()
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(1/20) # Stream ở 20 FPS

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@sock.route('/ws')
def ws_route(ws):
    connected_clients.add(ws)
    try:
        while True:
            # Nhận lệnh từ client nếu cần (hiện tại chưa dùng)
            message = ws.receive() 
    finally:
        connected_clients.remove(ws)

# --- MAIN ---
if __name__ == "__main__":
    try:
        load_local_config()
        reset_all_relays_to_default()
        threading.Thread(target=camera_capture_thread, daemon=True).start()
        threading.Thread(target=broadcast_state, daemon=True).start()
        threading.Thread(target=qr_detection_loop, daemon=True).start()
        
        print("Starting Flask server at http://0.0.0.0:5000")
        app.run(host='0.0.0.0', port=5000)

    except KeyboardInterrupt: 
        print("\n🛑 Shutting down...")
    finally: 
        main_loop_running = False
        time.sleep(0.5)
        GPIO.cleanup()
        print("GPIO cleaned up.")
