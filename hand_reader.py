import cv2
from cvzone.HandTrackingModule import HandDetector
import socket
import numpy as np
import time
import math
from collections import deque
import serial 

# ===============================
# 1. 아두이노 연결
# ===============================
serial_port = 'COM7' 
baud_rate = 115200
arduino = None

print("=== System Starting ===")
try:
    arduino = serial.Serial(serial_port, baud_rate, timeout=0.05)
    time.sleep(2) 
    arduino.reset_input_buffer()
    print(f"Connected to Arduino on {serial_port}")
except Exception as e:
    print(f"Arduino Connection Error: {e}")

def send_motor_command(cmd_type, value=0):
    if arduino:
        command = f"{cmd_type}{value}\n"
        arduino.write(command.encode())
        print(f"Tx -> Arduino: {command.strip()}")

# 초기 상태
current_speed = 0    
current_accel = 400
is_enabled = True   
current_pos_step = 0 # 현재 스텝값 저장용

if arduino:
    send_motor_command('v', 0)
    send_motor_command('a', current_accel)
    send_motor_command('e') 

# ===============================
# CONFIG
# ===============================
width, height = 1280, 720
cap = cv2.VideoCapture(0)
cap.set(3, width)
cap.set(4, height)

detector = HandDetector(maxHands=2, detectionCon=0.8)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
serverAddressPort = ("127.0.0.1", 5052)

# 제스처 변수
swipe_buffer = deque(maxlen=20); zoom_buffer = deque(maxlen=10); rotate_buffer = deque(maxlen=15)
cooldown_period = 1.0; last_gesture_time = 0
min_swipe_dist = 200; min_swipe_speed = 500; 
zoom_accum_threshold = 100; rotate_accum_threshold = math.radians(35) # [복구] 임계값 재정의

def check_swipe_onehand(now):
    if len(swipe_buffer) < 5: return None
    t_now, x_now, y_now = swipe_buffer[-1]
    t_min = now - 0.4
    valid = [(t, x, y) for (t, x, y) in swipe_buffer if t >= t_min]
    if len(valid) < 3: return None
    t0, x0, y0 = valid[0]
    dt = t_now - t0
    if dt <= 0: return None
    dx = x_now - x0; dy = y_now - y0
    speed_x = abs(dx)/dt; speed_y = abs(dy)/dt
    if abs(dx) >= min_swipe_dist and speed_x >= min_swipe_speed and abs(dy) <= abs(dx)*0.7:
        return "left swipe" if dx > 0 else "right swipe"
    if abs(dy) >= min_swipe_dist and speed_y >= 400 and abs(dx) <= abs(dy)*0.7:
        return "down swipe" if dy > 0 else "up swipe"
    return None

# ===============================
# MAIN LOOP
# ===============================
while True:
    # --- [1] 아두이노 수신 ---
    if arduino and arduino.in_waiting > 0:
        try:
            while arduino.in_waiting:
                line = arduino.readline().decode('utf-8', errors='ignore').strip()
                if line == "1" or line == "2":
                    msg = f"SENSOR:{line}"
                    sock.sendto(str.encode(msg), serverAddressPort)
                    print(f"Unity Send >> {msg}")
                elif line.startswith("POS:"):
                    sock.sendto(str.encode(line), serverAddressPort)
                    try: current_pos_step = int(line.split(":")[1])
                    except: pass
        except: pass

    # --- [2] 비전 처리 ---
    success, img = cap.read()
    if not success: break
    hands, img = detector.findHands(img)
    gesture = "None"
    now = time.time()

    if now - last_gesture_time >= cooldown_period:
        if hands:
            h0 = hands[0]; lmList = h0["lmList"]
            data = [coord for lm in lmList for coord in (lm[0], height - lm[1], lm[2])]
            sock.sendto(str.encode(str(data)), serverAddressPort)
            
            # ====================================================
            # [복구됨] 양손 제스처 (ZOOM & ROTATE)
            # ====================================================
            if len(hands) == 2:
                h1, h2 = hands[0], hands[1]
                c1 = np.array(h1["center"], dtype=float)
                c2 = np.array(h2["center"], dtype=float)

                fingers1 = detector.fingersUp(h1)
                fingers2 = detector.fingersUp(h2)

                # 양손 펴짐 -> 줌
                both_open = fingers1 == [1,1,1,1,1] and fingers2 == [1,1,1,1,1]
                # 양손 주먹 -> 회전
                both_fist = fingers1 == [0,0,0,0,0] and fingers2 == [0,0,0,0,0]

                # ---- 1️⃣ 줌 (양손 펴짐) ----
                if both_open:
                    dist_hands = np.linalg.norm(c2 - c1)
                    zoom_buffer.append(dist_hands)

                    if len(zoom_buffer) >= 5:
                        diff = zoom_buffer[-1] - zoom_buffer[0]
                        if abs(diff) > zoom_accum_threshold:
                            gesture = "zoom in" if diff > 0 else "zoom out"
                            last_gesture_time = now
                            zoom_buffer.clear()
                            rotate_buffer.clear()
                            print(f"Gesture: {gesture}")
                            sock.sendto(str.encode(gesture), serverAddressPort)

                # ---- 2️⃣ 회전 (양손 주먹) ----
                elif both_fist:
                    v_now = c2 - c1
                    rotate_buffer.append(v_now)

                    if len(rotate_buffer) >= 5:
                        total_angle = 0
                        rb = list(rotate_buffer)
                        for v1, v2 in zip(rb[:-1], rb[1:]):
                            cross_z = v1[0]*v2[1] - v1[1]*v2[0]
                            dot = np.dot(v1, v2)
                            angle = math.atan2(cross_z, dot)
                            total_angle += angle

                        if abs(total_angle) > rotate_accum_threshold:
                            gesture = "rotate ccw" if total_angle > 0 else "rotate cw"
                            last_gesture_time = now
                            rotate_buffer.clear()
                            zoom_buffer.clear()
                            print(f"Gesture: {gesture}")
                            sock.sendto(str.encode(gesture), serverAddressPort)
                            
            # ====================================================
            # 한손 제스처 (SWIPE)
            # ====================================================
            elif len(hands) == 1:
                swipe_buffer.append((now, hands[0]["center"][0], hands[0]["center"][1]))
                swipe = check_swipe_onehand(now)
                if swipe:
                    gesture = swipe; last_gesture_time = now; swipe_buffer.clear()
                    print(f"Gesture: {gesture}")
                    sock.sendto(str.encode(gesture), serverAddressPort)

    # --- [3] 키보드 입력 제어 ---
    key = cv2.waitKey(1) & 0xFF 

    if key != 255:
        if key == ord('w'): 
            current_speed += 100
            send_motor_command('v', current_speed)
        elif key == ord('s'): 
            current_speed -= 100
            send_motor_command('v', current_speed)
        elif key == 32: # Space -> 정지
            current_speed = 0
            send_motor_command('v', 0)
        elif key == ord('e'): 
            is_enabled = True
            send_motor_command('e')
        elif key == ord('d'): 
            is_enabled = False
            send_motor_command('d')
        elif key == ord('q'):
            break

    # 화면 표시
    if current_speed > 0: status = "FORWARD (>>>)"
    elif current_speed < 0: status = "REVERSE (<<<)"
    else: status = "STOPPED"
    
    motor_color = (0, 255, 0) if is_enabled else (0, 0, 255)
    motor_state_txt = "ENABLED" if is_enabled else "DISABLED"

    cv2.putText(img, f"Speed: {current_speed}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(img, f"Status: {status}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 100, 255), 2)
    cv2.putText(img, f"Motor: {motor_state_txt}", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.8, motor_color, 2)
    cv2.putText(img, f"Step Pos: {current_pos_step}", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
    
    # 양손/한손 모드 표시
    mode_text = "Two Hands" if hands and len(hands) == 2 else "One Hand"
    cv2.putText(img, mode_text, (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    if gesture != "None":
        cv2.putText(img, gesture, (50, 180), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)

    cv2.imshow("Gesture Control", cv2.resize(img, (640, 360)))

cap.release()
cv2.destroyAllWindows()
if arduino: arduino.close()