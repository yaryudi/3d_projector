import cv2
from cvzone.HandTrackingModule import HandDetector
import socket
import numpy as np
import time
import math
from collections import deque

#유니티에 손동작 인식 결과 데이터를 보내는 모듈

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

# ===============================
# PARAMETERS
# ===============================
cooldown_period = 1.0
last_gesture_time = 0

# ---- SWIPE ----
swipe_buffer = deque(maxlen=20)
swipe_window = 0.4
min_swipe_dist = 200
min_swipe_speed = 500
min_swipe_speed_y = 400
max_vertical_ratio = 0.7

# ---- ZOOM & ROTATE 안정화용 버퍼 ----
zoom_buffer = deque(maxlen=10)
rotate_buffer = deque(maxlen=15)

zoom_threshold = 40
zoom_accum_threshold = 100
rotate_accum_threshold = math.radians(35)

# ===============================
# 한손: SWIPE (좌/우 + 상/하)
# ===============================
def check_swipe_onehand(now):
    if len(swipe_buffer) < 5:
        return None
    t_now, x_now, y_now = swipe_buffer[-1]
    t_min = now - swipe_window
    valid = [(t, x, y) for (t, x, y) in swipe_buffer if t >= t_min]
    if len(valid) < 3:
        return None

    t0, x0, y0 = valid[0]
    dt = t_now - t0
    if dt <= 0:
        return None

    dx = x_now - x0
    dy = y_now - y0
    speed_x = abs(dx) / dt
    speed_y = abs(dy) / dt

    # ---- 가로 스와이프 ----
    if abs(dx) >= min_swipe_dist and speed_x >= min_swipe_speed and abs(dy) <= abs(dx) * max_vertical_ratio:
        return "left swipe" if dx > 0 else "right swipe"

    # ---- 세로 스와이프 ----
    if abs(dy) >= min_swipe_dist and speed_y >= min_swipe_speed_y and abs(dx) <= abs(dy) * max_vertical_ratio:
        return "down swipe" if dy > 0 else "up swipe"

    return None


# ===============================
# MAIN LOOP
# ===============================
while True:
    success, img = cap.read()
    if not success:
        break

    hands, img = detector.findHands(img)
    gesture = "None"
    now = time.time()

    # === 전역 쿨다운 ===
    if now - last_gesture_time < cooldown_period:
        remaining = cooldown_period - (now - last_gesture_time)
        cv2.putText(img, f"Cooldown: {remaining:.1f}s", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.imshow("Gesture Control", cv2.resize(img, (640, 360)))
        if cv2.waitKey(1) == ord('q'):
            break
        continue

    # === 랜드마크 송신 (첫 번째 손 기준, 기존 수신부 호환) ===
    if hands:
        h0 = hands[0]
        lmList = h0["lmList"]
        data = [coord for lm in lmList for coord in (lm[0], height - lm[1], lm[2])]
        sock.sendto(str.encode(str(data)), serverAddressPort)

    # ======================================
    # 2손 모드 → ZOOM 또는 ROTATE
    # ======================================
    if hands and len(hands) == 2:
        h1, h2 = hands[0], hands[1]
        c1 = np.array(h1["center"], dtype=float)
        c2 = np.array(h2["center"], dtype=float)

        fingers1 = detector.fingersUp(h1)
        fingers2 = detector.fingersUp(h2)

        # 조건 확인
        both_open = fingers1 == [1,1,1,1,1] and fingers2 == [1,1,1,1,1]
        both_fist = fingers1 == [0,0,0,0,0] and fingers2 == [0,0,0,0,0]

        # ---- 1️⃣ 줌 (양손 펴짐) ----
        if both_open:
            dist_hands = np.linalg.norm(c2 - c1)
            zoom_buffer.append(dist_hands)

            if len(zoom_buffer) >= 5:
                diff = zoom_buffer[-1] - zoom_buffer[0]
                mean_diff = np.mean(np.diff(zoom_buffer))
                if abs(mean_diff) < zoom_threshold / 10:
                    mean_diff = 0
                if abs(diff) > zoom_accum_threshold:
                    gesture = "zoom in" if diff > 0 else "zoom out"
                    last_gesture_time = now
                    zoom_buffer.clear()
                    rotate_buffer.clear()
                    print(f"{gesture} (two-hand open palm zoom)")

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
                    print(f"{gesture} (two-fist rotation, {math.degrees(total_angle):.1f}°)")

        # ---- 송신 ----
        sock.sendto(str.encode(gesture), serverAddressPort)
        cv2.putText(img, f"Two-hand mode", (50, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

    # ======================================
    # 1손 모드 → SWIPE (좌/우/상/하)
    # ======================================
    elif hands and len(hands) == 1:
        hand = hands[0]
        cx, cy = hand["center"]
        swipe_buffer.append((now, cx, cy))

        swipe = check_swipe_onehand(now)
        if swipe:
            gesture = swipe
            last_gesture_time = now
            swipe_buffer.clear()
            rotate_buffer.clear()
            zoom_buffer.clear()
            print(gesture)

        sock.sendto(str.encode(gesture), serverAddressPort)
        cv2.putText(img, f"One-hand mode", (50, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 255, 200), 2)

    # === 표시 ===
    if gesture != "None":
        cv2.putText(img, gesture, (50, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)

    cv2.imshow("Gesture Control", cv2.resize(img, (640, 360)))
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
