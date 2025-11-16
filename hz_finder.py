import sys, time
import serial
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ===== 설정 =====
PORT = 'COM7'
BAUD = 230400          # ← 아두이노와 동일
TIME_WINDOW_SEC = 0.5  # 화면에 보일 시간 구간(초)
TARGET_FPS = 120       # 목표 갱신 속도 (Matplotlib에서는 이 속도보다 느릴 수 있음)
TIMER_MS = max(1, int(1000 / TARGET_FPS))
TIMEOUT = 0            # 논블로킹
VPERCOUNT = 5.0 / 1023.0

# ===== 유틸 =====
def parse_int_lines(raw_bytes):
    """b'123\\n456\\n' → [123, 456] 빠르게 파싱 (에러 라인은 무시)"""
    lines = raw_bytes.split(b'\n')
    out = []
    for ln in lines:
        if not ln:
            continue
        try:
            out.append(int(ln.strip()))
        except:
            pass
    return out

# ===== 시리얼 및 데이터 버퍼 초기화 =====
try:
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
except Exception as e:
    print(f"Serial Error: {e}")
    sys.exit(1)

# 데이터(값/시간) 버퍼
vals = deque()
ts = deque()  # 각 샘플의 타임스탬프(초)

# 샘플링레이트 추정 (EMA)
sps_est = 5000.0  # 초기 추정치(대략값), 자동으로 수렴
ema_alpha = 0.2
last_frame_t = time.perf_counter()

# ===== Matplotlib 그래프 설정 =====
fig, ax = plt.subplots()
line, = ax.plot([], [], '-')  # 초기 빈 플롯
ax.set_ylim(0, 5)
ax.set_xlim(0, TIME_WINDOW_SEC)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Voltage (V)')
ax.grid(True)
fig.suptitle("Arduino Oscilloscope (Matplotlib)")

# ===== 업데이트 함수 =====
def update_plot(frame):
    global last_frame_t, sps_est  # 전역 변수 수정 명시

    t_now = time.perf_counter()

    # ── 1) 시리얼 버퍼 한 번에 비우기 (배치 읽기) ──
    waiting = ser.in_waiting
    if waiting > 0:
        buf = ser.read(waiting)
        ints = parse_int_lines(buf)

        # 프레임 소요시간과 추가 샘플수로 SPS 추정 (EMA)
        frame_dt = t_now - last_frame_t
        if frame_dt > 0 and ints:
            inst_sps = len(ints) / frame_dt
            sps_est = (1 - ema_alpha) * sps_est + ema_alpha * inst_sps

        # 새 샘플에 시간 할당
        if ints:
            if len(ints) == 1:
                ts_new = [t_now]
            else:
                ts_new = np.linspace(last_frame_t, t_now, num=len(ints), endpoint=True)

            for v, t in zip(ints, ts_new):
                volt = max(0.0, min(5.0, v * VPERCOUNT))
                vals.append(volt)
                ts.append(t)

    last_frame_t = t_now

    # ── 2) TIME_WINDOW_SEC 밖의 오래된 샘플 제거 ──
    cutoff = t_now - TIME_WINDOW_SEC
    while ts and ts[0] < cutoff:
        ts.popleft()
        vals.popleft()

    # ── 3) 플롯 업데이트 ──
    if ts:
        t0 = ts[0]
        # 리스트/deque를 numpy 배열로 변환
        x_data = np.fromiter((tt - t0 for tt in ts), dtype=float)
        y_data = np.fromiter(vals, dtype=float)
        
        # 데이터 설정
        line.set_data(x_data, y_data)
        
        # X축 범위 고정 (오실로스코프 느낌)
        if len(x_data) > 0:
             ax.set_xlim(0, max(TIME_WINDOW_SEC, x_data[-1]))
        else:
             ax.set_xlim(0, TIME_WINDOW_SEC)

    # 윈도우 타이틀에 추정 SPS 표시 (Matplotlib에서는 suptitle 사용)
    fig.suptitle(f"Arduino Oscilloscope (Matplotlib) | ~{int(sps_est)} SPS")

    return line,  # blit=True를 위해 수정된 아티스트 반환

# ===== 애니메이션 실행 =====
# blit=True는 성능 향상에 도움을 줍니다.
ani = animation.FuncAnimation(fig, update_plot, interval=TIMER_MS, blit=True, cache_frame_data=False)

# 창을 닫을 때 시리얼 포트 닫기
def on_close(event):
    print("Closing port...")
    ser.close()

fig.canvas.mpl_connect('close_event', on_close)

plt.show()
print("Plot window closed.")