import sys, time
import serial
import numpy as np
from collections import deque
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg

#아두이노가 읽어주는 신호를 오실로스코프처럼 표기함.

# ===== 설정 =====
PORT = 'COM9'
BAUD = 230400          # ← 아두이노와 동일
TIME_WINDOW_SEC = 0.5  # 화면에 보일 시간 구간(초) - 오실로스코프 time/div 느낌
TARGET_FPS = 120       # 목표 갱신 속도
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

class Scope(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Arduino Oscilloscope @120Hz (PyQtGraph)")
        self.resize(1000, 520)

        # 시리얼
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        except Exception as e:
            pg.QtWidgets.QMessageBox.critical(self, "Serial Error", str(e))
            sys.exit(1)

        # 데이터(값/시간) 버퍼
        self.vals = deque()
        self.ts   = deque()  # 각 샘플의 타임스탬프(초)

        # 샘플링레이트 추정 (EMA)
        self.sps_est = 5000.0  # 초기 추정치(대략값), 자동으로 수렴
        self.ema_alpha = 0.2
        self.last_frame_t = time.perf_counter()

        # 그래프
        self.plot = pg.PlotWidget(background="w")
        self.setCentralWidget(self.plot)
        self.curve = self.plot.plot(pen=pg.mkPen(width=1))
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel('left', 'Voltage (V)')
        self.plot.setLabel('bottom', 'Time (s)')
        self.plot.setYRange(0, 5)

        # 120 Hz 타이머
        self.timer = QtCore.QTimer()
        self.timer.setInterval(TIMER_MS)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    def update_plot(self):
        t_now = time.perf_counter()

        # ── 1) 시리얼 버퍼 한 번에 비우기 (배치 읽기) ──
        waiting = self.ser.in_waiting
        if waiting > 0:
            buf = self.ser.read(waiting)
            ints = parse_int_lines(buf)

            # 프레임 소요시간과 추가 샘플수로 SPS 추정 (EMA)
            frame_dt = t_now - self.last_frame_t
            if frame_dt > 0 and ints:
                inst_sps = len(ints) / frame_dt
                self.sps_est = (1 - self.ema_alpha) * self.sps_est + self.ema_alpha * inst_sps

            # 새 샘플에 시간 할당: 이번 프레임 시간 구간을 균등분배
            if ints:
                # 이번 프레임 시작~끝을 균등 분할하여 각 샘플 타임스탬프 부여
                # 더 정교하게 하려면 샘플 도착 간격을 추정해도 됨
                if len(ints) == 1:
                    ts_new = [t_now]
                else:
                    ts_new = np.linspace(self.last_frame_t, t_now, num=len(ints), endpoint=True)

                for v, t in zip(ints, ts_new):
                    volt = max(0.0, min(5.0, v * VPERCOUNT))
                    self.vals.append(volt)
                    self.ts.append(t)

        self.last_frame_t = t_now

        # ── 2) TIME_WINDOW_SEC 밖의 오래된 샘플 제거 ──
        cutoff = t_now - TIME_WINDOW_SEC
        while self.ts and self.ts[0] < cutoff:
            self.ts.popleft()
            self.vals.popleft()

        # ── 3) 플롯 업데이트 ──
        if self.ts:
            t0 = self.ts[0]
            x = np.fromiter((tt - t0 for tt in self.ts), dtype=float)
            y = np.fromiter(self.vals, dtype=float)
            # x축을 0~TIME_WINDOW_SEC로 고정 (오실로스코프 느낌)
            self.plot.setXRange(0, max(TIME_WINDOW_SEC, x[-1] if len(x) else TIME_WINDOW_SEC), padding=0)
            self.curve.setData(x, y)

        # (옵션) 윈도우 타이틀에 추정 SPS/프레임률 표시
        # 실제 UI FPS는 QTimer 간격이 결정, 시리얼 처리량은 sps_est로 확인
        self.setWindowTitle(f"Arduino Oscilloscope @{TARGET_FPS}Hz  |  ~{int(self.sps_est)} SPS")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = Scope()
    win.show()
    sys.exit(app.exec())