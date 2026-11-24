#!/usr/bin/env python3
"""Ghi log UART cua mach AGV (CH340, 460800) va bao su kien.

- Moi dong mach gui ve duoc luu vao logs/uart_YYYYMMDD.log (co gio).
- Gui lenh cho mach: echo 'm 3 40' > /tmp/agv_cmd  (FIFO)
- In ra stdout chi nhung su kien dang chu y (mach bat, mat tin hieu, lenh xong, loi, pin yeu).
Chay: python3 uart_logger.py [/dev/ttyUSBx]   (mac dinh tu tim)
"""
import glob, os, re, select, sys, termios, time

PORT = sys.argv[1] if len(sys.argv) > 1 else 'auto'   # auto = cong /dev/ttyUSB* dau tien
FIFO = '/tmp/agv_cmd'
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'logs')
SILENT_S = 3.0       # khong nhan gi qua thoi gian nay (khi dang in dinh ky) -> bao mat tin hieu
BATT_LOW_V = 11.3

PORT_NOW = PORT

EVENT_PATTERNS = [
    (re.compile(r'WHO_AM_I'), 'MACH VUA KHOI DONG'),
    (re.compile(r'^(xong|nang ha|khong chay|sai)'), None),   # khong bao 'chay ...' de do on
    (re.compile(r'IMU loi'), 'IMU LOI'),
]


def emit(msg):
    print(time.strftime('%H:%M:%S'), msg, flush=True)


def open_port():
    global PORT_NOW
    ports = sorted(glob.glob('/dev/ttyUSB*')) if PORT == 'auto' else [PORT]
    if not ports:
        raise OSError('khong co cong')
    PORT_NOW = ports[0]
    fd = os.open(PORT_NOW, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0; a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0
    a[4] = a[5] = termios.B460800
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    if not os.path.exists(FIFO):
        os.mkfifo(FIFO)
    cmd_fd = os.open(FIFO, os.O_RDWR | os.O_NONBLOCK)   # RDWR: nguoi ghi khong bi chan
    fd, buf, cmd_buf = None, b'', b''
    last_rx, alive, imu_bad, batt_warned = 0.0, False, False, False

    while True:
        if fd is None:
            try:
                fd = open_port()
                emit(f'MO CONG {PORT_NOW}')
            except OSError:
                time.sleep(1)
                continue
        try:
            r, _, _ = select.select([fd, cmd_fd], [], [], 0.2)
            if cmd_fd in r:
                cmd_buf += os.read(cmd_fd, 1024)
                while b'\n' in cmd_buf:
                    line, cmd_buf = cmd_buf.split(b'\n', 1)
                    line = line.strip()
                    if line:
                        os.write(fd, line + b'\r')
                        log_line('>>> ' + line.decode(errors='replace'))
            if fd in r:
                data = os.read(fd, 4096)
                if data:
                    buf += data
                    last_rx = time.time()
                    if not alive:
                        alive = True
                        emit('CO TIN HIEU TU MACH')
        except OSError:
            emit(f'MAT CONG {PORT_NOW} (rut cap CH340?)')
            try:
                os.close(fd)
            except OSError:
                pass
            fd, alive = None, False
            time.sleep(1)
            continue

        while b'\n' in buf:
            raw, buf = buf.split(b'\n', 1)
            line = raw.decode(errors='replace').strip('\r\x00 ')
            if not line:
                continue
            log_line(line)
            for pat, label in EVENT_PATTERNS:
                if pat.search(line):
                    if label == 'IMU LOI':
                        if not imu_bad:
                            imu_bad = True
                            emit('IMU LOI: ' + line)
                    else:
                        emit(label + ': ' + line if label else line)
            if 'IMU loi' not in line and line.startswith('ENC'):
                if imu_bad:
                    imu_bad = False
                    emit('IMU OK LAI')
            m = re.search(r'PIN (\d+\.\d+) V', line)
            if m:
                v = float(m.group(1))
                if v < BATT_LOW_V and not batt_warned:
                    batt_warned = True
                    emit(f'PIN YEU {v} V')
                elif v >= BATT_LOW_V + 0.2:
                    batt_warned = False

        if alive and time.time() - last_rx > SILENT_S:
            alive = False
            emit('MACH IM LANG > 3 s (mat dien 12V hoac tat in dinh ky)')


_log_day, _log_f = None, None


def log_line(text):
    global _log_day, _log_f
    day = time.strftime('%Y%m%d')
    if day != _log_day:
        if _log_f:
            _log_f.close()
        _log_f = open(os.path.join(LOG_DIR, f'uart_{day}.log'), 'a', buffering=1)
        _log_day = day
    _log_f.write(time.strftime('%H:%M:%S ') + text + '\n')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
