#!/usr/bin/env python3
"""Lien ket UART Pi <-> STM32 (khung nhi phan, xem docs/UART_PROTOCOL.md).

Dung:
    link = Stm32Link('/dev/serial0')
    link.set_speed([500, 500, 500, 500])   # xung/s; phai goi lai it nhat moi 0.25 s
    t = link.telem                          # dict telemetry moi nhat (hoac None)
    link.close()

An toan: neu chuong trinh khong goi set_speed()/set_pwm() trong HOLD_S giay,
link tu gui STOP. STM32 cung tu dung neu khong nhan khung nao trong 300 ms.
"""
import struct
import threading
import time

try:
    import serial          # sudo apt install python3-serial
except ImportError:        # cho phep import de test ham dong goi khung khi chua cai pyserial
    serial = None

CMD_SPEED, CMD_PWM, CMD_STOP, CMD_LIFT_MOVE, CMD_LIFT_HOME, CMD_PING, CMD_BODY = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
CMD_TELEM = 0x81

F_LIMIT_TOP, F_LIMIT_BOT, F_LIFT_BUSY, F_LIFT_HOMED = 0x01, 0x02, 0x04, 0x08
F_IMU_OK, F_TIMEOUT, F_WDG_RESET = 0x10, 0x20, 0x40

TELEM_FMT = '<I4i4h3h3hHiBBh'         # khop link_telem_t trong firmware (50 byte)
TELEM_SIZE = struct.calcsize(TELEM_FMT)

# Dau xe theo quy uoc nguoi dung nguoc voi truc x trong firmware (thu tren san 2026-11-19):
# chi dao vx. Kiem tra tren san: vy + = sang trai dung, wz + = xoay trai dung. Doi o day, khong can nap lai STM32.
FRAME_FLIP = False   # 2026-11-19: firmware da doi dau xe = phia M1, M2, khong can lat o Pi nua

SEND_PERIOD_S = 0.05                   # gui lenh 20 Hz (firmware tu dung sau 300 ms)
HOLD_S = 0.25                          # lenh cu qua thoi gian nay -> gui STOP


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE, giong Link_Crc16() trong firmware."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(cmd, payload=b''):
    body = bytes([len(payload), cmd]) + payload
    return b'\xAA\x55' + body + struct.pack('<H', crc16(body))


class FrameParser:
    """Tach khung tu dong byte. feed() tra ve danh sach (cmd, payload)."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b'\xAA\x55')
            if i < 0:
                del self.buf[:-1]               # giu lai byte cuoi phong khi la 0xAA
                return out
            del self.buf[:i]
            if len(self.buf) < 4:
                return out
            n = self.buf[2]
            if len(self.buf) < 6 + n:
                return out
            body = bytes(self.buf[2:4 + n])
            crc = struct.unpack_from('<H', self.buf, 4 + n)[0]
            if crc == crc16(body):
                out.append((body[1], body[2:]))
                del self.buf[:6 + n]
            else:
                del self.buf[:1]                # sai CRC: bo 0xAA, tim tiep


def parse_telem(payload):
    if len(payload) != TELEM_SIZE:
        return None
    v = struct.unpack(TELEM_FMT, payload)
    flags = v[17]
    return {
        'ms': v[0], 'enc': list(v[1:5]), 'speed': list(v[5:9]),
        'acc': list(v[9:12]), 'gyro': list(v[12:15]),
        'batt_v': v[15] / 1000.0, 'lift_pos': v[16], 'flags': flags, 'wheel_mode': v[18], 'heading_deg': v[19] / 100.0,
        'limit_top': bool(flags & F_LIMIT_TOP), 'limit_bot': bool(flags & F_LIMIT_BOT),
        'lift_busy': bool(flags & F_LIFT_BUSY), 'lift_homed': bool(flags & F_LIFT_HOMED),
        'imu_ok': bool(flags & F_IMU_OK), 'timeout': bool(flags & F_TIMEOUT),
        'wdg_reset': bool(flags & F_WDG_RESET),
    }


class Stm32Link:
    def __init__(self, port='/dev/serial0', baud=460800, on_telem=None):
        if serial is None:
            raise RuntimeError('Chua cai pyserial: sudo apt install python3-serial')
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.on_telem = on_telem
        self.telem = None
        self.telem_time = 0.0
        self._lock = threading.Lock()
        self._cmd = None                # (cmd, payload) lap lai lien tuc
        self._cmd_time = 0.0
        self._run = True
        self._parser = FrameParser()
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx.start()
        self._tx.start()

    # ----- banh xe -----
    def set_speed(self, cps):
        """Toc do 4 banh, xung encoder/giay (M1..M4)."""
        self._set_repeat(CMD_SPEED, struct.pack('<4h', *[_clamp16(x) for x in cps]))

    def drive(self, vx, vy, wz, hold=True):
        """Ca xe mecanum (STM32 tu chia 4 banh): vx toi +, vy sang trai +, wz xoay trai + (xung/s moi banh).
        hold=True: khi wz = 0 thi STM32 dung gyro giu huong xe. Goi lai it nhat moi 0.25 s."""
        if FRAME_FLIP:
            vx = -vx
        self._set_repeat(CMD_BODY, struct.pack('<3hB', _clamp16(vx), _clamp16(vy), _clamp16(wz), 1 if hold else 0))

    def set_pwm(self, duty):
        """PWM tho -5000..5000, de thu khi chua chinh PID."""
        self._set_repeat(CMD_PWM, struct.pack('<4h', *[max(-5000, min(5000, int(x))) for x in duty]))

    def stop(self):
        with self._lock:
            self._cmd = None
        self._send(CMD_STOP)

    # ----- nang ha -----
    def lift_move(self, steps_up, sps=1600):
        self._send(CMD_LIFT_MOVE, struct.pack('<iH', int(steps_up), int(sps)))

    def lift_home(self, sps=1600):
        self._send(CMD_LIFT_HOME, struct.pack('<H', int(sps)))

    # ----- noi bo -----
    def _set_repeat(self, cmd, payload):
        with self._lock:
            self._cmd = (cmd, payload)
            self._cmd_time = time.time()
        self._send(cmd, payload)

    def _send(self, cmd, payload=b''):
        self.ser.write(build_frame(cmd, payload))

    def _tx_loop(self):
        while self._run:
            time.sleep(SEND_PERIOD_S)
            with self._lock:
                cmd, age = self._cmd, time.time() - self._cmd_time
            try:
                if cmd is None:
                    self._send(CMD_PING)
                elif age > HOLD_S:              # chuong trinh chinh khong cap nhat -> dung
                    with self._lock:
                        self._cmd = None
                    self._send(CMD_STOP)
                else:
                    self._send(*cmd)
            except (OSError, serial.SerialException):
                time.sleep(0.5)

    def _rx_loop(self):
        while self._run:
            try:
                data = self.ser.read(256)
            except (OSError, serial.SerialException):
                time.sleep(0.5)
                continue
            for cmd, payload in self._parser.feed(data):
                if cmd == CMD_TELEM:
                    t = parse_telem(payload)
                    if t:
                        self.telem, self.telem_time = t, time.time()
                        if self.on_telem:
                            self.on_telem(t)

    def close(self):
        try:
            self.stop()
        finally:
            self._run = False
            time.sleep(0.1)
            self.ser.close()


def _clamp16(x):
    return max(-32768, min(32767, int(x)))


if __name__ == '__main__':
    # Thu nhanh: in telemetry. Banh KE LEN truoc khi dung tham so --spin.
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/serial0'
    link = Stm32Link(port)
    spin = '--spin' in sys.argv
    try:
        t0 = time.time()
        while True:
            if spin and time.time() - t0 < 3:
                link.set_speed([800, 800, 800, 800])
            elif spin:
                link.stop()
                spin = False
            t = link.telem
            if t:
                print(f"enc {t['enc']} v {t['speed']} pin {t['batt_v']:.2f}V lift {t['lift_pos']} "
                      f"imu {'OK' if t['imu_ok'] else 'LOI'} flags 0x{t['flags']:02x}", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
