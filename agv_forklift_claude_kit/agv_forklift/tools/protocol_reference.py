#!/usr/bin/env python3
"""Reference implementation of the Pi <-> STM32 UART protocol.

This file is the EXECUTABLE SPECIFICATION of the wire format described in
docs/03_uart_protocol.md. The STM32 firmware (C) and the ROS 2 bridge (Python)
MUST produce and accept exactly the bytes generated here.

Usage:
  python3 tools/protocol_reference.py --print   # print golden vectors
  python3 tools/protocol_reference.py --write   # regenerate tools/protocol_vectors.json
                                                #   and firmware/host_tests/protocol_vectors.h
  python3 tools/protocol_reference.py --check   # self-tests + verify files on disk are up to date

Any change to the wire format requires: bump PROTOCOL_VERSION, update
docs/03_uart_protocol.md, run --write, update firmware + bridge, record in
docs/07_decisions.md.
"""
import argparse
import json
import struct
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
JSON_PATH = REPO_ROOT / "tools" / "protocol_vectors.json"
C_HEADER_PATH = REPO_ROOT / "firmware" / "host_tests" / "protocol_vectors.h"

SOF1 = 0xAA
SOF2 = 0x55
PROTOCOL_VERSION = 0x01
HEADER_SIZE = 6            # SOF1 SOF2 VER MSG_ID SEQ LEN
CRC_SIZE = 2
MAX_PAYLOAD = 64
ARM_MAGIC = 0x214D5241     # bytes on wire (LE): 41 52 4D 21 = ASCII "ARM!"

# msg_id: (name, struct format (little-endian, no padding), field names in order)
MESSAGES = {
    # ---- Pi -> STM32 ----
    0x01: ("CMD_VELOCITY", "<fff", ("vx", "vy", "wz")),
    0x02: ("CMD_LIFT", "<BH", ("action", "request_id")),
    0x03: ("CMD_STOP", "<BB", ("reason", "disarm")),
    0x04: ("HEARTBEAT", "<I", ("uptime_ms",)),          # both directions
    0x05: ("CMD_ARM", "<I", ("magic",)),
    0x06: ("CMD_SET_PID", "<Bfff", ("wheel", "kp", "ki", "kd")),
    # ---- STM32 -> Pi ----
    0x81: ("TELEMETRY_ODOM", "<Iffffff", ("t_ms", "x", "y", "theta", "vx", "vy", "wz")),
    0x82: ("TELEMETRY_MOTOR", "<ffffffffhhhhB",
           ("rpm_target_fl", "rpm_target_fr", "rpm_target_rl", "rpm_target_rr",
            "rpm_actual_fl", "rpm_actual_fr", "rpm_actual_rl", "rpm_actual_rr",
            "pwm_fl", "pwm_fr", "pwm_rl", "pwm_rr", "encoder_fault_mask")),
    0x83: ("TELEMETRY_LIFT", "<BBBBHiH",
           ("state", "flags", "last_action", "reserved", "request_id",
            "position_steps", "error_code")),
    0x84: ("TELEMETRY_ERROR", "<HBBI", ("error_code", "severity", "component", "t_ms")),
    0x85: ("TELEMETRY_STATUS", "<IIBBHHH",
           ("t_ms", "fault_flags", "mode", "estop", "vbat_mv",
            "rx_crc_errors", "rx_frames_dropped")),
    0x86: ("TELEMETRY_IMU", "<ffffff", ("ax", "ay", "az", "gx", "gy", "gz")),
    0x8F: ("ACK", "<BBBB", ("acked_msg_id", "acked_seq", "result", "reason")),
}
NAME_TO_ID = {spec[0]: msg_id for msg_id, spec in MESSAGES.items()}


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, refin/refout false, xorout 0x0000."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def pack_payload(name: str, **fields) -> bytes:
    msg_id = NAME_TO_ID[name]
    _, fmt, names = MESSAGES[msg_id]
    missing = set(names) - set(fields)
    extra = set(fields) - set(names)
    if missing or extra:
        raise ValueError(f"{name}: missing={sorted(missing)} extra={sorted(extra)}")
    return struct.pack(fmt, *(fields[n] for n in names))


def encode_frame(name: str, seq: int, **fields) -> bytes:
    msg_id = NAME_TO_ID[name]
    payload = pack_payload(name, **fields)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"{name}: payload {len(payload)} > {MAX_PAYLOAD}")
    body = bytes([PROTOCOL_VERSION, msg_id, seq & 0xFF, len(payload)]) + payload
    return bytes([SOF1, SOF2]) + body + struct.pack("<H", crc16_ccitt_false(body))


class StreamDecoder:
    """Buffer-based decoder.

    On ANY framing error (bad version, bad length, bad CRC) it drops exactly ONE
    byte (the SOF1 of the bad candidate) and rescans the remaining buffer, so a
    valid frame hidden inside a corrupted candidate is never lost.
    """

    def __init__(self):
        self.buf = bytearray()
        self.stats = {"frames": 0, "crc_errors": 0, "version_errors": 0,
                      "length_errors": 0, "unknown_ids": 0, "bad_payload_len": 0}

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while True:
            idx = self.buf.find(bytes([SOF1, SOF2]))
            if idx < 0:
                # Keep a trailing SOF1: it may be the first half of the next header.
                keep = 1 if self.buf[-1:] == bytes([SOF1]) else 0
                del self.buf[: len(self.buf) - keep]
                return out
            del self.buf[:idx]
            if len(self.buf) < HEADER_SIZE:
                return out
            ver, msg_id, seq, length = self.buf[2], self.buf[3], self.buf[4], self.buf[5]
            if ver != PROTOCOL_VERSION:
                self.stats["version_errors"] += 1
                del self.buf[:1]
                continue
            if length > MAX_PAYLOAD:
                self.stats["length_errors"] += 1
                del self.buf[:1]
                continue
            total = HEADER_SIZE + length + CRC_SIZE
            if len(self.buf) < total:
                return out  # wait for more bytes; caller applies the inter-byte timeout
            body = bytes(self.buf[2:HEADER_SIZE + length])
            (rx_crc,) = struct.unpack_from("<H", self.buf, HEADER_SIZE + length)
            if crc16_ccitt_false(body) != rx_crc:
                self.stats["crc_errors"] += 1
                del self.buf[:1]
                continue
            payload = bytes(self.buf[HEADER_SIZE:HEADER_SIZE + length])
            del self.buf[:total]
            if msg_id not in MESSAGES:
                self.stats["unknown_ids"] += 1
                continue
            name, fmt, names = MESSAGES[msg_id]
            if struct.calcsize(fmt) != length:
                self.stats["bad_payload_len"] += 1
                continue
            self.stats["frames"] += 1
            out.append({"name": name, "msg_id": msg_id, "seq": seq,
                        "fields": dict(zip(names, struct.unpack(fmt, payload)))})

    def on_interbyte_timeout(self):
        """Call when a partial frame has waited longer than the inter-byte timeout."""
        if self.buf:
            del self.buf[:1]
        return self.feed(b"")


# --------------------------------------------------------------------------- vectors
FRAME_VECTORS = [
    # (vector name, message name, seq, fields)  -- floats chosen to be exact in binary32
    ("CMD_VELOCITY_FWD", "CMD_VELOCITY", 0, dict(vx=0.25, vy=0.0, wz=0.5)),
    ("CMD_LIFT_UP", "CMD_LIFT", 1, dict(action=2, request_id=7)),
    ("CMD_ARM", "CMD_ARM", 2, dict(magic=ARM_MAGIC)),
    ("CMD_STOP_OPERATOR", "CMD_STOP", 3, dict(reason=2, disarm=0)),
    ("HEARTBEAT_PI", "HEARTBEAT", 4, dict(uptime_ms=60000)),
    ("CMD_SET_PID_ALL", "CMD_SET_PID", 5, dict(wheel=0xFF, kp=1.5, ki=0.25, kd=0.0)),
    ("TELEMETRY_ODOM", "TELEMETRY_ODOM", 10,
     dict(t_ms=1000, x=1.0, y=-0.5, theta=0.25, vx=0.125, vy=0.0, wz=-0.5)),
    ("TELEMETRY_MOTOR", "TELEMETRY_MOTOR", 11,
     dict(rpm_target_fl=60.0, rpm_target_fr=60.0, rpm_target_rl=60.0, rpm_target_rr=60.0,
          rpm_actual_fl=59.5, rpm_actual_fr=60.25, rpm_actual_rl=58.75, rpm_actual_rr=61.0,
          pwm_fl=420, pwm_fr=-415, pwm_rl=430, pwm_rr=-410, encoder_fault_mask=0)),
    ("TELEMETRY_LIFT_DONE", "TELEMETRY_LIFT", 12,
     dict(state=4, flags=0x11, last_action=2, reserved=0, request_id=7,  # homed + at_up
          position_steps=16000, error_code=0)),
    ("TELEMETRY_ERROR_WATCHDOG", "TELEMETRY_ERROR", 13,
     dict(error_code=0x0101, severity=3, component=5, t_ms=123456)),
    ("TELEMETRY_STATUS_FAULT", "TELEMETRY_STATUS", 14,
     dict(t_ms=123456, fault_flags=0x00000003, mode=2, estop=1, vbat_mv=12340,
          rx_crc_errors=5, rx_frames_dropped=1)),
    ("TELEMETRY_IMU", "TELEMETRY_IMU", 15,
     dict(ax=0.0, ay=0.0, az=9.75, gx=0.0, gy=0.0, gz=0.5)),
    ("ACK_LIFT_OK", "ACK", 16, dict(acked_msg_id=0x02, acked_seq=1, result=0, reason=0)),
]

STREAM_EXPECTED_NAMES = ["CMD_LIFT", "CMD_ARM"]
STREAM_EXPECTED_STATS = {"crc_errors": 1, "version_errors": 1}


def build_stream_vector() -> bytes:
    """Garbage + CRC-corrupted frame + truncated header + valid frame + garbage + valid frame."""
    good_a = encode_frame("CMD_LIFT", 1, action=2, request_id=7)
    good_b = encode_frame("CMD_ARM", 2, magic=ARM_MAGIC)
    corrupted = bytearray(encode_frame("CMD_VELOCITY", 0, vx=0.25, vy=0.0, wz=0.5))
    corrupted[8] ^= 0xFF  # flip one payload byte -> CRC must fail
    return b"\x00\x13\xAA" + bytes(corrupted) + b"\xAA\x55" + good_a + b"\x55\xAA" + good_b


def build_vectors() -> dict:
    frames = {}
    for vec_name, msg_name, seq, fields in FRAME_VECTORS:
        frames[vec_name] = {"message": msg_name, "msg_id": NAME_TO_ID[msg_name], "seq": seq,
                            "fields": fields, "hex": encode_frame(msg_name, seq, **fields).hex(" ")}
    return {
        "protocol_version": PROTOCOL_VERSION,
        "crc_check": {"algorithm": "CRC-16/CCITT-FALSE", "input_ascii": "123456789",
                      "crc": f"0x{crc16_ccitt_false(b'123456789'):04X}"},
        "frames": frames,
        "stream": {"hex": build_stream_vector().hex(" "),
                   "expected_messages": STREAM_EXPECTED_NAMES,
                   "expected_stats": STREAM_EXPECTED_STATS},
    }


def render_c_header(vectors: dict) -> str:
    lines = [
        "/* AUTO-GENERATED by tools/protocol_reference.py --write. DO NOT EDIT. */",
        "#ifndef PROTOCOL_VECTORS_H",
        "#define PROTOCOL_VECTORS_H",
        "#include <stdint.h>",
        "",
        f"#define PV_PROTOCOL_VERSION 0x{PROTOCOL_VERSION:02X}u",
        '#define PV_CRC_CHECK_INPUT "123456789"',
        f"#define PV_CRC_CHECK_VALUE {vectors['crc_check']['crc']}u",
        f"#define PV_ARM_MAGIC 0x{ARM_MAGIC:08X}u",
        "",
    ]

    def c_array(name: str, hex_str: str):
        data = bytes.fromhex(hex_str)
        body = ", ".join(f"0x{b:02X}" for b in data)
        lines.append(f"static const uint8_t {name}[{len(data)}] = {{ {body} }};")

    for vec_name, vec in vectors["frames"].items():
        c_array(f"PV_{vec_name}", vec["hex"])
    lines.append("")
    c_array("PV_STREAM", vectors["stream"]["hex"])
    lines += [
        f"#define PV_STREAM_EXPECTED_FRAMES {len(STREAM_EXPECTED_NAMES)}u",
        f"#define PV_STREAM_EXPECTED_CRC_ERRORS {STREAM_EXPECTED_STATS['crc_errors']}u",
        f"#define PV_STREAM_EXPECTED_VERSION_ERRORS {STREAM_EXPECTED_STATS['version_errors']}u",
        "",
        "#endif /* PROTOCOL_VECTORS_H */",
        "",
    ]
    return "\n".join(lines)


def self_test() -> list:
    errors = []
    if crc16_ccitt_false(b"123456789") != 0x29B1:
        errors.append("CRC check value mismatch (expected 0x29B1)")
    for vec_name, msg_name, seq, fields in FRAME_VECTORS:
        frame = encode_frame(msg_name, seq, **fields)
        dec = StreamDecoder()
        out = dec.feed(frame)
        if len(out) != 1 or out[0]["name"] != msg_name or out[0]["seq"] != seq:
            errors.append(f"{vec_name}: round-trip failed")
            continue
        for key, val in fields.items():
            got = out[0]["fields"][key]
            if (abs(got - val) > 1e-6) if isinstance(val, float) else (got != val):
                errors.append(f"{vec_name}.{key}: {got} != {val}")
    stream = build_stream_vector()
    for mode in ("bulk", "bytewise"):
        dec = StreamDecoder()
        out = dec.feed(stream) if mode == "bulk" else [f for b in stream for f in dec.feed(bytes([b]))]
        names = [f["name"] for f in out]
        if names != STREAM_EXPECTED_NAMES:
            errors.append(f"stream ({mode}): got {names}, expected {STREAM_EXPECTED_NAMES}")
        for key, val in STREAM_EXPECTED_STATS.items():
            if dec.stats[key] != val:
                errors.append(f"stream ({mode}): stats[{key}]={dec.stats[key]}, expected {val}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--print", action="store_true")
    group.add_argument("--write", action="store_true")
    group.add_argument("--check", action="store_true")
    args = parser.parse_args()

    errors = self_test()
    if errors:
        print("SELF-TEST FAILED:\n  " + "\n  ".join(errors))
        return 1
    vectors = build_vectors()
    json_text = json.dumps(vectors, indent=2) + "\n"
    header_text = render_c_header(vectors)

    if args.print:
        print(json_text)
        return 0
    if args.write:
        JSON_PATH.write_text(json_text, encoding="utf-8")
        C_HEADER_PATH.parent.mkdir(parents=True, exist_ok=True)
        C_HEADER_PATH.write_text(header_text, encoding="utf-8")
        print(f"wrote {JSON_PATH.relative_to(REPO_ROOT)} and {C_HEADER_PATH.relative_to(REPO_ROOT)}")
        return 0
    stale = []
    if not JSON_PATH.exists() or JSON_PATH.read_text(encoding="utf-8") != json_text:
        stale.append(str(JSON_PATH.relative_to(REPO_ROOT)))
    if not C_HEADER_PATH.exists() or C_HEADER_PATH.read_text(encoding="utf-8") != header_text:
        stale.append(str(C_HEADER_PATH.relative_to(REPO_ROOT)))
    if stale:
        print("OUT OF DATE (run --write): " + ", ".join(stale))
        return 1
    print("OK: self-tests passed, vector files up to date")
    return 0


if __name__ == "__main__":
    sys.exit(main())
