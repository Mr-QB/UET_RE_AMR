#!/usr/bin/env python3
"""
Standalone serial probe for the amr_uart_bridge firmware, bypassing ROS2 /
ros2_control entirely. Opens the serial port raw, resyncs on the 0xAA
header, and decodes+prints FeedbackPacket frames as they arrive.

Wire format: see firmware/amr_uart_bridge/include/ros_protocol.h and
ros2/src/uet_amr_hardware/include/uet_amr_hardware/protocol.hpp (must stay
in sync with both).

Usage:
    tools/check_firmware_response.py [port] [baud]

Defaults: /dev/ttyCH341USB0, 921600. Exits 0 as soon as one valid frame is
decoded, non-zero on timeout (no data / no valid frame within TIMEOUT_S) --
useful to tell "nothing on the wire" (wiring/power/port) apart from "bytes
arriving but never a valid frame" (baud mismatch, wrong firmware, noise).
"""
import os
import sys
import termios
import time

HEAD = 0xAA
FB_LEN = 16
TIMEOUT_S = 5.0

BAUD_MAP = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}


def open_serial(port: str, baud: int) -> int:
    if baud not in BAUD_MAP:
        sys.exit(f"Unsupported baud {baud}; supported: {sorted(BAUD_MAP)}")

    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    speed = BAUD_MAP[baud]
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)

    cflag |= termios.CLOCAL | termios.CREAD
    cflag &= ~termios.PARENB
    cflag &= ~termios.CSTOPB
    cflag &= ~termios.CSIZE
    cflag |= termios.CS8
    if hasattr(termios, 'CRTSCTS'):
        cflag &= ~termios.CRTSCTS
    iflag = 0
    oflag = 0
    lflag = 0
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0

    termios.tcsetattr(
        fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, speed, speed, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def checksum16(data: bytes) -> int:
    c = 0
    for i, b in enumerate(data):
        c ^= b << ((i & 1) * 8)
    return c & 0xFFFF


def decode(frame: bytes) -> dict:
    en_tick_l = frame[4] | ((frame[5] & 0x3F) << 8)
    en_tick_r = ((frame[5] >> 6) & 0x3) | (frame[6] << 2) | ((frame[7] & 0xF) << 10)
    current_a = ((frame[8] >> 7) & 0x1) | (frame[9] << 1) | ((frame[10] & 0x3) << 9)
    battery = ((frame[10] >> 2) & 0x3F) | ((frame[11] & 0x1) << 6)

    return {
        'sys_status': frame[1] & 0x3,
        'speed_l': frame[2] if frame[2] < 128 else frame[2] - 256,
        'speed_r': frame[3] if frame[3] < 128 else frame[3] - 256,
        'en_tick_l': en_tick_l,
        'en_tick_r': en_tick_r,
        'temp_c': frame[8] & 0x7F,
        'current_a': current_a / 100.0,
        'battery_pct': battery,
        'charging': bool((frame[11] >> 1) & 0x1),
    }


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyCH341USB0'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600

    print(f"Opening {port} @ {baud} baud (raw, no ROS2 required)...")
    try:
        fd = open_serial(port, baud)
    except OSError as e:
        print(f"Failed to open '{port}': {e}")
        return 1

    buf = bytearray()
    bytes_seen = 0
    frames_ok = 0
    frames_bad_checksum = 0
    deadline = time.monotonic() + TIMEOUT_S

    try:
        while time.monotonic() < deadline:
            chunk = os.read(fd, 256)
            if chunk:
                bytes_seen += len(chunk)
                buf.extend(chunk)
            else:
                time.sleep(0.01)

            while True:
                idx = buf.find(HEAD)
                if idx < 0:
                    buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < FB_LEN:
                    break

                frame = bytes(buf[:FB_LEN])
                crc = checksum16(frame[:FB_LEN - 2])
                crc_wire = frame[FB_LEN - 2] | (frame[FB_LEN - 1] << 8)

                if crc == crc_wire:
                    frames_ok += 1
                    fields = decode(frame)
                    print(
                        f"[{frames_ok:04d}] sys={fields['sys_status']} "
                        f"speed=({fields['speed_l']:+d},{fields['speed_r']:+d}) "
                        f"ticks=({fields['en_tick_l']},{fields['en_tick_r']}) "
                        f"temp={fields['temp_c']}C "
                        f"current={fields['current_a']:.2f}A "
                        f"batt={fields['battery_pct']}% "
                        f"charging={fields['charging']}"
                    )
                    del buf[:FB_LEN]
                    deadline = time.monotonic() + TIMEOUT_S
                else:
                    frames_bad_checksum += 1
                    del buf[0]  # resync: drop the false header, keep scanning

        if frames_ok > 0:
            print(f"\nOK: {frames_ok} valid frame(s) decoded, "
                  f"{frames_bad_checksum} checksum failure(s), "
                  f"{bytes_seen} bytes total.")
            return 0

        if bytes_seen == 0:
            print(
                f"\nNo data at all on '{port}' within {TIMEOUT_S:.0f}s.\n"
                "Check: bridge MCU powered/flashed, correct port, cabling."
            )
        else:
            print(
                f"\n{bytes_seen} bytes seen but no valid frame "
                f"({frames_bad_checksum} checksum failures) within "
                f"{TIMEOUT_S:.0f}s.\n"
                "Check: baud rate matches firmware (921600), correct wire "
                "protocol version, or line noise."
            )
        return 1
    except KeyboardInterrupt:
        return 1
    finally:
        os.close(fd)


if __name__ == '__main__':
    sys.exit(main())
