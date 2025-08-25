# Create the arm_control.py file for the user
"""
arm_control.py
Simple CLI to control your Arduino-based robotic arm over a serial port.

Works with USB or an HC-05 Bluetooth serial link.
Depends on: pyserial

Usage examples:
  python arm_control.py --list
  python arm_control.py --port /dev/tty.usbmodem2101
  python arm_control.py            # will try to auto-pick a likely port

Once running, type commands like:
  up / down / set 120 / left / right / home / speed 6 / status / help / quit

These map to the Arduino sketch commands:
  SERVO <deg>, SERVO_REL <delta>, STEPPER <delta>, HOME, SPEED <rpm>, STATUS
"""
import argparse
import sys
import time
import threading
from typing import Optional

import serial
import serial.tools.list_ports as list_ports
import select
import termios
import tty

BAUD_DEFAULT = 115200


def find_default_port() -> Optional[str]:
    """Try to pick a sensible default serial port for Arduino/HC-05."""
    candidates = []
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        dev = p.device
        # Common hints for Arduino or USB serial devices on macOS/Windows/Linux
        if any(s in desc for s in ("arduino", "wch", "usb serial", "usb-serial", "usbmodem", "usbserial", "ch340", "cp210", "hc-05")):
            candidates.append(dev)
        else:
            # still consider any tty/COM device as a fallback candidate
            candidates.append(dev)
    return candidates[0] if candidates else None


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=0.1)
    # Give the Arduino time to reset after opening the port
    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def list_all_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        print(f"{p.device}  -  {p.description}")


def send(ser: serial.Serial, line: str) -> None:
    if not line.endswith("\n"):
        line += "\n"
    ser.write(line.encode("utf-8", errors="ignore"))
    ser.flush()


def reader_thread(ser: serial.Serial, stop_event: threading.Event) -> None:
    """Continuously print lines from Arduino prefixed with [arduino]."""
    while not stop_event.is_set():
        try:
            line = ser.readline()
            if line:
                try:
                    text = line.decode("utf-8", errors="replace").rstrip()
                except Exception:
                    text = str(line).rstrip()
                if text:
                    print(f"[arduino] {text}")
        except serial.SerialException:
            print("[!] Serial disconnected.")
            break
        except Exception:
            # Avoid noisy stack traces on intermittent decode errors
            pass
        time.sleep(0.01)


def wasd_loop(ser: serial.Serial, stop_event: threading.Event) -> None:
    """Read single keypresses (no Enter) and map WASD + helpers to commands."""
    print("WASD mode: press W/S to tilt servo, A/D to rotate stepper, H=home, Q=quit")
    print("Tip: holding a key will repeat based on your OS key repeat settings.")
    current_servo = 90

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not stop_event.is_set():
            try:
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
            except Exception:
                r = []
            if r:
                try:
                    ch = sys.stdin.read(1)
                except Exception:
                    ch = ""
                if not ch:
                    continue
                lower = ch.lower()
                if lower == "q" or ord(ch) == 3:  # 'q' or Ctrl-C
                    break
                if lower == "w":
                    current_servo = min(180, current_servo + 20)
                    send(ser, f"SERVO {current_servo}")
                    continue
                if lower == "s":
                    current_servo = max(0, current_servo - 20)
                    send(ser, f"SERVO {current_servo}")
                    continue
                if lower == "d":
                    send(ser, "STEPPER -20")
                    continue
                if lower == "a":
                    send(ser, "STEPPER +20")
                    continue
                if lower == "h":
                    send(ser, "HOME")
                    continue
                if lower == "x":
                    send(ser, "STATUS")
                    continue
            time.sleep(0.01)
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Control Arduino robotic arm over serial.")
    parser.add_argument("--port", "-p", help="Serial port (e.g., /dev/tty.usbmodem2101 or COM4).")
    parser.add_argument("--baud", "-b", type=int, default=BAUD_DEFAULT, help=f"Baud rate (default {BAUD_DEFAULT}).")
    parser.add_argument("--list", action="store_true", help="List available serial ports and exit.")
    parser.add_argument("--keys", "--wasd", dest="keys", action="store_true", help="Enable real-time WASD keys (no Enter required).")
    args = parser.parse_args(argv)

    if args.list:
        list_all_ports()
        return 0

    port = args.port or find_default_port()
    if not port:
        print("Could not find a serial port automatically. Use --list to see options, then pass --port.")
        return 1

    print(f"Opening {port} @ {args.baud}...")
    try:
        ser = open_serial(port, args.baud)
    except Exception as e:
        print(f"Failed to open port {port}: {e}")
        return 1

    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    print("Connected. Type commands or use shortcuts. Examples:")
    print("  up       → SERVO +5° (relative)")
    print("  down     → SERVO -5° (relative)")
    print("  set 120  → SERVO 120° (absolute)")
    print("  left     → STEPPER -20 steps")
    print("  right    → STEPPER +20 steps")
    print("  home     → HOME (center stepper)")
    print("  speed 6  → SPEED 6 rpm")
    print("  status   → STATUS from Arduino")
    print("  help     → show help")
    print("  quit     → exit")
    print("Also: WASD shortcuts available. Use --keys for immediate keypress control (no Enter).")
    print()

    current_servo = 90  # track relative up/down

    try:
        if args.keys:
            wasd_loop(ser, stop_event)
        else:
            while True:
                try:
                    cmd = input("> ").strip()
                except (EOFError, KeyboardInterrupt):
                    print()
                    break
                lower = cmd.lower()

                if lower in ("q", "quit", "exit"):
                    break

                if lower == "help":
                    print("Shortcuts: up, down, set <deg>, left, right, home, speed <rpm>, status, help, quit")
                    print("WASD shortcuts: w/s tilt servo ±20°, a/d rotate stepper ±20 steps")
                    print("Raw commands pass through to Arduino too (e.g., 'SERVO 135').")
                    continue

                if lower == "status":
                    send(ser, "STATUS")
                    continue

                if lower == "w":
                    current_servo = min(180, current_servo + 20)
                    send(ser, f"SERVO {current_servo}")
                    continue

                if lower in ("s", "down"):
                    delta = -20 if lower == "s" else -5
                    current_servo = max(0, current_servo + delta)
                    send(ser, f"SERVO {current_servo}")
                    continue

                if lower.startswith("set "):
                    try:
                        deg = int(lower.split()[1])
                        current_servo = max(0, min(180, deg))
                        send(ser, f"SERVO {current_servo}")
                    except Exception:
                        print("Usage: set <0-180>")
                    continue

                if lower in ("d", "left"):
                    send(ser, "STEPPER -20")
                    continue

                if lower in ("a", "right"):
                    send(ser, "STEPPER +20")
                    continue

                if lower == "home":
                    send(ser, "HOME")
                    continue

                if lower.startswith("speed "):
                    try:
                        rpm = int(lower.split()[1])
                        send(ser, f"SPEED {rpm}")
                    except Exception:
                        print("Usage: speed <2..15>")
                    continue

                # If user typed a raw Arduino command, forward it:
                if cmd:
                    send(ser, cmd)

    finally:
        stop_event.set()
        try:
            ser.close()
        except Exception:
            pass
        print("Closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

