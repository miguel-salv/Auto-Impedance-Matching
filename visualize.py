#!/usr/bin/env python3
import argparse
from collections import deque
import csv
import os
import time

import matplotlib.pyplot as plt
import serial
from serial.tools import list_ports


DEFAULT_BAUD = 500000
DEFAULT_WINDOW_SECONDS = 20.0

# Prevent MacOS matplotlib windows from stealing focus on redraw.
plt.rcParams["figure.raise_window"] = False


def get_default_port():
    ports = [p.device for p in list_ports.comports()]
    for device in ports:
        if "cu.usbmodem" in device:
            return device
    for device in ports:
        if "tty.usbmodem" in device:
            return device
    return "/dev/cu.usbmodem12345"


def parse_args():
    parser = argparse.ArgumentParser(description="Live plot VSWR telemetry from serial.")
    parser.add_argument("--port", default=get_default_port(), help="Serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--window-seconds", type=float, default=DEFAULT_WINDOW_SECONDS, help="Time window for recent plots")
    parser.add_argument("--csv", default="vswr_log.csv", help="CSV log output path")
    return parser.parse_args()


def connect_serial(port, baud):
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            print(f"Listening on {port} @ {baud} baud...")
            return ser
        except serial.SerialException as exc:
            print(f"Serial open failed: {exc}")
            print("Close Arduino/Teensy Serial Monitor (or other app using the port), then retrying in 1s...")
            time.sleep(1)


def main():
    args = parse_args()
    ser = connect_serial(args.port, args.baud)

    times = deque()
    vswr_vals = deque()
    all_times = []
    all_vswr_vals = []
    motor1_vals = deque()
    motor2_vals = deque()

    plt.ion()
    fig, (ax_vswr, ax_vswr_full, ax_motor) = plt.subplots(3, 1, sharex=False)
    line_vswr, = ax_vswr.plot([], [], label="VSWR")
    line_vswr_full, = ax_vswr_full.plot([], [], label="VSWR (Whole Run)")
    line_motor1, = ax_motor.plot([], [], label="Motor 1 Pos (rad)")
    line_motor2, = ax_motor.plot([], [], label="Motor 2 Pos (rad)")

    ax_vswr.set_ylabel("VSWR (Window)")
    ax_vswr_full.set_ylabel("VSWR (Run)")
    ax_motor.set_ylabel("Motor Pos (rad)")
    ax_motor.set_xlabel("Time (s)")
    ax_vswr.grid(True, alpha=0.3)
    ax_vswr_full.grid(True, alpha=0.3)
    ax_motor.grid(True, alpha=0.3)
    ax_vswr.legend(loc="upper right")
    ax_vswr_full.legend(loc="upper right")
    ax_motor.legend(loc="upper right")
    status_text = fig.text(0.02, 0.98, "Match: unknown", ha="left", va="top")
    plt.show(block=False)

    t0_host = None
    csv_exists = os.path.exists(args.csv)
    with open(args.csv, "a", newline="") as csv_file:
        writer = csv.writer(csv_file)
        if not csv_exists or os.path.getsize(args.csv) == 0:
            writer.writerow(["host_time_s", "device_millis", "vswr", "motor1_pos_rad", "motor2_pos_rad", "at_match"])

        try:
            while True:
                try:
                    raw = ser.readline().decode("utf-8", errors="ignore").strip()
                except serial.SerialException as exc:
                    print(f"Serial read failed: {exc}")
                    try:
                        ser.close()
                    except Exception:
                        pass
                    print("Attempting to reconnect...")
                    ser = connect_serial(args.port, args.baud)
                    continue

                if not raw.startswith("VSWR_CSV,"):
                    continue

                parts = raw.split(",")
                if len(parts) != 6:
                    continue

                _, millis_s, vswr_s, motor1_s, motor2_s, at_match_s = parts
                try:
                    t_ms = int(millis_s)
                    vswr = float(vswr_s)
                    motor1_pos = float(motor1_s)
                    motor2_pos = float(motor2_s)
                    at_match = int(at_match_s) != 0
                except ValueError:
                    continue

                host_now = time.time()
                if t0_host is None:
                    t0_host = host_now

                t_s = host_now - t0_host
                times.append(t_s)
                vswr_vals.append(vswr)
                all_times.append(t_s)
                all_vswr_vals.append(vswr)
                motor1_vals.append(motor1_pos)
                motor2_vals.append(motor2_pos)

                while times and (t_s - times[0]) > args.window_seconds:
                    times.popleft()
                    vswr_vals.popleft()
                    motor1_vals.popleft()
                    motor2_vals.popleft()

                line_vswr.set_data(times, vswr_vals)
                line_vswr_full.set_data(all_times, all_vswr_vals)
                line_motor1.set_data(times, motor1_vals)
                line_motor2.set_data(times, motor2_vals)

                status_text.set_text(f"Match: {'YES' if at_match else 'NO'}")

                writer.writerow([host_now, t_ms, vswr, motor1_pos, motor2_pos, int(at_match)])
                csv_file.flush()

                ax_vswr.relim()
                ax_vswr.autoscale_view()
                ax_vswr_full.relim()
                ax_vswr_full.autoscale_view()
                ax_motor.relim()
                ax_motor.autoscale_view()
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
        except KeyboardInterrupt:
            print("\nStopping.")
        finally:
            ser.close()


if __name__ == "__main__":
    main()
