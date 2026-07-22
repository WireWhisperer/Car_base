#!/usr/bin/env python3
"""
小车运行时日志查看器
====================

用法:
    # 实时查看日志 (连接串口)
    python log_viewer.py COM3 115200

    # 从保存的日志文件回放
    python log_viewer.py logfile.bin --replay

    # 导出为 CSV (可用 Excel 打开)
    python log_viewer.py COM3 115200 --csv output.csv

    # 实时绘图 (需要 matplotlib)
    python log_viewer.py COM3 115200 --plot

帧格式 (10 字节):
    [0xAA] [type:1] [timestamp:4 LE] [val_a:2 LE] [val_b:2 LE] [chk:1]
"""

import sys
import struct
import time
import serial
from datetime import datetime
from collections import defaultdict

# =============================================================================
# 日志类型定义
# =============================================================================

EVENT_NAMES = {
    0x00: "HEARTBEAT",
    0x01: "PID_SPEED",
    0x02: "PID_DUTY",
    0x03: "TARGET_SPEED",
    0x04: "GYRO_YAW",
    0x05: "GRAY_SENSOR",
    0x06: "STATE_CHANGE",
    0x07: "BUTTON",
    0x08: "ROTATE_PARAM",
    0x0F: "CUSTOM",
}

STATE_NAMES = {0: "IDLE", 1: "PATROL", 2: "RECT_TRACE", 3: "ROTATE", 4: "MANUAL"}

# =============================================================================
# 帧解析器
# =============================================================================


class LogParser:
    """从字节流中提取并验证日志帧"""

    def __init__(self):
        self.buffer = bytearray()
        self.frame_count = 0
        self.error_count = 0
        self.last_timestamp = 0

    def feed(self, data: bytes) -> list[dict]:
        """输入原始字节, 返回解析出的日志条目列表"""
        self.buffer.extend(data)
        entries = []

        while len(self.buffer) >= 10:
            # 查找帧头
            if self.buffer[0] != 0xAA:
                self.buffer.pop(0)
                continue

            # 提取候选帧
            frame = self.buffer[:10]

            # 验证校验和
            checksum = 0
            for i in range(1, 9):
                checksum ^= frame[i]

            if checksum != frame[9]:
                self.buffer.pop(0)
                self.error_count += 1
                continue

            # 解码
            entry = {
                "type": frame[1],
                "type_name": EVENT_NAMES.get(frame[1], f"UNKNOWN(0x{frame[1]:02X})"),
                "timestamp": struct.unpack("<I", frame[2:6])[0],
                "val_a": struct.unpack("<h", frame[6:8])[0],
                "val_b": struct.unpack("<h", frame[8:10])[0] if False else self._decode_val_b(frame),
            }
            # 手动解码 val_b (因为 checksum 占了 frame[9])
            entry["val_b"] = struct.unpack("<h", frame[7:9])[0] if False else None
            # 重新正确解码
            entry["val_b"] = struct.unpack("<h", bytes([frame[7], frame[8]]))[0]

            self.last_timestamp = entry["timestamp"]
            self.frame_count += 1
            entries.append(entry)

            # 移除已解析的帧
            self.buffer = self.buffer[10:]

        return entries


# =============================================================================
# 格式化输出
# =============================================================================


def format_entry(entry: dict) -> str:
    """将日志条目格式化为人类可读字符串"""
    ts = entry["timestamp"]
    ms = ts % 1000
    s = (ts // 1000) % 60
    m = (ts // 60000) % 60
    time_str = f"{m:02d}:{s:02d}.{ms:03d}"

    t = entry["type"]
    a = entry["val_a"]
    b = entry["val_b"]

    if t == 0x00:  # HEARTBEAT
        return f"[{time_str}] ♥ HEARTBEAT"

    elif t == 0x01:  # PID_SPEED
        return f"[{time_str}] 📏 SPEED  L={a / 1000:.3f}  R={b / 1000:.3f} m/s"

    elif t == 0x02:  # PID_DUTY
        return f"[{time_str}] ⚡ DUTY   L={a / 1000:.3f}  R={b / 1000:.3f}"

    elif t == 0x03:  # TARGET_SPEED
        return f"[{time_str}] 🎯 TARGET L={a / 1000:.3f}  R={b / 1000:.3f} m/s"

    elif t == 0x04:  # GYRO_YAW
        return f"[{time_str}] 🧭 GYRO   Yaw={a / 100:.2f}°  Pitch={b / 100:.2f}°"

    elif t == 0x05:  # GRAY_SENSOR
        return f"[{time_str}] 👁 GRAY   raw=0x{a:02X}  miss={b / 100:.2f}°"

    elif t == 0x06:  # STATE_CHANGE
        old = STATE_NAMES.get(b, f"STATE_{b}")
        new = STATE_NAMES.get(a, f"STATE_{a}")
        return f"[{time_str}] 🔄 STATE  {old} → {new}"

    elif t == 0x07:  # BUTTON
        return f"[{time_str}] 🔘 BTN_{a} pressed"

    elif t == 0x08:  # ROTATE_PARAM
        return f"[{time_str}] 🔃 ROTATE error={a / 100:.2f}°  sub={b / 10000:.4f}"

    else:
        return f"[{time_str}] ? TYPE=0x{t:02X} a={a} b={b}"


# =============================================================================
# 实时模式
# =============================================================================


def run_live(port: str, baud: int, csv_file: str | None = None,
             plot: bool = False, quiet: bool = False):
    """从串口实时读取并显示日志"""
    ser = serial.Serial(port, baud, timeout=0.1)
    parser = LogParser()
    csv_fp = open(csv_file, "w", newline="") if csv_file else None
    entries_for_plot: list[dict] = []

    if csv_fp:
        import csv
        writer = csv.writer(csv_fp)
        writer.writerow(["timestamp_ms", "type", "val_a", "val_b", "description"])

    print(f"📡 已连接 {port} @ {baud} baud")
    print(f"   按 Ctrl+C 停止\n")

    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if not data:
                continue

            entries = parser.feed(data)
            for entry in entries:
                if not quiet:
                    print(format_entry(entry))

                if csv_fp:
                    import csv
                    csv_fp.write(f"{entry['timestamp']},{entry['type']},"
                                 f"{entry['val_a']},{entry['val_b']},"
                                 f"\"{format_entry(entry)}\"\n")
                    csv_fp.flush()

                if plot:
                    entries_for_plot.append(entry)

    except KeyboardInterrupt:
        print(f"\n⏹ 停止. 共收到 {parser.frame_count} 条日志, "
              f"{parser.error_count} 条校验错误")

    finally:
        ser.close()
        if csv_fp:
            csv_fp.close()
            print(f"📊 CSV 已保存到 {csv_file}")

        if plot and entries_for_plot:
            plot_entries(entries_for_plot)


# =============================================================================
# 回放模式
# =============================================================================


def run_replay(filepath: str, quiet: bool = False):
    """从文件回放日志"""
    with open(filepath, "rb") as f:
        data = f.read()

    parser = LogParser()
    entries = parser.feed(data)

    print(f"📂 从 {filepath} 回放 ({len(entries)} 条日志)\n")

    for entry in entries:
        if not quiet:
            print(format_entry(entry))

    # 统计
    by_type = defaultdict(int)
    for e in entries:
        by_type[e["type_name"]] += 1

    print(f"\n📊 统计:")
    for tname, count in sorted(by_type.items()):
        print(f"   {tname}: {count}")


# =============================================================================
# 绘图模式
# =============================================================================


def plot_entries(entries: list[dict]):
    """用 matplotlib 绘制速度/占空比/偏航角趋势图"""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("⚠ matplotlib 未安装, 跳过绘图. pip install matplotlib")
        return

    # 按类型分组
    speeds = [e for e in entries if e["type"] == 0x01]
    duties = [e for e in entries if e["type"] == 0x02]
    targets = [e for e in entries if e["type"] == 0x03]
    gyros = [e for e in entries if e["type"] == 0x04]
    states = [e for e in entries if e["type"] == 0x06]

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # 图 1: 速度 (目标 vs 实际)
    ax1 = axes[0]
    if speeds:
        t = [s["timestamp"] / 1000.0 for s in speeds]
        ax1.plot(t, [s["val_a"] / 1000.0 for s in speeds], "b-", label="Left Speed", alpha=0.7)
        ax1.plot(t, [s["val_b"] / 1000.0 for s in speeds], "b--", label="Right Speed", alpha=0.7)
    if targets:
        t = [s["timestamp"] / 1000.0 for s in targets]
        ax1.plot(t, [s["val_a"] / 1000.0 for s in targets], "r-", label="Left Target", alpha=0.5)
        ax1.plot(t, [s["val_b"] / 1000.0 for s in targets], "r--", label="Right Target", alpha=0.5)
    ax1.set_ylabel("Speed (m/s)")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)
    ax1.set_title("Car Runtime Log")

    # 图 2: 占空比
    ax2 = axes[1]
    if duties:
        t = [d["timestamp"] / 1000.0 for d in duties]
        ax2.plot(t, [d["val_a"] / 1000.0 for d in duties], "g-", label="Left Duty")
        ax2.plot(t, [d["val_b"] / 1000.0 for d in duties], "g--", label="Right Duty")
    ax2.set_ylabel("Duty Cycle")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    # 图 3: 偏航角 + 状态
    ax3 = axes[2]
    if gyros:
        t = [g["timestamp"] / 1000.0 for g in gyros]
        ax3.plot(t, [g["val_a"] / 100.0 for g in gyros], "m-", label="Yaw (°)")
    if states:
        for s in states:
            ax3.axvline(x=s["timestamp"] / 1000.0, color="orange", alpha=0.5,
                        linestyle=":", label="State Change" if s == states[0] else "")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Yaw (degrees)")
    ax3.legend(loc="upper right")
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


# =============================================================================
# main
# =============================================================================

def print_usage():
    print(__doc__)


if __name__ == "__main__":
    args = sys.argv[1:]

    plot_mode = "--plot" in args
    csv_file = None
    replay_mode = "--replay" in args
    quiet = "--quiet" in args

    # 提取 --csv 参数
    if "--csv" in args:
        idx = args.index("--csv")
        if idx + 1 < len(args):
            csv_file = args[idx + 1]

    # 过滤掉标志参数
    positional = [a for a in args if not a.startswith("--") and a not in (
        csv_file or "",)]

    if len(positional) < 1:
        print_usage()
        sys.exit(1)

    if replay_mode:
        run_replay(positional[0], quiet=quiet)
    else:
        port = positional[0]
        baud = int(positional[1]) if len(positional) > 1 else 115200
        run_live(port, baud, csv_file=csv_file, plot=plot_mode, quiet=quiet)
