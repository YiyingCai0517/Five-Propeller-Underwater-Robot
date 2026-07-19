#!/usr/bin/env python3
"""
水下机器人上位机通信脚本

功能:
  1. 发送指令: hover / move / stop / turn / square
  2. 接收并解析遥测帧 (含速度、推力等新增字段)
  3. 交互式命令行界面 + 实时遥测显示

协议说明:
  发送格式 (PC→STM32):  [定点头 3B] + [AA 55 FuncID PayloadLen Payload... Checksum]
  接收格式 (STM32→PC):  [AA 55 0x80 PayloadLen Payload... Checksum]

LoRa 定点头: 目标地址(2B) + 信道(1B)
  PC 端发送时加 STM32 的定点头 (默认 0x00 0x01 0x17)
  PC 端接收时 LoRa 模块已去掉定点头, 直接从 AA 55 开始

指令列表:
  FuncID 0x01  HOVER   定深悬停      payload: float depth(m)
  FuncID 0x02  MOVE    直航          payload: float thrust(-1~+1)
  FuncID 0x03  STOP    急停          payload: 无
  FuncID 0x04  TURN    转向          payload: float target_yaw(deg)
  FuncID 0x05  SQUARE  方形轨迹      payload: float edge_time(s) + float surge(-1~+1)

遥测 Payload (33B):
  mode(1B) + depth(4B) + yaw(4B) + pitch(4B) + roll(4B)
  + target_depth(4B) + target_yaw(4B) + speed(4B) + surge(4B)
"""

import serial
import struct
import time
import threading
import sys
import os
from datetime import datetime

# ======================== 配置 ========================
SERIAL_PORT = "COM6"      # 根据实际端口修改
BAUD_RATE = 9600

# LoRa 定点头: 目标=STM32 (地址 0x00 0x01, 信道 0x17)
LORA_STM32_ADDR_H = 0x00
LORA_STM32_ADDR_L = 0x01
LORA_CHANNEL = 0x17
LORA_STM32_HEADER = bytes([LORA_STM32_ADDR_H, LORA_STM32_ADDR_L, LORA_CHANNEL])

# 模式名称映射
MODE_NAMES = {
    0: "IDLE",
    1: "HOVER",
    2: "AUTO",
    3: "TURN",
    4: "SQUARE",
}


# ======================== 校验和计算 ========================
def calc_checksum(data: bytes) -> int:
    """计算从 AA 开始的累加校验和 (低 8 位)"""
    return sum(data) & 0xFF


# ======================== 构建指令帧 ========================
def _build_frame(func_id: int, payload: bytes = b'') -> bytes:
    payload_len = len(payload)
    app_frame = bytes([0xAA, 0x55, func_id, payload_len]) + payload
    checksum = calc_checksum(app_frame)
    app_frame += bytes([checksum])
    return LORA_STM32_HEADER + app_frame

def build_hover_cmd(target_depth: float) -> bytes:
    return _build_frame(0x01, struct.pack('<f', target_depth))

def build_move_cmd(thrust: float) -> bytes:
    thrust = max(-1.0, min(1.0, thrust))
    return _build_frame(0x02, struct.pack('<f', thrust))

def build_stop_cmd() -> bytes:
    return _build_frame(0x03)

def build_turn_cmd(target_yaw: float) -> bytes:
    return _build_frame(0x04, struct.pack('<f', target_yaw))

def build_square_cmd(edge_time: float, surge: float) -> bytes:
    surge = max(-1.0, min(1.0, surge))
    payload = struct.pack('<ff', edge_time, surge)
    return _build_frame(0x05, payload)


# ======================== 解析遥测帧 ========================
def parse_telemetry(data: bytes) -> dict | None:
    idx = -1
    for i in range(len(data) - 1):
        if data[i] == 0xAA and data[i + 1] == 0x55:
            idx = i
            break

    if idx == -1:
        return None

    remaining = data[idx:]
    if len(remaining) < 4:
        return None

    func_id = remaining[2]
    payload_len = remaining[3]

    if func_id != 0x80:  # 不是遥测帧
        return None

    if len(remaining) < 4 + payload_len + 1:
        return None

    # 校验和验证
    frame_data = remaining[:4 + payload_len]
    expected_checksum = calc_checksum(frame_data)
    actual_checksum = remaining[4 + payload_len]

    if expected_checksum != actual_checksum:
        return None  # 校验和错误, 静默丢弃

    # 解析 Payload (33 字节)
    payload = remaining[4: 4 + payload_len]
    if len(payload) < 33:
        return None

    mode = payload[0]
    fields = struct.unpack('<ffffffff', payload[1:33])
    depth, yaw, pitch, roll, tgt_depth, tgt_yaw, speed, surge = fields

    return {
        "mode": MODE_NAMES.get(mode, f"?({mode})"),
        "mode_id": mode,
        "depth": depth,
        "yaw": yaw,
        "pitch": pitch,
        "roll": roll,
        "target_depth": tgt_depth,
        "target_yaw": tgt_yaw,
        "speed": speed,
        "surge": surge,
    }


# ======================== 数据记录 ========================
class DataLogger:
    """将遥测数据记录到 CSV 文件"""

    def __init__(self, enabled: bool = False):
        self.enabled = enabled
        self.file = None
        self.writer = None

    def start(self, filename: str = None):
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"telemetry_log_{timestamp}.csv"
        self.file = open(filename, 'w', encoding='utf-8')
        header = "timestamp,mode,depth,yaw,pitch,roll,target_depth,target_yaw,speed,surge\n"
        self.file.write(header)
        self.enabled = True
        return filename

    def log(self, telem: dict):
        if not self.enabled or not self.file:
            return
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = (f"{ts},{telem['mode']},{telem['depth']:.4f},{telem['yaw']:.2f},"
                f"{telem['pitch']:.2f},{telem['roll']:.2f},{telem['target_depth']:.4f},"
                f"{telem['target_yaw']:.2f},{telem['speed']:.4f},{telem['surge']:.4f}\n")
        self.file.write(line)
        self.file.flush()

    def stop(self):
        if self.file:
            self.file.close()
            self.file = None
        self.enabled = False


# ======================== 接收线程 ========================
def receive_thread(ser: serial.Serial, running: threading.Event, logger: DataLogger):
    """后台线程: 持续接收并解析遥测"""
    buf = bytearray()
    telem_count = 0

    while running.is_set():
        try:
            if ser.in_waiting > 0:
                buf.extend(ser.read(ser.in_waiting))

                # 尝试从缓冲区中解析完整帧
                while len(buf) >= 38:  # 最小遥测帧: AA(1)+55(1)+FuncID(1)+Len(1)+Payload(33)+Checksum(1) = 38
                    # 找帧头
                    idx = -1
                    for i in range(len(buf) - 1):
                        if buf[i] == 0xAA and buf[i + 1] == 0x55:
                            idx = i
                            break

                    if idx == -1:
                        buf.clear()
                        break

                    # 丢弃帧头之前的garbagedata
                    if idx > 0:
                        buf = buf[idx:]

                    if len(buf) < 4:
                        break

                    payload_len = buf[3]
                    frame_total = 4 + payload_len + 1

                    if len(buf) < frame_total:
                        break  # 数据不完整, 等更多数据

                    frame = bytes(buf[:frame_total])
                    buf = buf[frame_total:]

                    result = parse_telemetry(frame)
                    if result:
                        telem_count += 1
                        logger.log(result)

                        # 格式化输出
                        mode_str = f"{result['mode']:>6s}"
                        print(f"\r📡 [{telem_count:>4d}] "
                              f"模式={mode_str} | "
                              f"深度={result['depth']:+.3f}m | "
                              f"Yaw={result['yaw']:+7.1f}° | "
                              f"Pitch={result['pitch']:+6.1f}° | "
                              f"Roll={result['roll']:+6.1f}° | "
                              f"速度={result['speed']:+.3f}m/s | "
                              f"推力={result['surge']:+.2f}",
                              end='', flush=True)
            else:
                time.sleep(0.01)
        except (serial.SerialException, OSError):
            break


# ======================== 打印帮助 ========================
def print_help():
    print()
    print("=" * 70)
    print("  水下机器人上位机控制台 — 可用命令")
    print("=" * 70)
    print("  hover <depth>          定深悬停, 如: hover 0.15 (15cm)")
    print("  move <thrust>          直航,     如: move 0.3")
    print("  stop                   急停")
    print("  turn <yaw>             转向,     如: turn 90 (转到90°)")
    print("  square <time> <surge>  方形轨迹, 如: square 5 0.3")
    print("                           (每边前进5秒, 推力0.3)")
    print("  log [start|stop]       开始/停止数据记录到CSV文件")
    print("  raw                    显示原始 HEX (调试用)")
    print("  help                   显示本帮助")
    print("  quit                   退出")
    print("=" * 70)


# ======================== 主程序 ========================
def main():
    # 支持命令行传入串口
    port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD_RATE

    print(f"正在打开串口 {port} @ {baud}...")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"❌ 无法打开串口: {e}")
        print(f"   请检查端口名称或设备连接。")
        print(f"   用法: python {os.path.basename(__file__)} [COM端口] [波特率]")
        return

    print(f"✅ 串口已打开 ({port} @ {baud})")
    print()

    # 数据记录器
    logger = DataLogger()

    # 启动接收线程
    running = threading.Event()
    running.set()
    rx_thread = threading.Thread(target=receive_thread,
                                 args=(ser, running, logger),
                                 daemon=True)
    rx_thread.start()

    print_help()

    try:
        while True:
            try:
                cmd = input("\n> ").strip()
            except EOFError:
                break

            if not cmd:
                continue

            parts = cmd.split()
            verb = parts[0].lower()

            if verb == "hover":
                depth = float(parts[1]) if len(parts) > 1 else 0.15
                frame = build_hover_cmd(depth)
                ser.write(frame)
                print(f"  ✅ HOVER 指令已发送, 目标深度={depth:.3f}m")
                print(f"     HEX: {frame.hex(' ')}")

            elif verb == "move":
                thrust = float(parts[1]) if len(parts) > 1 else 0.3
                thrust = max(-1.0, min(1.0, thrust))
                frame = build_move_cmd(thrust)
                ser.write(frame)
                print(f"  ✅ MOVE 指令已发送, 推力={thrust:+.2f}")
                print(f"     HEX: {frame.hex(' ')}")

            elif verb == "stop":
                frame = build_stop_cmd()
                ser.write(frame)
                print(f"  ✅ STOP 指令已发送")
                print(f"     HEX: {frame.hex(' ')}")

            elif verb == "turn":
                yaw = float(parts[1]) if len(parts) > 1 else 0.0
                frame = build_turn_cmd(yaw)
                ser.write(frame)
                print(f"  ✅ TURN 指令已发送, 目标航向={yaw:+.1f}°")
                print(f"     HEX: {frame.hex(' ')}")

            elif verb == "square":
                edge_time = float(parts[1]) if len(parts) > 1 else 5.0
                surge = float(parts[2]) if len(parts) > 2 else 0.3
                surge = max(-1.0, min(1.0, surge))
                frame = build_square_cmd(edge_time, surge)
                ser.write(frame)
                total_time = edge_time * 4 + 10  # 4边 + 转弯预留
                print(f"  ✅ SQUARE 指令已发送")
                print(f"     每边={edge_time:.1f}s, 推力={surge:+.2f}")
                print(f"     预计总时间≈{total_time:.0f}s")
                print(f"     HEX: {frame.hex(' ')}")

            elif verb == "log":
                if len(parts) < 2 or parts[1].lower() == "start":
                    fn = logger.start()
                    print(f"  📝 数据记录已开始 → {fn}")
                elif parts[1].lower() == "stop":
                    logger.stop()
                    print(f"  📝 数据记录已停止")
                else:
                    print(f"  用法: log [start|stop]")

            elif verb == "raw":
                time.sleep(0.5)
                raw = ser.read(ser.in_waiting) if ser.in_waiting > 0 else b''
                print(f"  原始数据 ({len(raw)}B): {raw.hex(' ') if raw else '(空)'}")

            elif verb == "help":
                print_help()

            elif verb in ("quit", "exit", "q"):
                break

            else:
                print(f"  ❓ 未知命令: {verb} (输入 help 查看帮助)")

    except KeyboardInterrupt:
        pass
    finally:
        print("\n正在关闭...")
        running.clear()
        logger.stop()
        ser.close()
        print("✅ 串口已关闭, 程序退出。")


if __name__ == "__main__":
    main()