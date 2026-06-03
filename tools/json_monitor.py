#!/usr/bin/env python3
"""
JSON Monitor — 从串口读取 ESP32 Remote ID Scanner 的 JSON 输出并实时展示

双端口模式：
  - 数据端口（UART1 GPIO17）：纯净的 UAV 解析数据流
  - 调试端口（USB CDC）：启动信息、调试、告警、错误

用法:
    python3 json_monitor.py [--port PORT] [--baud BAUD]
    python3 json_monitor.py --mode data    # 仅显示 UAV 数据（默认）
    python3 json_monitor.py --mode debug   # 仅显示调试信息
    python3 json_monitor.py --mode all     # 显示所有事件

默认: --port /dev/cu.usbmodem* (自动检测) --baud 115200 --mode data
"""

import argparse
import json
import sys
import time
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("需要 pyserial: pip install pyserial")
    sys.exit(1)


# ── 颜色支持 ──────────────────────────────────────────────

class Colors:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"
    GRAY = "\033[90m"


# ── 事件类型对应的颜色和图标 ───────────────────────────────

EVT_STYLE = {
    "startup":       ("🚀", Colors.CYAN + Colors.BOLD),
    "status":        ("📊", Colors.BLUE + Colors.BOLD),
    "uav_discovery": ("🆕", Colors.GREEN + Colors.BOLD),
    "uav_update":    ("📡", Colors.GRAY),
    "uav_status":    ("💚", Colors.GREEN),
    "uav_detail":    ("📋", Colors.MAGENTA),
    "uav_timeout":   ("⏰", Colors.YELLOW),
    "warning":       ("⚠️ ", Colors.YELLOW),
    "error":         ("❌", Colors.RED + Colors.BOLD),
    "debug":         ("🔧", Colors.GRAY),
    "decode_fail":   ("❓", Colors.RED),
}


def format_ts(ms):
    """将毫秒时间戳转为 HH:MM:SS"""
    s = ms / 1000.0
    return datetime.utcfromtimestamp(s).strftime("%H:%M:%S")


def print_separator(char="─", width=80):
    print(Colors.GRAY + char * width + Colors.RESET)


def print_uav_update(obj):
    """格式化打印 uav_update 事件"""
    mac = obj.get("mac", "?")
    rssi = obj.get("rssi", "?")
    ch = obj.get("channel", "?")
    cnt = obj.get("msg_count", 0)
    transport = obj.get("transport", "?")
    protocol = obj.get("protocol", "?")

    # 基本信息行
    print(f"  {Colors.CYAN}MAC:{Colors.RESET} {mac}  "
          f"{Colors.YELLOW}RSSI:{Colors.RESET} {rssi} dBm  "
          f"ch:{ch}  "
          f"msgs:{cnt}  "
          f"{transport} / {protocol}")

    # Basic ID
    bid = obj.get("basic_id")
    if bid:
        id_type = bid.get("id_type", "?")
        ua_type = bid.get("ua_type", "?")
        uas_id = bid.get("uas_id", "?")
        if uas_id:
            print(f"  🆔 {id_type} | {ua_type} | ID: {Colors.BOLD}{uas_id}{Colors.RESET}")

    # Location
    loc = obj.get("location")
    if loc:
        lat = loc.get("latitude")
        lng = loc.get("longitude")
        alt = loc.get("alt_baro") or loc.get("alt_geo")
        hgt = loc.get("height")
        spd = loc.get("speed_h")
        direction = loc.get("direction")
        status = loc.get("status", "?")

        parts = []
        if lat is not None and lng is not None:
            parts.append(f"📍 {lat:.6f}, {lng:.6f}")
        if alt is not None:
            parts.append(f"alt={alt:.1f}m")
        if hgt is not None:
            parts.append(f"h={hgt:.1f}m")
        if spd is not None:
            parts.append(f"{spd:.1f}m/s")
        if direction is not None:
            parts.append(f"→{direction:.0f}°")
        parts.append(f"[{status}]")
        if parts:
            print(f"  {'  '.join(parts)}")

    # Operator ID
    oid = obj.get("operator_id")
    if oid and oid.get("id"):
        print(f"  👤 Operator: {oid.get('id')}")

    # Self ID
    sid = obj.get("self_id")
    if sid and sid.get("desc"):
        print(f"  📝 Self ID: {sid.get('desc')}")

    # Auth
    auth_list = obj.get("auth", [])
    if auth_list:
        print(f"  🔐 Auth pages: {len(auth_list)}")

    # System
    sys_data = obj.get("system")
    if sys_data:
        lat = sys_data.get("operator_lat")
        lng = sys_data.get("operator_lon")
        if lat is not None and lng is not None:
            print(f"  🧑 Operator pos: {lat:.6f}, {lng:.6f}")


def print_status(obj):
    """格式化打印 status 事件"""
    loop = obj.get("loop_min", "?")
    heap = obj.get("free_heap", 0)
    active = obj.get("active_uavs", 0)
    total = obj.get("total_pkts", 0)
    rate = obj.get("pkt_rate", 0)
    rid = obj.get("rid_pkts", 0)
    rid_rate = obj.get("rid_rate", 0)
    beacons = obj.get("beacon_count", 0)
    beacon_rate = obj.get("beacon_rate", 0)
    overflows = obj.get("queue_overflows", 0)

    print(f"  ⏱  运行 {loop} min  "
          f"💾 空闲堆: {heap // 1024} KB  "
          f"🛸 活跃 UAV: {active}")
    print(f"  📦 总包: {total} ({rate:.1f}/s)  "
          f"📡 RID: {rid} ({rid_rate:.1f}/s)  "
          f"📶 Beacon: {beacons} ({beacon_rate:.1f}/s)")
    if overflows > 0:
        print(f"  {Colors.RED}⚠ 队列溢出: {overflows}{Colors.RESET}")


def print_uav_status(obj):
    """格式化打印 uav_status 事件"""
    mac = obj.get("mac", "?")
    age = obj.get("age_ms", 0)
    rssi = obj.get("rssi", "?")
    ch = obj.get("channel", "?")
    cnt = obj.get("msg_count", 0)
    print(f"  {Colors.CYAN}{mac}{Colors.RESET}  "
          f"age={age / 1000:.0f}s  RSSI={rssi}  ch={ch}  msgs={cnt}")


def print_startup(obj):
    """格式化打印 startup 事件"""
    name = obj.get("name", "?")
    ver = obj.get("version", "?")
    target = obj.get("target", "?")
    idf = obj.get("idf_version", "?")
    heap = obj.get("free_heap", 0)
    print(f"  {Colors.BOLD}{name} v{ver}{Colors.RESET}")
    print(f"  芯片: {target}  IDF: {idf}  空闲堆: {heap // 1024} KB")


# 数据端口事件（UAV 解析数据）
DATA_EVENTS = {"uav_discovery", "uav_update", "uav_status", "uav_timeout", "uav_detail", "status"}

# 调试端口事件（启动、诊断）
DEBUG_EVENTS = {"startup", "warning", "error", "debug", "decode_fail"}

def handle_json(line, mode):
    """处理一行 JSON"""
    try:
        obj = json.loads(line)
    except json.JSONDecodeError:
        stripped = line.strip()
        if stripped:
            print(Colors.GRAY + stripped + Colors.RESET)
        return

    evt = obj.get("evt", "unknown")

    # 按模式过滤
    if mode == "data" and evt in DEBUG_EVENTS:
        return
    if mode == "debug" and evt in DATA_EVENTS:
        return

    ts = obj.get("ts", 0)
    style = EVT_STYLE.get(evt, ("", Colors.RESET))
    icon, color = style

    # 数据事件在数据模式下不打印标题行，节省空间
    if mode == "debug" or evt in DEBUG_EVENTS or evt in ("uav_discovery", "status"):
        print()
        print(f"{color}{icon} [{evt}] @ {format_ts(ts)}{Colors.RESET}")

    if evt == "uav_update":
        print_uav_update(obj)
    elif evt == "uav_discovery":
        print(Colors.GREEN + Colors.BOLD + "  ═══ 新无人机发现 ═══" + Colors.RESET)
        print_uav_update(obj)
        print_separator()
    elif evt == "status":
        print_separator()
        print_status(obj)
        print_separator()
    elif evt == "uav_status":
        print_uav_status(obj)
    elif evt == "uav_timeout":
        mac = obj.get("mac", "?")
        print(f"  {Colors.YELLOW}{mac} 已超时移除{Colors.RESET}")
    elif evt == "uav_detail":
        print_uav_update(obj)
    elif evt == "startup":
        print_separator("═")
        print_startup(obj)
        print_separator("═")
    elif evt in ("warning", "error"):
        module = obj.get("module", "?")
        msg = obj.get("msg", "")
        print(f"  [{module}] {msg}")
    elif evt == "debug":
        module = obj.get("module", "?")
        msg = obj.get("msg", "")
        print(f"  [{module}] {msg}")
    elif evt == "decode_fail":
        byte0 = obj.get("byte0", "?")
        byte1 = obj.get("byte1", "?")
        length = obj.get("length", "?")
        print(f"  解码失败: byte0=0x{byte0:02X} byte1=0x{byte1:02X} len={length}")


def find_esp32_port():
    """自动查找 ESP32 串口"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # macOS: cu.usbmodem* 或 cu.usbserial*
        if "usbmodem" in port.device or "usbserial" in port.device:
            return port.device
        # 检查描述
        desc = (port.description or "") + (port.manufacturer or "")
        if "CP210" in desc or "CH340" in desc or "CH9102" in desc or "Silicon Labs" in desc:
            return port.device
    return None


def main():
    parser = argparse.ArgumentParser(
        description="ESP32 Remote ID Scanner — JSON 串口监视器"
    )
    parser.add_argument("--port", "-p", help="串口路径 (如 /dev/cu.usbmodem*)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="波特率 (默认: 115200)")
    parser.add_argument("--mode", "-m", choices=["data", "debug", "all"], default="data",
                        help="显示模式: data(仅UAV数据), debug(仅调试), all(全部) (默认: data)")
    parser.add_argument("--raw", action="store_true", help="原始输出模式 (不格式化)")
    args = parser.parse_args()

    port = args.port
    if not port:
        port = find_esp32_port()
        if not port:
            print("未找到 ESP32 串口，请用 --port 指定")
            print("可用串口:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device} — {p.description}")
            sys.exit(1)
        print(f"自动检测到串口: {port}")

    print(f"连接 {port} @ {args.baud} baud ...")
    try:
        ser = serial.Serial(port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"无法打开串口: {e}")
        sys.exit(1)

    print(f"已连接，模式: {args.mode}，等待 JSON 数据... (Ctrl+C 退出)")
    print()

    # 丢弃连接时的残留数据
    ser.reset_input_buffer()

    buffer = ""
    try:
        while True:
            try:
                data = ser.read(ser.in_waiting or 1).decode("utf-8", errors="replace")
            except serial.SerialException:
                time.sleep(0.1)
                continue

            if not data:
                continue

            buffer += data
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                if args.raw:
                    print(line)
                else:
                    handle_json(line, args.mode)

    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}已退出{Colors.RESET}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
