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
    python3 json_monitor.py --mode static  # 仅显示无人机静态消息（basic_id/self_id/operator_id/system/auth）
    python3 json_monitor.py --mode all     # 显示所有事件

默认: --port /dev/cu.usbmodem* (自动检测) --baud 115200 --mode data
"""

import argparse
import json
import sys
import time
from datetime import datetime, timezone

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
    return datetime.fromtimestamp(s, tz=timezone.utc).strftime("%H:%M:%S")


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


# ── 静态消息去重追踪 ──────────────────────────────────────

# 记录每个 MAC 已打印过的静态字段 hash，避免重复输出
_static_seen = {}  # mac -> set of field names already printed

def print_static_msg(obj, force=False):
    """仅打印 UAV 的静态消息部分，已打印过的跳过（除非 force=True）"""
    mac = obj.get("mac", "?")

    if mac not in _static_seen:
        _static_seen[mac] = set()

    # 检查是否有新的静态字段
    current_keys = set()
    if obj.get("basic_id") is not None:
        current_keys.add("basic_id")
    if obj.get("self_id") is not None:
        current_keys.add("self_id")
    if obj.get("operator_id") is not None:
        current_keys.add("operator_id")
    if obj.get("system") is not None:
        current_keys.add("system")
    if obj.get("auth"):
        current_keys.add("auth")

    new_keys = current_keys - _static_seen[mac]
    if not new_keys and not force:
        return  # 所有静态字段都已打印过

    _static_seen[mac].update(current_keys)

    # 打印标题
    ts = obj.get("ts", 0)
    print()
    if force:
        print(f"{Colors.BOLD}{Colors.MAGENTA}📋 [static] @ {format_ts(ts)} — {mac} (全部){Colors.RESET}")
    else:
        print(f"{Colors.BOLD}{Colors.MAGENTA}📋 [static] @ {format_ts(ts)} — {mac} (新增: {', '.join(sorted(new_keys))}){Colors.RESET}")

    # MAC / 基本行
    rssi = obj.get("rssi", "?")
    ch = obj.get("channel", "?")
    transport = obj.get("transport", "?")
    protocol = obj.get("protocol", "?")
    print(f"  {Colors.CYAN}MAC:{Colors.RESET} {mac}  "
          f"{Colors.YELLOW}RSSI:{Colors.RESET} {rssi}  "
          f"ch:{ch}  {transport} / {protocol}")

    # Basic ID
    bid = obj.get("basic_id")
    if bid and "basic_id" in current_keys:
        id_type = bid.get("id_type", "?")
        ua_type = bid.get("ua_type", "?")
        uas_id = bid.get("uas_id", "?")
        if uas_id:
            print(f"  🆔 Basic ID: type={id_type}  ua_type={ua_type}")
            print(f"     UAS ID: {Colors.BOLD}{Colors.GREEN}{uas_id}{Colors.RESET}")

    # Self ID
    sid = obj.get("self_id")
    if sid and "self_id" in current_keys:
        desc = sid.get("desc", "?")
        stype = sid.get("type", "?")
        if desc:
            print(f"  📝 Self ID: [{stype}] {desc}")

    # Operator ID
    oid = obj.get("operator_id")
    if oid and "operator_id" in current_keys:
        op_id = oid.get("id", "?")
        op_type = oid.get("type", "?")
        if op_id:
            print(f"  👤 Operator ID: [{op_type}] {Colors.BOLD}{op_id}{Colors.RESET}")

    # System (操作员位置、区域等)
    sys_data = obj.get("system")
    if sys_data and "system" in current_keys:
        op_lat = sys_data.get("operator_lat")
        op_lon = sys_data.get("operator_lon")
        op_alt = sys_data.get("operator_alt_geo")
        loc_type = sys_data.get("operator_loc_type", "?")
        area_count = sys_data.get("area_count", 0)
        area_radius = sys_data.get("area_radius", 0)
        area_ceiling = sys_data.get("area_ceiling")
        area_floor = sys_data.get("area_floor")
        classification = sys_data.get("classification")
        cat_eu = sys_data.get("category_eu")
        cls_eu = sys_data.get("class_eu")

        parts = []
        if op_lat is not None and op_lon is not None:
            parts.append(f"📍 操作员: {op_lat:.7f}, {op_lon:.7f}")
            if op_alt is not None:
                parts.append(f"alt={op_alt:.1f}m")
            parts.append(f"[{loc_type}]")
        if classification is not None:
            parts.append(f"分类: {classification}")
        if cat_eu is not None and cat_eu > 0:
            parts.append(f"EU类别: {cat_eu}")
        if cls_eu is not None and cls_eu > 0:
            parts.append(f"EU等级: {cls_eu}")
        if area_count > 0:
            parts.append(f"区域×{area_count} r={area_radius}m")
            if area_ceiling is not None:
                parts.append(f"上限={area_ceiling:.0f}m")
            if area_floor is not None:
                parts.append(f"下限={area_floor:.0f}m")
        if parts:
            print(f"  🧑 System: {'  '.join(parts)}")

    # Auth
    auth_list = obj.get("auth", [])
    if auth_list and "auth" in current_keys:
        print(f"  🔐 Auth: {len(auth_list)} page(s)")
        for a in auth_list:
            page = a.get("page", "?")
            atype = a.get("type", "?")
            extra = ""
            if a.get("last_page") is not None:
                extra = f" last_page={a['last_page']} len={a.get('length', '?')}"
            print(f"     page={page} type={atype}{extra}")

    print_separator("─", 60)


# ── 全局统计状态（用于退出时的摘要） ────────────────────────

class SummaryState:
    def __init__(self):
        self.start_time = time.time()
        self.start_ts_ms = 0
        self.uavs = {}
        self.uav_first_seen = {}
        self.uav_last_seen = {}
        self.discovery_count = 0
        self.timeout_count = 0
        self.decode_fails = 0
        self.warning_count = 0
        self.error_count = 0
        self.last_status = None
        self.total_uav_updates = 0

    def track(self, evt, obj):
        ts = obj.get("ts", 0)
        if evt == "uav_update":
            mac = obj.get("mac", "")
            if mac:
                if mac not in self.uav_first_seen:
                    self.uav_first_seen[mac] = time.time()
                self.uavs[mac] = obj
                self.uav_last_seen[mac] = time.time()
                self.total_uav_updates += 1
        elif evt == "uav_discovery":
            mac = obj.get("mac", "")
            if mac:
                self.uav_first_seen[mac] = time.time()
                self.uavs[mac] = obj
                self.uav_last_seen[mac] = time.time()
            self.discovery_count += 1
        elif evt == "uav_timeout":
            self.timeout_count += 1
        elif evt == "status":
            self.last_status = obj
        elif evt == "decode_fail":
            self.decode_fails += 1
        elif evt == "warning":
            self.warning_count += 1
        elif evt == "error":
            self.error_count += 1
        elif evt == "startup":
            self.start_ts_ms = ts

    def get_run_duration(self):
        secs = time.time() - self.start_time
        if secs < 60:
            return f"{secs:.0f}s"
        elif secs < 3600:
            return f"{secs // 60:.0f}m {secs % 60:.0f}s"
        else:
            h = secs // 3600
            m = (secs % 3600) // 60
            return f"{h:.0f}h {m:.0f}m"

summary = SummaryState()


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

    # 追踪统计
    summary.track(evt, obj)

    # 按模式过滤
    if mode == "data" and evt in DEBUG_EVENTS:
        return
    if mode == "debug" and evt in DATA_EVENTS:
        return
    if mode == "static":
        # 仅处理 uav_update / uav_discovery 中的静态消息
        if evt in ("uav_update", "uav_discovery"):
            print_static_msg(obj)
        elif evt == "uav_timeout":
            mac = obj.get("mac", "?")
            if mac in _static_seen:
                del _static_seen[mac]
            print(f"\n{Colors.YELLOW}⏰ [uav_timeout] @ {format_ts(obj.get('ts', 0))}{Colors.RESET}")
            print(f"  {mac} 已超时移除")
        elif evt == "status":
            print()
            print(f"{Colors.BLUE}{Colors.BOLD}📊 [status] @ {format_ts(obj.get('ts', 0))}{Colors.RESET}")
            print_status(obj)
            print_separator()
        elif evt in ("warning", "error"):
            print()
            icon = "⚠️" if evt == "warning" else "❌"
            color = Colors.YELLOW if evt == "warning" else Colors.RED
            print(f"{color}{icon} [{evt}] @ {format_ts(obj.get('ts', 0))} — {obj.get('module', '?')}{Colors.RESET}")
            print(f"  {obj.get('msg', '')}")
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
        byte0 = obj.get("byte0", 0)
        byte1 = obj.get("byte1", 0)
        length = obj.get("length", 0)
        if isinstance(byte0, int) and isinstance(byte1, int):
            print(f"  解码失败: byte0=0x{byte0:02X} byte1=0x{byte1:02X} len={length}")
        else:
            print(f"  解码失败: byte0={byte0} byte1={byte1} len={length}")


def print_summary():
    """退出时打印总结摘要"""
    print()
    print_separator("═")
    print(f"{Colors.BOLD}{Colors.CYAN}  📊 会话摘要{Colors.RESET}")
    print_separator("═")

    # 运行时长
    print(f"  ⏱  运行时长: {Colors.BOLD}{summary.get_run_duration()}{Colors.RESET}")

    # 无人机统计
    total_uavs = len(summary.uavs)
    active_now = sum(1 for mac in summary.uavs if time.time() - summary.uav_last_seen.get(mac, 0) < 300)
    print(f"  🛸 累计发现: {Colors.GREEN}{total_uavs}{Colors.RESET} 架 UAV  "
          f"(当前活跃: {Colors.GREEN}{active_now}{Colors.RESET})")
    print(f"  📡 UAV 更新总数: {summary.total_uav_updates}  "
          f"🆕 发现事件: {summary.discovery_count}  "
          f"⏰ 超时: {summary.timeout_count}")

    # 诊断统计
    if summary.decode_fails or summary.warning_count or summary.error_count:
        parts = []
        if summary.decode_fails:
            parts.append(f"❓ 解码失败: {summary.decode_fails}")
        if summary.warning_count:
            parts.append(f"⚠️  告警: {summary.warning_count}")
        if summary.error_count:
            parts.append(f"❌ 错误: {summary.error_count}")
        print(f"  {'  '.join(parts)}")

    # 全局状态
    if summary.last_status:
        s = summary.last_status
        print()
        print(f"  {Colors.BOLD}全局状态:{Colors.RESET}")
        print(f"    运行 {s.get('loop_min', '?')} min  "
              f"总包: {s.get('total_pkts', 0)} ({s.get('pkts_per_sec', 0):.1f}/s)  "
              f"RID: {s.get('rid_detections', 0)} ({s.get('rid_per_sec', 0):.1f}/s)")
        if s.get('queue_overflows', 0) > 0:
            print(f"    {Colors.RED}队列溢出: {s.get('queue_overflows', 0)}{Colors.RESET}")

    # 各 UAV 详情
    if summary.uavs:
        print()
        print(f"  {Colors.BOLD}无人机详情:{Colors.RESET}")
        # 按首次发现时间排序
        sorted_uavs = sorted(summary.uavs.items(),
                             key=lambda x: summary.uav_first_seen.get(x[0], 0))
        for mac, uav in sorted_uavs:
            first_seen = summary.uav_first_seen.get(mac, 0)
            last_seen = summary.uav_last_seen.get(mac, 0)
            age_sec = time.time() - last_seen if last_seen else 0
            is_active = age_sec < 300

            status_icon = "🟢" if is_active else "🔴"
            mac_display = f"{Colors.CYAN}{mac}{Colors.RESET}"

            # Basic ID
            bid = uav.get("basic_id")
            uas_id = ""
            if bid:
                uas_id = bid.get("uas_id", "")

            # Location
            loc = uav.get("location")
            loc_str = ""
            if loc and loc.get("latitude") is not None:
                loc_str = f"📍 {loc['latitude']:.6f}, {loc['longitude']:.6f}"
                if loc.get("alt_baro") is not None:
                    loc_str += f" alt={loc['alt_baro']:.0f}m"
                if loc.get("speed_h") is not None:
                    loc_str += f" {loc['speed_h']:.1f}m/s"
                if loc.get("direction") is not None:
                    loc_str += f" →{loc['direction']:.0f}°"

            rssi = uav.get("rssi", "?")
            msg_cnt = uav.get("msg_count", 0)

            line = f"  {status_icon} {mac_display}  RSSI={rssi}  msgs={msg_cnt}"
            if uas_id:
                line += f"  ID={Colors.BOLD}{uas_id}{Colors.RESET}"
            if not is_active:
                line += f"  {Colors.YELLOW}({age_sec:.0f}s 未更新){Colors.RESET}"

            print(line)
            if loc_str:
                print(f"      {loc_str}")

            # 首次发现时间
            if first_seen:
                first_str = datetime.fromtimestamp(first_seen).strftime("%H:%M:%S")
                print(f"      {Colors.GRAY}首次: {first_str}{Colors.RESET}")

    print()
    print_separator("═")


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
    parser.add_argument("--mode", "-m", choices=["data", "debug", "static", "all"], default="data",
                        help="显示模式: data(仅UAV数据), debug(仅调试), static(仅静态消息), all(全部) (默认: data)")
    parser.add_argument("--raw", action="store_true", help="原始输出模式 (不格式化)")
    parser.add_argument("--no-summary", action="store_true", help="退出时不显示摘要")
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

    mode_desc = {"data": "UAV 数据", "debug": "调试信息", "static": "静态消息", "all": "全部事件"}
    print(f"已连接，模式: {mode_desc.get(args.mode, args.mode)}，等待 JSON 数据... (Ctrl+C 退出)")
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
        if not args.no_summary:
            print_summary()
    finally:
        ser.close()


if __name__ == "__main__":
    main()
