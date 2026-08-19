# -*- coding: utf-8 -*-
"""
OTA 串口升级工具(文档《07》协议,与 Bootloader 状态机严格对齐)

升级流程:
  1. 握手 0x01 → 0x81:读设备当前版本与活动分区,目标分区自动取对侧
  2. 开始 0x02 → 0x82:传 16B 固件头(版本过低/长度超限会被设备拒绝)
  3. 数据 0x03 → 0x83:256B/包,设备攒满 2KB 页才擦写一次 Flash
     - 应答失败时设备会带"期望包号",工具从该包续传(丢包自动恢复)
     - 单包超时自动重发,3 次失败中止
  4. 结束 0x04 → 0x84:核对已收包数,缺包自动续传
  5. 校验 0x05 → 0x85:设备全片 CRC32 比对
  6. 执行 0x06 → 0x86:设备写标志区后软复位,Boot 复验 CRC32 后切换分区
  7. 等设备重启完成,再次握手确认新版本生效

帧格式: AA 55 | cmd | seq(2B) | len(2B) | data(N) | CRC16(2B) | 0D
  CRC16 = CRC16-MODBUS(0xA001),覆盖 cmd ~ data 末尾
  CRC32 = zlib 标准(与固件头/设备算法一致)

用法:
  python ota_tool.py COM5 fw_package.bin [--baud 115200] [--timeout 5]

前提:设备已进入 Bootloader 升级模式
  - Modbus 写 40020=0x5A5A,或
  - 调试命令 ota,或
  - 上电时两分区都无效自动进入
"""
import argparse
import struct
import sys
import time
import zlib

import serial

# ==================== 协议常量(与 shared/ota_common.h 一致) ====================
FRAME_HEAD1 = 0xAA
FRAME_HEAD2 = 0x55
FRAME_TAIL = 0x0D
FRAME_DATA_MAX = 512

CMD_HANDSHAKE = 0x01
CMD_START = 0x02
CMD_DATA = 0x03
CMD_FINISH = 0x04
CMD_VERIFY = 0x05
CMD_EXECUTE = 0x06
CMD_PROGRESS = 0x07

ACK_OK = 0x00
ACK_VER_LOW = 0x01
ACK_PART_INVALID = 0x02
ACK_LEN_OVER = 0x03
ACK_CRC_FAIL = 0x01   # 0x83/0x85 的失败码复用版本过低值(协议如此定义)

OTA_FW_MAGIC = 0x4D4F4749    # "GWNM"
PACKET_SIZE = 256
RETRY_LIMIT = 3              # 单包最大重发次数


def crc16_modbus(data, init_value=0xFFFF):
    """CRC16-MODBUS 逐字节查位算法(多项式 0xA001,初值 0xFFFF)"""
    crc = init_value
    for byte_value in data:
        crc ^= byte_value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


class OtaClient:
    """串口帧收发:字节流解析(容忍设备调试打印混入)"""

    def __init__(self, port, baud):
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.5)
        except serial.SerialException as exc:
            print(f"[错误] 打开串口 {port} 失败: {exc}")
            sys.exit(1)
        # 关键修复(2026-08-17 实物排查):pyserial 打开端口时默认把 DTR/RTS
        # 拉高,绿深板 CH340 一键下载电路(DTR/RTS 经三极管接 STM32 复位脚)
        # 会把板子按在复位上——工具发的帧全部无应答(手动发帧正常是因为
        # 串口助手打开时电平早已稳定)。打开后立即拉低并等板子启动完成:
        # 若升级标志已置位(upgrade_req=1),Boot 会自动进升级模式。
        self.serial_port.setDTR(False)
        self.serial_port.setRTS(False)
        time.sleep(2)
        self.seq = 0    # 帧序号,每发一帧 +1
        self.stray = bytearray()   # 接收到的非帧杂散字节(设备调试打印),排查用

    def send_frame(self, cmd, data=b""):
        """发一帧:拼帧 → CRC16 → 串口写出"""
        body = bytes([cmd]) + struct.pack("<H", self.seq) \
             + struct.pack("<H", len(data)) + data
        crc_value = crc16_modbus(body)
        frame = bytes([FRAME_HEAD1, FRAME_HEAD2]) + body \
              + struct.pack("<H", crc_value) + bytes([FRAME_TAIL])
        self.serial_port.write(frame)
        self.seq = (self.seq + 1) & 0xFFFF
        return frame

    def recv_frame(self, timeout=5.0):
        """收一帧(字节流状态机):返回 (cmd, seq, data)

        帧头前任何杂散字节(设备调试打印)自动丢弃;
        CRC/帧尾不符的帧也丢弃重找帧头,直到超时抛 TimeoutError
        """
        deadline = time.time() + timeout
        state = 0                # 0=等 0xAA  1=等 0x55  2=收帧体
        body = bytearray()
        body_len = 0

        while time.time() < deadline:
            byte_data = self.serial_port.read(1)
            if not byte_data:
                continue
            byte_value = byte_data[0]

            if state == 0:
                if byte_value == FRAME_HEAD1:
                    state = 1
                else:
                    # 杂散字节:设备调试打印(如 "OTA: verify fail local=...")
                    # 先于应答帧发出,会被这里收集,0x05 失败时打印出来排查
                    self.stray.append(byte_value)
            elif state == 1:
                if byte_value == FRAME_HEAD2:
                    state = 2
                    body = bytearray()
                    body_len = 0
                elif byte_value != FRAME_HEAD1:
                    state = 0       # AA 后跟的不是 55:不是帧头,重新找
                    self.stray.append(byte_value)   # 同上:收集杂散字节
            else:
                body.append(byte_value)
                if len(body) == 5:
                    data_len = body[3] | (body[4] << 8)
                    if data_len > FRAME_DATA_MAX:
                        state = 0   # 长度非法:整帧作废
                        continue
                    body_len = 5 + data_len + 3   # 帧体 = cmd+seq+len+data+crc+tail
                if body_len > 0 and len(body) >= body_len:
                    cmd = body[0]
                    seq = body[1] | (body[2] << 8)
                    data_len = body[3] | (body[4] << 8)
                    crc_received = body[5 + data_len] | (body[6 + data_len] << 8)
                    crc_calc = crc16_modbus(bytes(body[:5 + data_len]))
                    tail = body[7 + data_len]
                    if crc_received == crc_calc and tail == FRAME_TAIL:
                        return cmd, seq, bytes(body[5:5 + data_len])
                    state = 0       # 校验失败:丢弃重找

        raise TimeoutError("接收帧超时")

    def wait_ack(self, cmd, timeout=5.0):
        """等指定命令的应答帧:返回 (seq, data);等不到抛 TimeoutError"""
        ack_cmd = 0x80 | cmd
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = deadline - time.time()
            try:
                got_cmd, seq, data = self.recv_frame(timeout=min(1.0, remaining))
            except TimeoutError:
                continue
            if got_cmd == ack_cmd:
                return seq, data
        raise TimeoutError(f"应答 0x{ack_cmd:02X} 超时")


def do_handshake(client, retry_timeout):
    """0x01 握手:3 次尝试,成功打印设备信息并返回 (当前版本, 活动分区)"""
    for attempt in range(1, 4):
        client.send_frame(CMD_HANDSHAKE, struct.pack("<H", 0))
        try:
            _, data = client.wait_ack(CMD_HANDSHAKE, timeout=retry_timeout)
        except TimeoutError:
            print(f"  握手第 {attempt} 次无应答...")
            continue
        if len(data) < 4:
            print("  [错误] 握手应答数据域不足 4 字节")
            sys.exit(1)
        state = data[0]
        version = (data[1] << 8) | data[2]
        part = chr(data[3]) if chr(data[3]) in ("A", "B") else "?"
        print(f"  设备在线: 状态={state} 当前版本={version >> 8}.{version & 0xFF}"
              f" 分区={part}")
        return version, part
    print("[错误] 握手失败:设备无应答。请确认设备已进升级模式且串口未占用")
    sys.exit(1)


def send_data_packets(client, firmware, total_packets, start_packet=0):
    """0x03 数据包循环:从 start_packet 发到 total_packets-1

    应答处理:
      结果=0 → 设备确认(包号一致或为旧包重复确认) → 下一包
      结果≠0 → 设备带的包号是期望包号 → 从该包续传
    """
    packet = start_packet
    retries = 0
    while packet < total_packets:
        chunk = firmware[packet * PACKET_SIZE:(packet + 1) * PACKET_SIZE]
        payload = struct.pack("<H", packet) + chunk
        client.send_frame(CMD_DATA, payload)
        try:
            _, data = client.wait_ack(CMD_DATA, timeout=5.0)
        except TimeoutError:
            retries += 1
            if retries >= RETRY_LIMIT:
                print(f"[错误] 第 {packet} 包连续 {RETRY_LIMIT} 次无应答,中止")
                sys.exit(1)
            print(f"  第 {packet} 包超时,第 {retries} 次重发...")
            continue
        if len(data) < 3:
            continue
        got_packet = data[0] | (data[1] << 8)
        result = data[2]
        if result == ACK_OK:
            packet += 1         # 确认收下(含旧包重复确认),前进
            retries = 0
            if packet % 40 == 0 or packet == total_packets:
                print(f"  进度: {packet}/{total_packets} 包 ({packet * 100 // total_packets}%)")
        else:
            # 设备拒绝:got_packet 是它期望的包号(丢包续传点)
            print(f"  第 {packet} 包被拒,设备期望第 {got_packet} 包,续传...")
            packet = got_packet
            retries += 1
            if retries >= RETRY_LIMIT:
                print(f"[错误] 续传连续失败 {RETRY_LIMIT} 次,中止")
                sys.exit(1)
    return packet


def main():
    parser = argparse.ArgumentParser(description="STM32F103 智慧农业节点 OTA 升级工具")
    parser.add_argument("port", help="串口号,如 COM5")
    parser.add_argument("package", help="升级包路径(pack_ota.py 生成,16B 头 + 固件)")
    parser.add_argument("--baud", type=int, default=115200, help="波特率(默认 115200)")
    parser.add_argument("--timeout", type=int, default=5, help="应答超时秒数(默认 5)")
    args = parser.parse_args()

    # ---- 读升级包并自校验 ----
    try:
        with open(args.package, "rb") as file_handle:
            package = file_handle.read()
    except OSError:
        print(f"[错误] 打不开升级包: {args.package}")
        sys.exit(1)
    if len(package) < 16:
        print("[错误] 升级包不足 16 字节(缺固件头)")
        sys.exit(1)

    magic, major, minor, patch, _flags, length, crc32_value \
        = struct.unpack("<IBBBBII", package[:16])
    if magic != OTA_FW_MAGIC:
        print("[错误] 固件头魔数不对:不是有效升级包")
        sys.exit(1)
    firmware = package[16:16 + length]
    if len(firmware) != length:
        print(f"[错误] 升级包不完整:头声明 {length} 字节,实际 {len(firmware)} 字节")
        sys.exit(1)
    if (zlib.crc32(firmware) & 0xFFFFFFFF) != crc32_value:
        print("[错误] 升级包 CRC32 校验失败:文件已损坏,请重新打包")
        sys.exit(1)

    total_packets = (length + PACKET_SIZE - 1) // PACKET_SIZE
    print(f"升级包: 版本 {major}.{minor}.{patch}, {length} 字节, {total_packets} 包")

    # ---- 连接设备 ----
    client = OtaClient(args.port, args.baud)

    # ---- 1. 握手 ----
    print("[1/6] 握手...")
    device_version, active_part = do_handshake(client, args.timeout)
    target_part = "B" if active_part == "A" else "A"
    print(f"      目标分区: {target_part}")

    # ---- 2. 固件头 ----
    print("[2/6] 传输固件头...")
    client.send_frame(CMD_START, package[:16])
    _, data = client.wait_ack(CMD_START, timeout=args.timeout)
    if len(data) < 1:
        print("[错误] 0x82 应答无结果码")
        sys.exit(1)
    code = data[0]
    if code == ACK_VER_LOW:
        print("[错误] 设备拒绝:新固件版本低于设备当前版本(禁止降级)")
        sys.exit(1)
    if code == ACK_LEN_OVER:
        print("[错误] 设备拒绝:固件长度超过分区容量")
        sys.exit(1)
    if code != ACK_OK:
        print(f"[错误] 设备拒绝固件头: 结果码 {code}")
        sys.exit(1)
    print("      固件头已接受")

    # ---- 3. 数据包 ----
    print("[3/6] 传输数据包...")
    start_time = time.time()
    send_data_packets(client, firmware, total_packets)
    print(f"      完成,耗时 {time.time() - start_time:.1f}s")

    # ---- 4. 结束并核对 ----
    print("[4/6] 传输结束核对...")
    client.send_frame(CMD_FINISH, struct.pack("<H", total_packets))
    _, data = client.wait_ack(CMD_FINISH, timeout=args.timeout)
    if len(data) < 2:
        print("[错误] 0x84 应答无包数")
        sys.exit(1)
    received = data[0] | (data[1] << 8)
    print(f"      设备已收 {received}/{total_packets} 包")
    if received < total_packets:
        print(f"      缺包,从第 {received} 包续传...")
        send_data_packets(client, firmware, total_packets, start_packet=received)
        client.send_frame(CMD_FINISH, struct.pack("<H", total_packets))
        _, data = client.wait_ack(CMD_FINISH, timeout=args.timeout)
        received = data[0] | (data[1] << 8)
        print(f"      续传后已收 {received}/{total_packets} 包")
        if received < total_packets:
            print("[错误] 续传后仍缺包,中止")
            sys.exit(1)

    # ---- 5. 全片 CRC32 ----
    print("[5/6] 设备全片 CRC32 校验...")
    client.send_frame(CMD_VERIFY, struct.pack("<I", crc32_value))
    _, data = client.wait_ack(CMD_VERIFY, timeout=max(args.timeout, 10))
    if len(data) < 1 or data[0] != ACK_OK:
        # 失败定位:设备先打印 "OTA: verify fail local=xxxxxxxx" 再发应答帧,
        # 打印字节已被 recv_frame 当杂散收进 stray;再多收 0.8s 兜底
        # (打印字节可能比应答帧更晚到齐)。把两段拼起来按 ASCII 打印
        tail_bytes = b""
        deadline = time.time() + 0.8
        while time.time() < deadline:
            tail_bytes += client.serial_port.read(1024)
        debug_text = (bytes(client.stray) + tail_bytes).decode("ascii", errors="replace")
        if debug_text:
            # 只取最后 300 字符,避免握手前的启动日志淹没关键行
            print(f"  [设备打印] ...{debug_text[-300:]}")
        print("[错误] 设备 CRC32 校验不通过(可能是传输损坏,请重新升级)")
        sys.exit(1)
    print("      校验通过")

    # ---- 6. 执行升级 ----
    print("[6/6] 执行升级(设备将重启并切换分区)...")
    client.send_frame(CMD_EXECUTE, bytes([ord(target_part)]))
    _, data = client.wait_ack(CMD_EXECUTE, timeout=args.timeout)
    # 打印设备调试输出(Boot 会报告标志区写入结果与字段,排查用)
    if client.stray:
        debug_text = bytes(client.stray).decode("ascii", errors="replace")
        print(f"  [设备打印] ...{debug_text[-300:]}")
    if len(data) < 1 or data[0] != ACK_OK:
        print("[错误] 设备拒绝执行")
        sys.exit(1)
    print("      设备已接受,正在重启...")

    # ---- 7. 重启后确认新版本 ----
    print("等待设备重启并完成 Boot 验证(约 3 秒)...")
    time.sleep(3)
    client.seq = 0
    try:
        new_version, new_part = do_handshake(client, retry_timeout=3)
        if new_part != target_part:
            print(f"[警告] 设备分区 {new_part} 与目标 {target_part} 不一致(可能回滚)")
        if new_version == (major << 8 | minor):
            print(f"[成功] 升级完成,设备运行版本 {major}.{minor}.{patch} 于分区 {new_part}")
        else:
            print(f"[警告] 设备版本 {new_version >> 8}.{new_version & 0xFF} 与目标不符")
    except SystemExit:
        # 设备调试输出:升级复位后 Boot 会打印态 2 验证日志
        # ("verify new firmware..." / "verify ok" / "verify fail, rollback"),
        # 恰好落在本段 3 秒握手等待窗口里,杂散字节已被 stray 收集
        if client.stray:
            debug_text = bytes(client.stray).decode("ascii", errors="replace")
            print(f"  [设备打印] ...{debug_text[-500:]}")
        print("[提示] 设备已重启完成(握手无应答:已进 App 正常运行时属正常,"
              "可串口发 version 命令确认)")

    client.serial_port.close()


if __name__ == "__main__":
    main()
