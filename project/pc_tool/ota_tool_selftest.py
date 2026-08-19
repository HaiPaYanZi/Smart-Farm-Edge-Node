# -*- coding: utf-8 -*-
"""
OTA 工具自测:内存 loopback 模拟设备(复刻 Bootloader 协议行为)

在没有实物的条件下端到端验证 ota_tool.py 的协议流程:
  模拟设备实现 Boot 侧状态机(帧解析/命令应答/丢包与重传逻辑),
  通过伪串口与 OtaClient 对接,跑完整升级流程。

用法:
  python ota_tool_selftest.py
"""
import struct
import sys
import zlib

# 复用 ota_tool 的协议常量与 CRC 实现
import ota_tool

PACKET_SIZE = ota_tool.PACKET_SIZE


class MockDevice:
    """模拟 Bootloader:字节流帧解析 + 命令应答(行为与 Boot 状态机一致)"""

    def __init__(self):
        self.rx_buffer = bytearray()     # 从 PC 收到的字节
        self.tx_buffer = bytearray()     # 待发给 PC 的字节
        self.state = "idle"              # idle/rx/verified/rebooted
        self.firmware = bytearray()      # 已收固件
        self.expected_packet = 0         # 期望包号
        self.packet_count = 0            # 已收包数
        self.total_packets = 0
        self.header = None
        self.active_part = "A"           # 模拟设备当前活动分区

    # ---- 伪串口接口:PC 侧 write 进这里,read 从 tx 取 ----
    def write(self, data):
        self.rx_buffer.extend(data)
        self.process_all()

    def read(self, size):
        if not self.tx_buffer:
            return b""
        out = bytes(self.tx_buffer[:size])
        del self.tx_buffer[:size]
        return out

    # ---- Boot 侧帧解析(与 boot_proto.c 同构) ----
    def process_all(self):
        while True:
            frame = self.try_extract_frame()
            if frame is None:
                return
            cmd, seq, data = frame
            self.handle_command(cmd, seq, data)

    def try_extract_frame(self):
        """找一帧:AA 55 ... CRC16 ... 0D;不完整返回 None;坏帧丢弃后继续"""
        while True:
            if len(self.rx_buffer) < 2:
                return None
            if self.rx_buffer[0] != ota_tool.FRAME_HEAD1:
                del self.rx_buffer[0]
                continue
            if self.rx_buffer[1] != ota_tool.FRAME_HEAD2:
                del self.rx_buffer[0]
                continue
            break
        if len(self.rx_buffer) < 8:
            return None
        data_len = self.rx_buffer[5] | (self.rx_buffer[6] << 8)
        if data_len > ota_tool.FRAME_DATA_MAX:
            del self.rx_buffer[:2]      # 长度非法:丢帧头重找
            return None
        frame_len = 8 + data_len + 2
        if len(self.rx_buffer) < frame_len:
            return None
        frame = bytes(self.rx_buffer[:frame_len])
        del self.rx_buffer[:frame_len]
        body = frame[2:7 + data_len]
        crc_received = frame[7 + data_len] | (frame[8 + data_len] << 8)
        if ota_tool.crc16_modbus(body) != crc_received or frame[-1] != ota_tool.FRAME_TAIL:
            return None                 # 坏帧丢弃,继续找
        cmd = frame[2]
        seq = frame[3] | (frame[4] << 8)
        data = frame[7:7 + data_len]
        return cmd, seq, data

    # ---- 命令处理(与 boot_proto.c 的 handle_* 对齐) ----
    def send_ack(self, cmd, seq, data=b""):
        body = bytes([0x80 | cmd]) + struct.pack("<H", seq) \
             + struct.pack("<H", len(data)) + data
        crc_value = ota_tool.crc16_modbus(body)
        self.tx_buffer.extend(bytes([ota_tool.FRAME_HEAD1, ota_tool.FRAME_HEAD2]) + body
                              + struct.pack("<H", crc_value)
                              + bytes([ota_tool.FRAME_TAIL]))

    def handle_command(self, cmd, seq, data):
        if self.state == "rebooted":
            return                      # App 运行中:不答协议(真实设备行为)
        if cmd == ota_tool.CMD_HANDSHAKE:
            self.send_ack(cmd, seq, bytes([0, 1, 7, ord(self.active_part)]))
        elif cmd == ota_tool.CMD_START:
            if len(data) != 16:
                self.send_ack(cmd, seq, bytes([ota_tool.ACK_PART_INVALID]))
                return
            self.header = data
            self.firmware = bytearray()
            self.expected_packet = 0
            self.packet_count = 0
            length = struct.unpack("<I", data[8:12])[0]
            self.total_packets = (length + PACKET_SIZE - 1) // PACKET_SIZE
            self.state = "rx"
            self.send_ack(cmd, seq, bytes([ota_tool.ACK_OK]))
        elif cmd == ota_tool.CMD_DATA:
            if self.state != "rx" or len(data) < 2:
                return
            packet_number = data[0] | (data[1] << 8)
            payload = data[2:]
            if packet_number == self.expected_packet:
                self.firmware.extend(payload)
                self.expected_packet += 1
                self.packet_count += 1
                self.send_ack(cmd, seq, struct.pack("<HB", packet_number, ota_tool.ACK_OK))
            elif packet_number < self.expected_packet:
                self.send_ack(cmd, seq, struct.pack("<HB", packet_number, ota_tool.ACK_OK))
            else:
                # 丢包:答期望包号 + 失败码(工具应从期望包续传)
                self.send_ack(cmd, seq,
                              struct.pack("<HB", self.expected_packet, ota_tool.ACK_CRC_FAIL))
        elif cmd == ota_tool.CMD_FINISH:
            if self.state != "rx" or len(data) < 2:
                return
            self.send_ack(cmd, seq, struct.pack("<H", self.packet_count))
        elif cmd == ota_tool.CMD_VERIFY:
            if self.state != "rx" or len(data) < 4:
                return
            host_crc = struct.unpack("<I", data[:4])[0]
            local_crc = zlib.crc32(bytes(self.firmware)) & 0xFFFFFFFF
            if host_crc == local_crc:
                self.state = "verified"
                self.send_ack(cmd, seq, bytes([ota_tool.ACK_OK]))
            else:
                self.send_ack(cmd, seq, bytes([ota_tool.ACK_CRC_FAIL]))
        elif cmd == ota_tool.CMD_EXECUTE:
            if self.state != "verified" or len(data) < 1:
                return
            target = chr(data[0])
            expect = "B" if self.active_part == "A" else "A"
            if target != expect:
                self.send_ack(cmd, seq, bytes([ota_tool.ACK_PART_INVALID]))
                return
            self.send_ack(cmd, seq, bytes([ota_tool.ACK_OK]))
            # 模拟 Boot 复验通过 → 切区 → 跳 App(App 不答协议)
            self.active_part = target
            self.state = "rebooted"


def main():
    """跑完整升级流程(把 OtaClient 的串口换成 mock)"""
    device = MockDevice()
    client = ota_tool.OtaClient.__new__(ota_tool.OtaClient)   # 绕过串口打开
    client.serial_port = device
    client.seq = 0

    # 用刚打包的测试文件
    package_path = "fw_test.bin"
    try:
        with open(package_path, "rb") as file_handle:
            package = file_handle.read()
    except OSError:
        print(f"[跳过] 找不到测试包 {package_path},请先运行 pack_ota.py")
        return

    magic, major, minor, patch, _flags, length, crc32_value \
        = struct.unpack("<IBBBBII", package[:16])
    firmware = package[16:16 + length]
    total_packets = (length + PACKET_SIZE - 1) // PACKET_SIZE
    print(f"测试包: 版本 {major}.{minor}.{patch}, {length} 字节, {total_packets} 包")

    failures = []

    def check(name, condition):
        print(f"  [{'PASS' if condition else 'FAIL'}] {name}")
        if not condition:
            failures.append(name)

    # ---- 握手 ----
    device_version, active_part = ota_tool.do_handshake(client, 2)
    check("握手得到版本与分区", (device_version, active_part) == ((1 << 8) | 7, "A"))

    # ---- 固件头 ----
    client.send_frame(ota_tool.CMD_START, package[:16])
    _, data = client.wait_ack(ota_tool.CMD_START, timeout=2)
    check("固件头接受", data == bytes([ota_tool.ACK_OK]))

    # ---- 数据包(正常传输) ----
    ota_tool.send_data_packets(client, firmware, total_packets)
    check("数据包全部确认", device.packet_count == total_packets)

    # ---- 结束 ----
    client.send_frame(ota_tool.CMD_FINISH, struct.pack("<H", total_packets))
    _, data = client.wait_ack(ota_tool.CMD_FINISH, timeout=2)
    check("结束核对包数一致", (data[0] | (data[1] << 8)) == total_packets)

    # ---- CRC32 ----
    client.send_frame(ota_tool.CMD_VERIFY, struct.pack("<I", crc32_value))
    _, data = client.wait_ack(ota_tool.CMD_VERIFY, timeout=2)
    check("CRC32 校验通过", data == bytes([ota_tool.ACK_OK]))

    # ---- 执行 ----
    client.send_frame(ota_tool.CMD_EXECUTE, bytes([ord("B")]))
    _, data = client.wait_ack(ota_tool.CMD_EXECUTE, timeout=2)
    check("执行接受", data == bytes([ota_tool.ACK_OK]))
    check("设备切到 B 区", device.active_part == "B")

    # ---- 丢包续传专项:重置设备,故意错序发包 ----
    device2 = MockDevice()
    client.serial_port = device2
    client.seq = 0
    ota_tool.do_handshake(client, 2)
    client.send_frame(ota_tool.CMD_START, package[:16])
    client.wait_ack(ota_tool.CMD_START, timeout=2)

    # 先发 5 号包(超前):设备应拒绝并告知期望 0 号
    chunk = firmware[5 * PACKET_SIZE:6 * PACKET_SIZE]
    client.send_frame(ota_tool.CMD_DATA, struct.pack("<H", 5) + chunk)
    _, data = client.wait_ack(ota_tool.CMD_DATA, timeout=2)
    reject_packet = data[0] | (data[1] << 8)
    check("超前包被拒且告知期望包号", data[2] != ota_tool.ACK_OK and reject_packet == 0)

    # 工具自身逻辑继续从 0 发(丢包续传入口)
    ota_tool.send_data_packets(client, firmware, total_packets)
    check("丢包后自动续传收齐", device2.packet_count == total_packets)

    print()
    if failures:
        print(f"[结果] {len(failures)} 项失败: {failures}")
        sys.exit(1)
    print("[结果] 全部通过:OTA 协议链路自测 OK")


if __name__ == "__main__":
    main()
