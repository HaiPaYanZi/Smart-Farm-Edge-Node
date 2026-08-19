# -*- coding: utf-8 -*-
"""
OTA 握手诊断 v4:打开端口后拉低 DTR/RTS,等板子启动完成,再握手

背景(已查明的机制):pyserial/助手打开 COM14 的瞬间,DTR/RTS 电平跳变
会触发 STM32 复位(Boot 重启日志为证)。ota_tool.py 打开后立即发握手帧,
帧落在 Boot 重启窗口 → 丢失 → 无应答。
本脚本:打开 → 拉低 DTR/RTS(释放复位) → 等 3s → 连发 5 次握手帧。

前提:升级标志已置位(先在助手发 ota 进升级模式,或 upgrade_req=1 遗留)
用法:
  %PY% ota_diag.py [COMx]
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM14"

# 手动验证成功的握手帧:AA 55 | 01 | seq=0 | len=2 | 版本 0x0000 | CRC16 | 0D
HANDSHAKE = bytes.fromhex("AA 55 01 00 00 02 00 00 00 0A 78 0D")

print(f"== 打开 {PORT} @ 115200 ==")
s = serial.Serial(PORT, 115200, timeout=0.2)
print(f"   打开时: DTR={int(s.dtr)} RTS={int(s.rts)}")

print("== 拉低 DTR/RTS(释放复位,避免打开瞬间的跳变持续影响板子) ==")
s.setDTR(False)
s.setRTS(False)
time.sleep(0.1)
print(f"   拉低后: DTR={int(s.dtr)} RTS={int(s.rts)}")
s.reset_input_buffer()

print("== 等 3s 让板子完成启动(upgrade_req=1 时 Boot 会自动进升级模式) ==")
time.sleep(3)

print("== 连发 5 次握手帧 ==")
for attempt in range(1, 6):
    print(f"   第 {attempt} 次发送: {HANDSHAKE.hex(' ')}")
    s.write(HANDSHAKE)
    s.flush()
    data = b""
    deadline = time.time() + 3
    while time.time() < deadline:
        data += s.read(4096)
    if data:
        print(f"   收到 {len(data)} 字节: {data.hex(' ')}")
    else:
        print("   3s 无应答")
    if b"\xAA\x55\x81" in data:
        print("   → 握手成功!Boot 在线!")
        break
    time.sleep(0.5)

s.close()
print("== 诊断结束 ==")
