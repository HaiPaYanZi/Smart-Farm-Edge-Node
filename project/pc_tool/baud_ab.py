# -*- coding: utf-8 -*-
"""
波特率 A/B 对照:同样发 version 命令,分别用 115200 和 57600 打开

背景:助手 115200 正常,但 pyserial 115200 打开收到乱码(疑似实际波特率减半)。
若 57600 打开能收到正常文本 → 实锤 CH340 在 pyserial 打开下实际 57600。

前提:板子已断电重启,正在运行 App(version 命令会回 "app version 1.7.x")
用法:
  %PY% baud_ab.py [COMx]
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM14"

for baud in (115200, 57600):
    print(f"--- 用 {baud} 打开,发 version ---")
    s = serial.Serial(PORT, baud, timeout=0.3)
    print(f"    DTR={int(s.dtr)} RTS={int(s.rts)}")
    s.reset_input_buffer()
    s.write(b"version\r\n")
    s.flush()
    data = b""
    deadline = time.time() + 2
    while time.time() < deadline:
        data += s.read(4096)
    print(f"    收到 {len(data)} 字节")
    print(f"    hex: {data.hex(' ')}")
    print(f"    txt: {data.decode('ascii', errors='replace')!r}")
    s.close()
    time.sleep(0.5)

print("== 结束:哪个波特率下 txt 是可读的 'app version...',哪个就是实际波特率 ==")
