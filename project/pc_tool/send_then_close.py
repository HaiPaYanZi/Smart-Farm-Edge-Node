# -*- coding: utf-8 -*-
"""
发送后立即关闭:pyserial 发一条命令后不读取,关闭端口,
让串口助手查看板子是否有响应(CH340 接收缓冲里的字节会保留)

用途:分水岭实验——
  助手能看到板子响应 → pyserial 发送正常,问题在 pyserial 接收侧
  助手也看不到 → pyserial 发送没到达板子

用法:
  %PY% send_then_close.py [COMx] [命令文本]
  例: %PY% send_then_close.py COM14 version
      %PY% send_then_close.py COM14 ota
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM14"
CMD = (sys.argv[2] + "\r\n").encode() if len(sys.argv) > 2 else b"version\r\n"

s = serial.Serial(PORT, 115200, timeout=0.1)
print(f"已打开 {PORT} @ 115200, DTR={int(s.dtr)} RTS={int(s.rts)}")
s.write(CMD)
s.flush()
print(f"已发送: {CMD!r}")
time.sleep(0.8)          # 给板子响应时间(不读取,字节留在 CH340 缓冲)
s.close()                # 立即关闭
print("端口已关闭。请立刻打开串口助手(115200),查看 RX 里板子是否回了日志")
