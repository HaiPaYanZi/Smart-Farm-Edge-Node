# -*- coding: utf-8 -*-
"""
波特率扫描:发 version 命令,看哪个波特率下能收到可读文本日志

背景:ota_diag.py 在 115200 下收到 fe fe f3 fe 乱码——板子实际波特率可能不是 115200。
这个脚本依次用常用波特率打开发送 version,能收到 "app version 1.7.x" 的就是正确波特率。

用法:
  %PY% scan_baud.py [COMx]
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM14"

# 覆盖常见波特率(含 Boot 曾出现的 12800 错配、APB 分频常见的 76800/153600)
BAUDS = [9600, 12800, 19200, 38400, 57600, 74880, 76800, 115200, 153600, 230400]


def readable_ratio(data):
    """可读 ASCII 占比(0~100),判断是不是文本日志"""
    if not data:
        return 0
    good = sum(1 for byte_value in data if 32 <= byte_value < 127)
    return good * 100 // len(data)


print(f"== 波特率扫描 {PORT}:发 'version' 看哪个能收到可读文本 ==")
for baud in BAUDS:
    try:
        s = serial.Serial(PORT, baud, timeout=0.3)
        s.reset_input_buffer()
        s.write(b"version\r\n")
        s.flush()
        time.sleep(1.2)          # 等日志回显(1.2s 足够)
        data = s.read(4096)
        ratio = readable_ratio(data)
        marker = " ◀◀ 可读!" if ratio > 70 else ""
        print(f"  {baud:7d}: {len(data):4d} 字节  可读率 {ratio:3d}%  {data[:48]!r}{marker}")
        s.close()
    except Exception as exc:
        print(f"  {baud:7d}: 打开失败 {exc}")

print("== 扫描结束:可读率 >70% 的就是实际波特率 ==")
