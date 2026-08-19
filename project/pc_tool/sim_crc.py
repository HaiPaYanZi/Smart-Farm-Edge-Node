# -*- coding: utf-8 -*-
"""
复现 Boot 的 CRC32 计算(查表法 0xEDB88320 / 初值 0xFFFFFFFF / 结果异或),与 zlib 对比

背景:OTA 升级 [5/6] 全片 CRC32 校验不通过——Boot 算的 local CRC 与工具发的
host CRC(0x339CB305)不一致。本脚本验证"Boot 的算法实现"本身是否正确:
  1. 按 Boot 的 boot_crc_init/update/final 逐字节复现,算固件 CRC
  2. 与 zlib.crc32 对比
  3. 顺带算"数据区末尾 408 字节是 0xFF"等假设场景,供和 Boot 打印的 local 值对比

用法:
  %PY% sim_crc.py [fw_package.bin]
"""
import sys
import zlib

FW_FILE = sys.argv[1] if len(sys.argv) > 1 else "fw_package.bin"

with open(FW_FILE, "rb") as fh:
    package = fh.read()

# 固件 = 升级包去掉 16B 头
firmware = package[16:]
print(f"固件长度: {len(firmware)} 字节 (0x{len(firmware):X})")

# ---- 1. Boot 查表法复现 ----
table = []
for i in range(256):
    c = i
    for _ in range(8):
        c = (c >> 1) ^ 0xEDB88320 if c & 1 else c >> 1
    table.append(c)


def boot_crc32(data):
    crc = 0xFFFFFFFF
    for b in data:
        crc = (crc >> 8) ^ table[(crc ^ b) & 0xFF]
    return crc ^ 0xFFFFFFFF


print(f"zlib 结果:   0x{zlib.crc32(firmware) & 0xFFFFFFFF:08X}  (打包时用的值)")
print(f"Boot 复现:   0x{boot_crc32(firmware):08X}")

# ---- 2. 假设场景:数据区末尾 408 字节(页 24 段)是 0xFF(未写入) ----
# 数据终点 0x0802C198,页 24 起点 0x0802C000 → 固件内偏移 48640 起
# 若该段未写入,Flash 里是 0xFF
scenario = bytearray(firmware)
for i in range(48640, len(firmware)):
    scenario[i] = 0xFF
print(f"假设'末 408B 是 0xFF': 0x{boot_crc32(bytes(scenario)):08X}")

# ---- 3. 假设场景:整页 0x0802C000(最后 0x200 起)是 0xFF ----
scenario2 = bytearray(firmware)
for i in range(48896, len(firmware)):
    scenario2[i] = 0xFF
print(f"假设'末 152B 是 0xFF': 0x{boot_crc32(bytes(scenario2)):08X}")

print()
print("对比方式:把上面结果和 Boot 打印的 'verify fail local=' 对比,"
      "命中哪个就是哪种损坏")
