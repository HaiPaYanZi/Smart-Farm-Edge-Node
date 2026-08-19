# -*- coding: utf-8 -*-
"""
定位 OTA [5/6] CRC32 失败:枚举"Flash 里哪段数据错了"

Boot 实测 local = 0x3704353B(本次运行改 TARGET 即可)。在 PC 端把固件按各种
"损坏假设"改掉再算 CRC32,命中 0x3704353B 的假设就是 Flash 里的真实情况。

假设枚举:
  A. 第 i 页(2KB)整页没写进去 → 内容 0xFF
  B. 第 i 页写成了前一页的内容(重复)
  C. 从偏移 N 起之后全部 0xFF(某处起中断写入,尾部优先扫)

用法:
  %PY% verify_sim.py [fw_package.bin]
"""
import sys
import zlib

TARGET = 0x3704353B    # Boot 打印的 verify fail local= 值(本次实测)

with open(sys.argv[1] if len(sys.argv) > 1 else "fw_package.bin", "rb") as fh:
    package = fh.read()
firmware = package[16:]      # 去掉 16B 固件头
L = len(firmware)
PAGE = 2048
pages = (L + PAGE - 1) // PAGE

print(f"固件 {L} 字节, 共 {pages} 页, 目标 local = 0x{TARGET:08X}")
print(f"正确固件 CRC = 0x{zlib.crc32(firmware) & 0xFFFFFFFF:08X}")

hit = False

# ---- 假设 A: 第 i 页整页是 0xFF(没写进去) ----
print("\n[A] 逐页 0xFF(整页没写入)...")
for i in range(pages):
    blob = bytearray(firmware)
    start, end = i * PAGE, min(L, (i + 1) * PAGE)
    blob[start:end] = b"\xFF" * (end - start)
    if (zlib.crc32(bytes(blob)) & 0xFFFFFFFF) == TARGET:
        print(f"  ★命中! 页 {i}: 偏移 0x{start:X}~0x{end:X} 是 0xFF")
        hit = True
if not hit:
    print("  无命中")

# ---- 假设 B: 第 i 页写成了前一页的内容(重复/错位一页) ----
print("[B] 逐页重复前一页内容...")
for i in range(1, pages):
    blob = bytearray(firmware)
    start, end = i * PAGE, min(L, (i + 1) * PAGE)
    span = end - start
    blob[start:end] = firmware[(i - 1) * PAGE:(i - 1) * PAGE + span]
    if (zlib.crc32(bytes(blob)) & 0xFFFFFFFF) == TARGET:
        print(f"  ★命中! 页 {i} 内容=页 {i-1} 内容 (偏移 0x{start:X} 起)")
        hit = True
if not hit:
    print("  无命中")

# ---- 假设 C: 从偏移 N 起全部 0xFF(某处起中断写入) ----
# 用"前缀中间值 + 尾填 0xFF"逐点算;尾部优先(错误在尾部概率最高)
print("\n[C] 从偏移 N 起全 0xFF(尾部 8KB 逐点 + 页边界 + 全范围粗扫)...")
table = []
for i in range(256):
    c = i
    for _ in range(8):
        c = (c >> 1) ^ 0xEDB88320 if c & 1 else c >> 1
    table.append(c)


def crc_from_mid(mid, length):
    """从中间状态起再灌 length 个 0xFF,返回最终 CRC32"""
    for _ in range(length):
        mid = (mid >> 8) ^ table[(mid ^ 0xFF) & 0xFF]
    return mid ^ 0xFFFFFFFF


def f_n(n):
    """前 n 字节正确 + 之后全 0xFF 的 CRC32"""
    mid = 0xFFFFFFFF
    for b in firmware[:n]:
        mid = (mid >> 8) ^ table[(mid ^ b) & 0xFF]
    return crc_from_mid(mid, L - n)


# 候选点:尾部 8KB 逐字节(倒序,命中即停) + 页边界 + 64 字节粗扫
cands = list(range(L - 1, L - 8192 - 1, -1))
cands += [i * PAGE for i in range(pages)]
cands += list(range(0, L, 64))
seen = set()
for n in cands:
    if n in seen:
        continue
    seen.add(n)
    if f_n(n) == TARGET:
        print(f"  ★命中! 偏移 0x{n:X} 起之后全是 0xFF"
              f" (对应页 {(n // PAGE)}, 页内偏移 0x{n % PAGE:X})")
        hit = True
        break
if not hit:
    print("  无命中(假设 C 不成立)")

# ---- 假设 D: 第 i 页内容 = 固件第 j 页内容(单页写错来源) ----
print("\n[D] 逐页替换为任意固件页(错位来源枚举)...")
for i in range(pages):
    start, end = i * PAGE, min(L, (i + 1) * PAGE)
    span = end - start
    for j in range(pages):
        if j == i:
            continue
        blob = bytearray(firmware)
        blob[start:end] = firmware[j * PAGE:j * PAGE + span]
        if (zlib.crc32(bytes(blob)) & 0xFFFFFFFF) == TARGET:
            print(f"  ★命中! Flash 页 {i}(0x{start:X} 起) 内容=固件页 {j}(0x{j * PAGE:X} 起)")
            hit = True
            break
    if hit:
        break
if not hit:
    print("  无命中")

# ---- 假设 E: 整体头部错位(0xFF×k + 固件[:L-k],写地址整体偏前) ----
print("\n[E] 整体头部错位(页粒度 + 256 粒度 + 细扫)...")
for k in sorted({p * PAGE for p in range(pages)} | {n * 256 for n in range(L // 256)}):
    if k <= 0 or k >= L:
        continue
    blob = bytearray(b"\xFF" * k) + firmware[:L - k]
    if (zlib.crc32(bytes(blob)) & 0xFFFFFFFF) == TARGET:
        print(f"  ★命中! Flash 内容 = 0xFF×{k} + 固件[:{L - k}] (整体错位 0x{k:X})")
        hit = True
        break
if not hit:
    print("  无命中")

if not hit:
    print("\n五种假设都没命中 → 错误形状更复杂,需抓 Flash 实际内容逐段对比")
