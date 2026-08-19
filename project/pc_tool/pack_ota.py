# -*- coding: utf-8 -*-
"""
OTA 固件打包工具(文档《05》§3.2 固件头)

把裸固件 bin 打包成升级包:16 字节固件头 + 固件数据
  固件头结构(1 字节对齐,与 shared/ota_common.h 的 ota_fw_header_t 一致):
    magic   4B  LE  0x4D4F4749 "GWNM"
    major   1B      主版本
    minor   1B      次版本
    patch   1B      补丁版本
    flags   1B      保留(0)
    length  4B  LE  固件长度(不含头)
    crc32   4B  LE  固件数据 CRC32(zlib 标准,与 Boot/设备算法一致)

用法:
  python pack_ota.py <app.bin> [选项]
  选项:
    -o OUT         输出升级包路径(默认 fw_package.bin)
    --major N      主版本(默认从 version.h 自动读取)
    --minor N      次版本(默认从 version.h 自动读取)
    --patch N      补丁版本(默认从 version.h 自动读取)
    --version-file PATH   version.h 路径(默认 ../MDK-ARM/App/version.h)

示例:
  python pack_ota.py ..\\MDK-ARM\\project\\project.bin -o fw_1.7.0.bin
"""
import argparse
import re
import struct
import sys
import zlib

OTA_FW_MAGIC = 0x4D4F4749   # "GWNM" 小端存储后的值
MAX_FW_SIZE = 100 * 1024 - 0x200   # 100KB 分区 - 512B 固件头段


def read_version_from_header(path):
    """从 version.h 提取 APP_VERSION_MAJOR/MINOR/PATCH 三个宏的值"""
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as file_handle:
            text = file_handle.read()
    except OSError:
        print(f"[错误] 打不开版本头文件: {path}")
        sys.exit(1)
    pattern = r"#define\s+APP_VERSION_(\w+)\s+(\d+)"
    values = {key: int(val) for key, val in re.findall(pattern, text)}
    for key in ("MAJOR", "MINOR", "PATCH"):
        if key not in values:
            print(f"[错误] {path} 里找不到 APP_VERSION_{key} 宏定义")
            sys.exit(1)
    return values["MAJOR"], values["MINOR"], values["PATCH"]


def main():
    parser = argparse.ArgumentParser(description="OTA 固件打包: bin + 16B 头 = 升级包")
    parser.add_argument("app_bin", help="裸固件 bin 文件路径(如 ..\\MDK-ARM\\project\\project.bin)")
    parser.add_argument("-o", "--output", default="fw_package.bin", help="输出升级包路径")
    parser.add_argument("--major", type=int, help="主版本(默认自动读 version.h)")
    parser.add_argument("--minor", type=int, help="次版本(默认自动读 version.h)")
    parser.add_argument("--patch", type=int, help="补丁版本(默认自动读 version.h)")
    parser.add_argument("--version-file", default=r"..\MDK-ARM\App\version.h",
                        help="version.h 路径")
    args = parser.parse_args()

    # 读裸固件
    try:
        with open(args.app_bin, "rb") as file_handle:
            firmware = file_handle.read()
    except OSError:
        print(f"[错误] 打不开固件文件: {args.app_bin}")
        sys.exit(1)

    if len(firmware) == 0:
        print("[错误] 固件文件为空")
        sys.exit(1)
    if len(firmware) > MAX_FW_SIZE:
        print(f"[错误] 固件 {len(firmware)} 字节,超过分区上限 {MAX_FW_SIZE} 字节")
        sys.exit(1)

    # 版本号:命令行优先,缺省读 version.h
    if args.major is not None and args.minor is not None and args.patch is not None:
        major, minor, patch = args.major, args.minor, args.patch
    else:
        major, minor, patch = read_version_from_header(args.version_file)

    # 打包:16B 头 + 固件
    crc32_value = zlib.crc32(firmware) & 0xFFFFFFFF
    header = struct.pack("<IBBBBII", OTA_FW_MAGIC, major, minor, patch, 0,
                         len(firmware), crc32_value)
    if len(header) != 16:
        print("[错误] 内部错误:固件头不是 16 字节")
        sys.exit(1)

    with open(args.output, "wb") as file_handle:
        file_handle.write(header + firmware)

    print(f"[完成] 升级包已生成: {args.output}")
    print(f"       固件版本 {major}.{minor}.{patch}, 长度 {len(firmware)} 字节, "
          f"CRC32=0x{crc32_value:08X}")


if __name__ == "__main__":
    main()
