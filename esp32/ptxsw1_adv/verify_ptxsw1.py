"""PTX 无线开关（蓝牙版）0x38BB 固件 PC 侧可测验证。

两大部分：
  1) 静态检查固件源码关键配置（PID/MAC/NVS/三事件对象号/栈/GATT 修复）
  2) 动态检查串口日志：三种按键事件帧是否按 0x4E0C/0x4E0D/0x4E0E 发出并回到电量帧

用法：
  python verify_ptxsw1.py                 # 只跑静态检查
  python verify_ptxsw1.py <串口日志路径>   # 静态 + 动态检查
"""
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(ROOT, "main", "app_main.c")
GATT = os.path.join(ROOT, "main", "gatt_miot.c")
SDKCFG = os.path.join(ROOT, "sdkconfig.defaults")

STATIC_CHECKS = [
    ("PID 0x38BB (btsw1)",        SRC, "#define PTX_PID        0x38BB"),
    ("设备名 PTX-BTSW1",          SRC, "#define PTX_NAME       \"PTX-BTSW1\""),
    ("独立 NVS 命名空间",         SRC, '#define NVS_NS         "ptxbsw1"'),
    ("独立 MAC 后缀 0x8B",        SRC, "#define BASE_MAC_B4    0x8B"),
    ("单击对象 MB_OBJ_BTN_CLICK", SRC, "s_obj_click  = MB_OBJ_BTN_CLICK"),
    ("双击对象 MB_OBJ_BTN_DOUBLE", SRC, "s_obj_double = MB_OBJ_BTN_DOUBLE"),
    ("长按对象 MB_OBJ_BTN_HOLD",  SRC, "s_obj_hold   = MB_OBJ_BTN_HOLD"),
    ("低电量事件对象 0x4C01",     SRC, "s_obj_lowbat = 0x4C01"),
    ("电量对象 MB_OBJ_BATTERY",   SRC, "s_obj_batt   = MB_OBJ_BATTERY"),
    ("双击 CLI 命令",             SRC, '!strncmp(line, "double", 6)'),
    ("低电量 CLI 命令",           SRC, '!strncmp(line, "lowbatt", 7)'),
    ("BOOT 双击状态机",           SRC, "fire_event(s_obj_double, \"DOUBLE\")"),
    ("低电量阈值 15%",            SRC, "#define LOW_BATT_THRESHOLD 15"),
    ("登录后 spec 帧切换来路面",   GATT, "miot_auth_is_logged_in(s_auth)"),
    ("12 特征 GATT 真机对齐",      GATT, "s_c01_uuid"),
    ("NimBLE host 栈 8192",       SDKCFG, "CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE=8192"),
    ("main 栈 8192",              SDKCFG, "CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192"),
]

DYNAMIC_MARKERS = [
    ("单击帧", "对象=0x4E0C (CLICK)"),
    ("双击帧", "对象=0x4E0D (DOUBLE)"),
    ("长按帧", "对象=0x4E0E (HOLD)"),
    ("低电量帧", "对象=0x4C01 (LOWBATT)"),
    ("事件回电量心跳", "回到电量帧"),
]


def check_static():
    ok = True
    print("== 静态检查 ==")
    for label, path, needle in STATIC_CHECKS:
        found = os.path.exists(path) and needle in open(path, encoding="utf-8", errors="replace").read()
        print(f"  [{'OK' if found else 'FAIL'}] {label}")
        ok = ok and found
    return ok


def check_dynamic(logpath):
    ok = True
    print("== 动态检查（串口日志）==")
    data = open(logpath, "rb").read().decode("utf-8", "replace") if os.path.exists(logpath) else ""
    for label, needle in DYNAMIC_MARKERS:
        found = needle in data
        print(f"  [{'OK' if found else 'FAIL'}] {label}")
        ok = ok and found
    return ok


if __name__ == "__main__":
    result = check_static()
    if len(sys.argv) > 1:
        result = check_dynamic(sys.argv[1]) and result
    print("\n结果:", "PASS" if result else "FAIL")
    sys.exit(0 if result else 1)
