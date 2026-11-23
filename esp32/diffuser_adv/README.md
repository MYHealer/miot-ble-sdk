# 米家智能香氛机-杯托版 可绑定固件 —— ESP32-C3

**可以直接编译烧录、被米家 App 绑定、设备页完整渲染并逐属性双向控制的固件。**
2026-11-23 实测：米家「添加设备」发现「米家智能香氛机-杯托版」→ 绑定 `REGISTER COMPLETE 0x11`
→ 设备页显示香型/浓度/出香模式 → 各控件 SET 下发并回推。

- 目标芯片：ESP32-C3（USB-Serial/JTAG）
- 模拟型号：米家智能香氛机-杯托版 `xiaomi.diffuser.xwczxf`，PID **0x39E6**（pdid 14822）
  - 官方产品库 `resolve_type=3` 广播可识别（米家自家）
  - spec：`urn:miot-spec-v2:device:diffuser:0000A01E:xiaomi-xwczxf:1`（11 属性）
- 通道：MiBeacon V5 加密广播（4 帧轮播 on/mode/out-time/liquid → 网关）
  + 0xFE95 GATT（手机设备页，12 特征值真机对齐）

## 属性与控制（全部实测）

| siid.piid | 名称 | App 控件 | 实测 |
|---|---|---|---|
| 2.1032 | on | 开机/关机 | 工作态 on=1 时浓度/香型可点 |
| 2.1020 | mode | 出香模式（午夜橙花/晨间松木/正午丁香）| ✅ SET 2.1020 val=1 → UI 切晨间松木 |
| 2.1069 | fragrance-out-time | 出香浓度 1-5 档 | ✅ SET 2.1069 val=150 → UI 显示 5档 |
| 2.2 | top-cover-status | — | ok |
| 3/4/5.1070 | 香液余量 | 耗材 | ok |
| 3/4/5.1 | 香液名称 | — | ok |
| 6.1032 | light on | — | ok |

★注意：官方 spec 把多数属性标 `r` 或空 `rw`，但 App 实际把它们当可写下发
（电水壶 3.3 同类先例）。全部注册为 RW 才能通过 SET 并回推。

## 编译 / 烧录

```bash
cd esp32/diffuser_adv
idf.py build
idf.py -p COM36 flash        # 不要 erase-flash（丢绑定身份）
```

## 米家添加

米家 → **+** → **添加设备** → 发现列表出现「米家智能香氛机-杯托版」→ 点卡片 →
`REGISTER COMPLETE 0x11` → 完成房间/命名。

## 串口命令

| 命令 | 作用 |
|---|---|
| `on 0\|1` | 开机/关机 |
| `mode 0-4` | 出香模式 |
| `liq N` | 香液余量 |
| `info` / `factory` | 状态 / 擦身份重绑 |

BOOT 键：短按切换开关；长按 5s 恢复出厂。

## GATT / SDK 修复

GATT 层与 sdkconfig 来自 `carlight_adv` 完整修复版：12 特征值真机对齐、
登录后 `s_rx_chan` 只在 spec 帧更新、NimBLE host/main 栈 8192。
u16 属性（2.1069 out-time）走 SDK PR#16 的按 size 精确读写。
