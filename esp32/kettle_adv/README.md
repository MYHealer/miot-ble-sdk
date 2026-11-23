# 米家恒温电水壶 Pro 可绑定固件 —— ESP32-C3

**可以直接编译烧录、被米家 App 绑定、设备页完整渲染并逐属性双向控制的固件。**
2026-11-23 实测：米家「添加设备」发现「米家恒温电水壶Pro」→ 绑定 `REGISTER COMPLETE 0x11`
→ 云端下发 did → 设备页显示水温/档位/时长 → 各控件 SET 都能下发并回推。

- 目标芯片：ESP32-C3（USB-Serial/JTAG）
- 模拟型号：米家恒温电水壶 Pro `yunmi.kettle.v13`，PID **0x1809**（pdid 6153）
  - 官方产品库：`resolve_type=3` 广播可识别，`bt_match=1` 本地控制型
  - spec：`urn:miot-spec-v2:device:kettle:0000A009:yunmi-v13:1`（17 个属性）
- 通道：MiBeacon V5 加密广播（4 帧轮播状态/开关/时长/水温 → 网关上云）
  + 0xFE95 GATT（手机设备页，12 特征值真机对齐）

## 属性与控制（全部实测）

| siid.piid | 名称 | 权限 | App 控件 | 实测 |
|---|---|---|---|---|
| 2.1 | system-status | R | 待机/加热中 | 广播上云 |
| 2.2 | err-state | R | — | ok |
| 3.1 | insulation-mode | RW | 保温模式 | ok |
| 3.2 | insulation-switch | RW | 保温开关 | ok |
| 3.3 | insulation-gear | RW | 45/55/65/80/90℃ 档位按钮 | ✅ SET 3.3 val=4 → 水温 90 回推 |
| 3.5 | keep-warn-time | R | — | 广播上云 |
| 3.6 | water-temp | R | 当前水温 | ✅ 传感器 `temp 80` → UI 显示 80℃ |
| 5.1-5.5 | level-one..five | RW | 温度滑块 | ok |
| 5.6-5.10 | duration-one..five | RW | 时长滑块 | ✅ SET 5.10 val=17 → 回推 |

★坑：官方 spec 把 3.3 标为只读，但 App 点档位按钮实际下发 `SET 3.3`（当可写用）。
必须把 3.3 注册成 RW 并联动水温回推，否则点档位返 ERR、UI 不更新。

## 编译 / 烧录

```bash
cd esp32/kettle_adv
idf.py build
idf.py -p COM36 flash        # 不要 erase-flash（丢绑定身份）
```

## 米家添加

米家 → **+** → **添加设备** → 发现列表出现「米家恒温电水壶Pro」→ 点卡片 →
`REGISTER COMPLETE 0x11` → 完成房间/命名。

## 串口命令

| 命令 | 作用 |
|---|---|
| `status 0-5` | 改系统状态（待机/加热/保温等）|
| `temp N` | 模拟传感器探到 N℃ → 立刻回推 App（3.6）|
| `switch 0\|1` | 保温开关 |
| `mode 0\|1` | 保温模式 |
| `info` / `factory` | 状态 / 擦身份重绑 |

BOOT 键：短按切换待机/加热；长按 5s 恢复出厂。

## GATT / SDK 修复

GATT 层与 sdkconfig 来自 `carlight_adv` 完整修复版：12 特征值真机对齐、
登录后 `s_rx_chan` 只在 spec 帧更新（避免来路面污染）、NimBLE host/main 栈 8192。
属性表 size=2 的 u16（3.5 keep-warn-time）走 SDK PR#16 的按 size 精确读写。
