# 米家夜灯2 蓝牙版（yeelink.light.nl1 / 0x07F6）小白版固件 —— ESP32-C3

**一根 Type-C 线、几条串口命令就能用。**

烧完这块板子在你的米家里就变成一台「米家夜灯2 蓝牙版（夜灯）」：米家 App 能添加它、
设备页渲染（夜灯设置/智能场景/传感器日志），还能拿它当自动化触发条件（无人移动时长 → 联动）。

- 目标芯片：ESP32-C3（USB-Serial/JTAG 直连）
- 模拟型号：米家夜灯2 蓝牙版 `yeelink.light.nl1`，PID **0x07F6**
- 上报内容：`2.1` no-motion-duration（uint16，广播对象 `0x1017`，秒）→ 网关按 spec 枚举
  （2/5/10/20/30 分钟）归档 → 云端 `2:1`
- 通道：MiBeacon V5 加密**广播**（网关收）+ 0xFE95 GATT（绑定 + 属性）
- display_type=232（标准 spec 设备；与采用私有 careco-spec 的补光灯不同）

> 本工程与 `ddla_adv` / `nlight_adv` 骨架相同，只有 PID / 对象号 / 属性表不同。
> 原理见 [`../t3pro_adv/README.md`](../t3pro_adv/README.md)。

---

## 一、动手前先看这几行

| 你要做的 | 怎么做 |
|---|---|
| 无人移动时长（上云） | 串口 `motion 600`（秒），发 `0x1017` 事件帧；网关按分钟档归档 |
| 电量（GATT 属性） | 串口 `batt 42` |
| 光照（GATT 属性） | 串口 `lux 120` |
| 恢复出厂（重新添加） | 按住 BOOT 5 秒（或串口 `factory`） |

---

## 二、编译

```bash
. E:\ESP\esp_env.ps1
cd E:\ESP\c3supermini\miot_sdk\esp32\nl1_adv
python E:\ESP\.espressif\v5.5.4\esp-idf\tools\idf.py build
```

> ★`sdkconfig.defaults` 里 `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192`：会话建立后立刻推
> 3 条属性初值，默认 3584B 栈会 `Stack protection fault`（Guru Meditation 实测）。

---

## 三、烧录（第一次一定要擦）

```bash
idf.py -p COM36 erase-flash flash     # 第一次：擦掉旧身份，从待绑定态开始
```

重刷（设备已绑定、只是换了固件）：

```bash
idf.py -p COM36 flash                 # 不要 erase —— NVS 里的绑定身份会保留
```

---

## 四、在米家里添加

1. 米家 App 右上角 **+** → 添加设备
2. 「发现的设备」里出现 **米家夜灯2 蓝牙版** → 点它
3. 选房间 → 设名称 → 「更多设置」→ 用户协议 → 完成

设备页应出现「夜灯设置 / 智能场景 / 传感器日志」——这是云端按 `yeelink.light.nl1`
的 spec 正确识别的铁证。

---

## 五、对象号与属性表

| 属性 | siid.piid | spec 类型 | 通道 | 云端 prop |
|---|---|---|---|---|
| 无人移动时长 | 2.1 | uint16（枚举分钟） | 广播 `0x1017`（uint32 秒） | `2:1` |
| 电量 | 3.1 | uint8 0~100 | GATT 属性（只读） | — |
| 光照 | 4.1 | float 0~10000 | GATT 属性（只读） | — |

`0x1017` 是标准 MiBeacon `no-motion` 对象（4B 秒数，RTCGQ02LM/夜灯共用），
不是新式 `eid` 公式对象。网关收到秒数后按 spec 枚举归档：
`motion 123` → `2:1=2`（2 分钟档），`motion 600` → `2:1=10`（10 分钟档）。

---

## 六、★验证时序（坑）

**验证广播属性前** `adb shell am force-stop com.xiaomi.smarthome` —— App 设备页占着
BLE 连接时网关收不到广播帧，云端读不更新。串口发 `motion N` 后等 10~20s 网关同步再读。

---

## 七、串口命令

| 命令 | 作用 |
|---|---|
| `motion N` | 无人移动时长 s + 发 `0x1017` 事件帧 |
| `batt N` | 电量 %（3.1） |
| `lux N` | 光照（4.1） |
| `toggle` | 切换 motion（10s ↔ 0s，等价短按 BOOT） |
| `info` | 打印 PID/MAC/bound/motion/batt/lux |
| `obj XXXX` | 临时改广播对象号（盲测排查用） |
| `factory` | 擦身份重启（恢复待绑定） |

---

## 八、已知边界

- 「传感器日志」页面需要账号下有蓝牙网关才出历史，纯广播设备只提供实时值。
- `3.1` 电量 / `4.1` 光照是标准 spec 的 read/notify 属性，GATT 属性表已实现并推送初值；
  本工程主验证通道是广播 `2.1`（与 gas/smoke/petwater 同级的「串口改→广播→网关→云端读」）。
- 设备身份：did=`blt.3.1qtq2jessck00`，派生 MAC `10:00:3B:CE:7B:05`，NVS 命名空间 `nl1`。

## 免责声明

仅供个人学习与智能家居 DIY 使用。请勿用于商业用途或干扰他人设备。
