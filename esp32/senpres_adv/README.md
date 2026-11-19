# 领普压力有无传感器（linp.senpres.ps1bb / 0x3F4C）小白版固件 —— ESP32-C3

**一根 Type-C 线、几条串口命令就能用。**

烧完这块板子在你的米家里就变成一台「领普压力有无传感器」：米家 App 能添加它、
设备页显示有/无压力，还能拿它当自动化触发条件（有压力 → 联动）。

- 目标芯片：ESP32-C3（USB-Serial/JTAG 直连）
- 模拟型号：领普压力有无传感器 `linp.senpres.ps1bb`，PID **0x3F4C**
- 上报内容：有无压力 `0x483C`（bool）/ 电量 `0x4C03`（uint8 %）
- 通道：MiBeacon V5 加密**广播**（网关收）+ 0xFE95 GATT（只用于绑定）

> 本工程与 `smoke_adv` / `gas_adv` 骨架相同，只有 PID / 对象号 / 载荷语义不同。
> 原理见 [`../t3pro_adv/README.md`](../t3pro_adv/README.md)。

---

## 一、动手前先看这几行

| 你要做的 | 怎么做 |
|---|---|
| 有压力 / 无压力 | 串口 `press 1` / `press 0`（或 `toggle`，短按 BOOT 同效） |
| 电量 % | 串口 `batt 77` |
| 恢复出厂（重新添加） | 按住 BOOT 5 秒（或串口 `factory`） |
| 看它有没有在干活 | 串口日志 `>>> 串口设有无压力 = 1` + 事件帧补发 3 次 |

---

## 二、编译

```bash
. E:\ESP\esp_env.ps1
cd E:\ESP\c3supermini\miot_sdk\esp32\senpres_adv
python E:\ESP\.espressif\v5.5.4\esp-idf\tools\idf.py build
```

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

1. 米家 App 右上角 **+** → **添加设备**（菜单第一项，不是扫一扫）
2. 「发现的设备」里出现 **领普压力有无传感器** → 点它
3. 选房间 → 设名称 → 完成（串口应打印 `绑定身份已保存` + `绑定成功`）

设备页应出现「无压力 / 有压力」+「蓝牙网关已连接」——云端按 `linp.senpres.ps1bb`
的 spec 正确识别的铁证。

---

## 五、对象号（标准新式 spec，eid 公式）

```
eid = 0x4000 | (siid << 10) | (piid - 1000)
```

| 属性 | siid.piid | spec 类型 | 广播对象 | 云端 prop |
|---|---|---|---|---|
| 有无压力 Pressure Present State | 2.1060 | bool | **0x483C**（1B 0/1） | `2:1060` |
| 电量 Battery Level | 3.1003 | uint8 0~100 | **0x4C03**（1B 百分比） | `3:1003` |

推导：`2.1060 → 0x4000\|(2<<10)\|60 = 0x483C`；`3.1003 → 0x4000\|(3<<10)\|3 = 0x4C03`。
电量是**百分比**不是枚举（与门3 的 `0x4C6B` 枚举不同，别照抄）。

---

## 六、上云双向验证（实测 2026-11-19）

did `blt.3.1qtto9isg4402`，添加 3/3 完成，设备页渲染正常。

| 串口命令 | 广播对象 | 云端读值 |
|---|---|---|
| `press 1` | `0x483C` = 1 | `2:1060 = true`（code=0） |
| `press 0` | `0x483C` = 0 | `2:1060 = false`（code=0） |
| `batt 77` | `0x4C03` = 77 | `3:1003 = 77`（code=0） |
| `batt 33` | `0x4C03` = 33 | `3:1003 = 33`（code=0） |

验证时序：**先关米家设备页**（占 BLE 连接 → 网关收不到广播），串口发命令后等 5~10s
再读云端。事件帧连发 3 次（400ms 间隔）防丢。

---

## 七、串口命令

| 命令 | 作用 |
|---|---|
| `toggle` | 有/无压力切换，发 `0x483C` 事件帧 |
| `press 0\|1` | 直接设有无压力 |
| `batt N` | 设电量 0-100，发 `0x4C03` 事件帧 |
| `info` | 打印 PID / MAC / bound / press / batt |
| `objp XXXX` | 临时改 pressure 对象号（排查用） |
| `objb XXXX` | 临时改 battery 对象号（排查用） |
| `factory` | 擦身份重启 |

---

## 八、改参数（都在 `main/app_main.c` 顶部）

- `SENPRES_PID` / `SENPRES_NAME` —— 型号
- `NVS_NS` —— 必须与其它工程不同
- `BASE_MAC_B4/B5` —— 派生 MAC（本工程 `0x7F:0x03` → 空中 `…:7F:05`）
- `s_obj_press` / `s_obj_batt` —— 对象号
