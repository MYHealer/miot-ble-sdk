# 米家甲醛检测仪（JQJCY01YM）小白版固件 —— ESP32-C3

**一根 Type-C 线、三条命令、按一下 BOOT 就能用。**

烧完这块板子在你的米家里就变成一台「甲醛检测仪」：米家 App 能添加它、
设备页能看甲醛浓度 / 温度 / 湿度 / 电量，还能拿它当自动化触发条件
（甲醛超标 → 开新风 / 开窗 / 响小爱）。

- 目标芯片：ESP32-C3（USB-Serial/JTAG 直连）
- 模拟型号：甲醛检测仪 `JQJCY01YM`，PID **0x02DF**
- 上报内容（四槽轮播，4 秒一轮）：
  `0x1010` 甲醛 0.01 mg/m³ / `0x1004` 温度 0.1 ℃ / `0x1006` 湿度 0.1 % / `0x100A` 电量 %
- 通道：MiBeacon V5 加密**广播**（网关收） + 0xFE95 GATT（只用于绑定）

> 只想知道原理不用烧板子：看 [`../t3pro_adv/README.md`](../t3pro_adv/README.md)。
> 本工程跟 `leak_adv` 一个骨架，只有 PID / 广播对象 / 按键动作三处不同，代码里都标了 ★。
>
> **2026-11-14 实机验收通过**：米家添加一次过（did `00 62 6C 74`，bound=1，
> 重启后串口仍打印「已绑定」）。

---

## 一、动手前先看这三行

| 你要做的 | 怎么做 |
|---|---|
| 甲醛 合格 ↔ 超标 | **短按一次 BOOT**（0.02 → 0.30 → 0.02，连发 3 帧防丢） |
| 换一块新板子重来 | **长按 BOOT 5 秒**（擦 NVS 恢复出厂，回到待绑定态） |
| 看串口 | `python -m serial.tools.miniterm COM6 115200`（**注意：开串口会复位板子**） |

---

## 二、烧录（三条命令）

```bash
idf.py set-target esp32c3 && idf.py build          # 或在容器里：见仓库 QUICKSTART.md
python -m esptool --chip esp32c3 -p COM6 erase-flash
python -m esptool --chip esp32c3 -p COM6 -b 460800 write-flash \
  --flash-mode dio --flash-freq 80m --flash-size 4MB \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x10000 build/hcho_adv.bin
```

启动日志应当长这样：

```
I (142) HCHO: ===== 甲醛检测仪 小白版（JQJCY01YM / 0x02DF）=====
I (332) BLE_INIT: Bluetooth MAC: 10:00:3b:ce:61:03
I (382) MIOT-GATT: 0xFE95 service registered (0010/0019/001A/001B)
I (392) HCHO: 协议栈就绪  MAC 10:00:3B:CE:61:03  未绑定（等待米家添加）
```

---

## 三、添加到米家

1. 手机蓝牙打开，米家 → `+` → 添加设备 → **附近设备**
2. 列表里出现 `JQJCY01YM`，点它 → 按提示走完
3. 串口出现 `REGISTER COMPLETE 0x11` + `bound=1` 就是绑上了

**扫不到**先查这三条：

| 现象 | 原因 |
|---|---|
| 列表里没有 | 这个 MAC 被别的工程绑过（米家按 MAC 去重）→ 改 `BASE_MAC_B4` 重烧 |
| 连上就断、App 报 `-27` | PID 与档案类型不匹配（本工程用的是真 PID，正常不会） |
| 添加成功但页面空 | 广播没被网关收到，看门狗/网关距离；或 `checkin` 前先等 4 秒一轮 |

---

## 四、改造成真实传感器

现在四路数据是写死的模拟值（`v_hcho / v_temp / v_hum / v_batt`）。
接真传感器只改 `app_main.c` 里 `★` 标的那一块：

- 甲醛：常见电化学模组（如 ZE08-CH2O / Dart WZ-S）走 UART，按量纲换算成 `0.01 mg/m³`
- 温度湿度：SHT30 / AHT20 走 I²C，温度 ×10、湿度 ×10 填 `v_temp / v_hum`
- 电量：分压电阻读 ADC，或干脆固定 100（USB 供电时本来就无电池）

改完不用碰广播/认证逻辑，`build_frame()` 会自动按新值组帧。

---

## 五、这个工程的边界

- **只做广播**：真机的历史曲线、实时长连接取数没实现，App 里看到的是「最近一次广播值」
- 四槽轮播是「一帧一个对象」，因为 BLE 单包 31B 装不下四个对象
- 删设备重加：先 **长按 BOOT 5 秒**恢复出厂，否则广播仍带 registered 态会被扫描过滤
