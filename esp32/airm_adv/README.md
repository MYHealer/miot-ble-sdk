# 树色空气检测仪（shuse.airm.ss006 / 0x6651）小白版固件 —— ESP32-C3

**一根 Type-C 线、几条串口命令就能用。**

烧完这块板子在你的米家里就变成一台「树色空气检测仪（BLE）」：米家 App 能添加它、
设备页显示温湿度 / 甲醛 / CO₂ / TVOC / AQI。

- 目标芯片：ESP32-C3（USB-Serial/JTAG 直连）
- 模拟型号：`shuse.airm.ss006`，PID **0x6651**
- 上报：六属性走 **beacon→网关→云端** 双向（HCHO/TVOC 见硬规则）
- 通道：MiBeacon V5 加密广播 + 0xFE95 GATT（绑定 + 属性表）

> 骨架同 `senpres_adv` / `plant_adv`。原理见 [`../t3pro_adv/README.md`](../t3pro_adv/README.md)。

---

## 一、串口命令

| 命令 | 作用 |
|---|---|
| `temp 32.5` | 温度 ℃（云端按 spec **步长 1** 取整） |
| `hum 55` | 湿度 % |
| `aqi 77` | AQI |
| `co2 650` | CO₂ ppm |
| `tvoc 0.22` | TVOC mg/m³（**必须 ∈[0,2]**） |
| `hcho 0.08` | 甲醛 mg/m³（**必须 ∈[0,2]**） |
| `fire 0-5` | 只发指定槽事件帧 |
| `info` / `factory` | 状态 / 恢复出厂 |

---

## 二、编译烧录

```bash
. E:\ESP\esp_env.ps1
cd E:\ESP\c3supermini\miot_sdk\esp32\airm_adv
python E:\ESP\.espressif\v5.5.4\esp-idf\tools\idf.py build
python E:\ESP\.espressif\v5.5.4\esp-idf\tools\idf.py -p COM36 flash
```

---

## 三、米家添加

1. **+ → 添加设备**（菜单第一项）
2. 发现 **树色空气检测仪（BLE）** → 点它 → 绑定完成
3. 设备页：温湿度 / 甲醛 / CO₂ / TVOC +「已连接」或「蓝牙网关已连接」

> `bt_match=0` 但 `resolve_type=3`，实测**扫描仍能出型号名**。
> 若删设备后扫不到：先删除云端旧设备（MAC 去重）再 factory 重绑。

---

## 四、对象号（新式 eid 公式）

| 属性 | siid.piid | 对象 | 云端 | 官方 range | 网关 beacon 双向 |
|---|---|---|---|---|---|
| 温度 | 3.1001 | **0x4C01** f32 | `3:1001` | [-30,100] step 1 | OK |
| 湿度 | 3.1008 | **0x4C08** f32 | `3:1008` | [0,100] 0.1 | OK |
| AQI | 3.1028 | **0x4C1C** u16 | `3:1028` | [0,600] | OK |
| CO2 | 3.1029 | **0x4C1D** u16 | `3:1029` | [0,8000] | OK |
| TVOC | 3.1031 | **0x4C1F** f32 | `3:1031` | **[0,2] 0.01** | OK（~195s） |
| 甲醛 | 3.1030 | **0x4C1E** f32 | `3:1030` | **[0,2] 0.01** | OK（~135s） |

公式：`eid = 0x4000|(siid<<10)|(piid-1000)`。

---

## 五、HCHO/TVOC 硬规则（实测）

1. **能上报**，延迟约 **2~3 分钟**（满密度单槽 180 帧连发 + 45s 自动补发）。
2. **值必须 ∈ [0, 2]**（官方 spec）。超范围（如 3.0）→ 云端 `-704220043`，
   读口被打坏；**再发合法值**满密度连发约 135s 恢复。
3. 双槽 **顺序排队不打断**：`fire 5` 后再 `fire 3`，前槽跑满再接续；
   逐帧交替半密度进不去。
4. GATT `properties_changed` 刷不动云端；数据走 **beacon→网关**。
5. 验证前 `force-stop` 米家（设备页占 BLE）。

---

## 六、实测摘要（did `blt.3.1qtul47fk4o00`，2026-11-20）

| 串口 | 云端 |
|---|---|
| `temp 34.5` | `3:1001=35`（step 1）code=0 |
| `hcho 0.42` + `fire 5` | `3:1030=0.42` @135s code=0 |
| `tvoc 0.66` + `fire 3` | `3:1031=0.66` @195s code=0 |
| `hcho 3.15`（超范围） | `-704220043`，合法值覆盖后恢复 |
