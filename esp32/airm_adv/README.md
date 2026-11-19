# 树色空气检测仪（shuse.airm.ss006 / 0x6651）小白版固件 —— ESP32-C3

**一根 Type-C 线、几条串口命令就能用。**

烧完这块板子在你的米家里就变成一台「树色空气检测仪（BLE）」：米家 App 能添加它、
设备页显示温湿度 / 甲醛 / CO₂ / TVOC / AQI。

- 目标芯片：ESP32-C3（USB-Serial/JTAG 直连）
- 模拟型号：`shuse.airm.ss006`，PID **0x6651**
- 上报：温度/湿度/AQI/CO2 走 **beacon→网关→云端** 双向；HCHO/TVOC 见「已知边界」
- 通道：MiBeacon V5 加密广播 + 0xFE95 GATT（绑定 + 属性表）

> 骨架同 `senpres_adv` / `plant_adv`。原理见 [`../t3pro_adv/README.md`](../t3pro_adv/README.md)。

---

## 一、串口命令

| 命令 | 作用 |
|---|---|
| `temp 32.5` | 温度 ℃（f32） |
| `hum 55` | 湿度 % |
| `aqi 77` | AQI |
| `co2 650` | CO₂ ppm |
| `tvoc 0.22` | TVOC mg/m³ |
| `hcho 0.08` | 甲醛 mg/m³ |
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

| 属性 | siid.piid | 对象 | 云端 | 网关 beacon 双向 |
|---|---|---|---|---|
| 温度 | 3.1001 | **0x4C01** f32 | `3:1001` | OK |
| 湿度 | 3.1008 | **0x4C08** f32 | `3:1008` | OK |
| AQI | 3.1028 | **0x4C1C** u16 | `3:1028` | OK |
| CO2 | 3.1029 | **0x4C1D** u16 | `3:1029` | OK |
| TVOC | 3.1031 | **0x4C1F** f32 | `3:1031` | 初值可见，改值不上云 |
| 甲醛 | 3.1030 | **0x4C1E** f32 | `3:1030` | 初值可见，改值不上云 |

公式：`eid = 0x4000|(siid<<10)|(piid-1000)`。

---

## 五、已知边界（HCHO / TVOC）

实测（2026-11-19）：

- **温度 / 湿度 / AQI / CO2**：串口改值 → beacon → 网关 → 云端，双向 code=0。
- **HCHO / TVOC**：设备页能显示初值；串口改值后 **云端 updateTime 不变**。
  已排除：错误对象号（试过 0x4C1E/0x1010）、GATT `properties_changed` 三连推、
  直连会话窗口。网关/云端对 3.1030/3.1031 **上报不落库**。
- 设备页断开直连后显示 **「蓝牙网关已连接」**——数据走网关，不是纯 GATT 直连设备。

验证上云前请 `force-stop` 米家（设备页占 BLE）。

---

## 六、实测摘要（did `blt.3.1qtul47fk4o00`）

| 串口 | 云端 |
|---|---|
| `temp 32.5` | `3:1001=33`（API 取整）code=0 |
| `hum 55` | `3:1008=55` code=0 |
| `aqi 77` | `3:1028=77` code=0 |
| `co2 650` | `3:1029=650` code=0 |
