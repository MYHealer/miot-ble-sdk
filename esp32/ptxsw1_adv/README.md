# PTX 无线开关（蓝牙版）可绑定固件 —— ESP32-C3

**可以直接编译烧录、被米家 App 扫描识别并绑定、四种按键/状态事件上报的固件。**
2026-11-23 实测：米家「添加设备」发现列表出现「PTX 无线开关（蓝牙版）」→ 绑定
`REGISTER COMPLETE 0x11` → 云端下发 did（`blt.3.1qv3uu4s04c00`）→ `bound=1`。

- 目标芯片：ESP32-C3（USB-Serial/JTAG，一根 Type-C 线）
- 模拟型号：PTX 无线开关（蓝牙版）`090615.remote.btsw1`，PID **0x38BB**（pdid 14523）
  - 来源：米家官方产品库 `plugin_config_cn-zh_cn`，`resolve_type=3`（广播可扫描识别）
  - spec：`urn:miot-spec-v2:device:remote-control:0000A021:090615-btsw1:1`
    （siid 2 switch-sensor / siid 3 battery，properties 表为空，事件对象号靠实测）
- **四个自动化触发事件**（用户实证）：
  - 单击 → `0x4E0C`
  - 双击 → `0x4E0D`
  - 长按 → `0x4E0E`
  - 低电量 → `0x4C01`（按 MiBeacon2 公式 `0x4000|(3<<10)|(1001-1000)` 算，串口 `lowbatt` 触发）
- 常态心跳：电量 `0x100A` u8 %；电量 ≤15% 时快广播补发低电事件
- 通道：MiBeacon V5 加密广播（网关/自动化用）+ 0xFE95 GATT（手机绑定用，12 特征值真机对齐）

GATT 层与 sdkconfig 基于 `carlight_adv` 的完整修复版（12 特征值真机对齐、
登录后 `s_rx_chan` 只在 spec 帧更新避免来路面污染、NimBLE host / main 栈 8192）。

## 编译

```bash
cd esp32/ptxsw1_adv
idf.py set-target esp32c3   # sdkconfig.defaults 已预设可跳过
idf.py build
```

## 烧录

```bash
idf.py -p COM36 flash
```

不要 `erase-flash`（会丢绑定身份）。换型号/换 MAC 后若米家按 MAC 去重扫不到，
用串口 `factory` 擦身份重绑。

## 米家添加（唯一入口）

米家 → 右上角 **+** → **添加设备** → 发现列表出现「PTX 无线开关（蓝牙版）」→ 点卡片，
等串口 `REGISTER COMPLETE (0x11)` → `bound=1` → 完成房间/命名。

⚠️ 只能等「发现的设备」自动列出后点卡片，不要用「扫一扫 / 手动添加 / 扫描设备」。

## 串口命令

| 命令 | 作用 |
|---|---|
| `click` / `double` / `hold` / `lowbatt` | 触发单击/双击/长按/低电量事件帧（重复 2.5s） |
| `batt N` | 设电量；≤15 时自动触发低电量事件 + 快广播 |
| `obj C D H L B` | 运行时改五个事件/电量对象号（十六进制，盲测用） |
| `info` | 打印 PID / 绑定态 / 电量 / 各对象号 / fcnt |
| `factory` | 擦 NVS 重启（重绑用） |

板载 BOOT 键：单击 `<1s` → 单击；`400ms` 内二次 → 双击；按住 `≥1s` → 长按；`≥5s` → 恢复出厂。

## 验收清单（PC 可测）

```bash
python verify_ptxsw1.py <串口日志路径>    # 静态配置 + 动态四事件帧检查
```

| 步骤 | 判据 |
|---|---|
| 烧完启动横幅 | `PID=0x38BB ... click=0x4E0C double=0x4E0D hold=0x4E0E lowbat=0x4C01` |
| 米家添加 | 发现列表出现型号名 → `REGISTER COMPLETE 0x11` → `bound=1`、云端有 did |
| 四事件 | 串口 `click/double/hold/lowbatt` 分别发 `0x4E0C/0x4E0D/0x4E0E/0x4C01`，2.5s 后回电量帧 |
| 自动化 | 创建自动化 → 触发条件选该设备，出现单击/双击/长按/低电量四项 |
