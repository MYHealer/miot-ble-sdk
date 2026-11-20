# 米家伺服扫振电动牙刷 Pro（xiaomi.toothbrush.24p / 0x6800）—— 广播型固件

**小米旗舰牙刷：伺服扫振 Pro。UI 比 T501 更丰富（本周刷牙情况 / 当前模式 / 覆盖率 / 参数设置）。**

烧完板子在你的米家里就是一台「米家伺服扫振电动牙刷 Pro」：米家 App 能发现、绑定，
设备页渲染（本周刷牙情况 / 当前模式「柔和」/ 覆盖率 / 时长 / 参数设置 / 个性模式）。

- 目标芯片：ESP32-C3（USB-Serial/JTAG 直连）
- 模拟型号：`xiaomi.toothbrush.24p`，PID **0x6800**（pdid 26624，pid=6，bt_bind_style=2，bt_match=1，resolve_type=3）
- 上报内容：电量 / 充电状态 / 刷头剩余寿命（三属性轮换广播 → 网关 → 云端）
- 通道：MiBeacon V5 加密广播（网关收）+ 0xFE95 GATT（绑定）
- display_type=64（标准 spec 设备）

> 本工程由 `toothbrush_adv`（T501）改造而来。骨架相同，只有 PID / 刷头范围 / MAC 不同。

## 关键参数

| 项 | 值 |
|---|---|
| model | `xiaomi.toothbrush.24p` |
| PID | **0x6800**（pdid 26624） |
| 扫描名 | `XIAOMI-24P` |
| NVS | `brushpro` |
| MAC | base `10:00:3B:CE:85:03` → 空中 `10:00:3B:CE:85:05`（85 后缀空闲） |
| 绑定 | 标准认证（0xA2 → ECDH → did → 0x11） |

## 对象号（新式 eid = 0x4000|(siid<<10)|(piid-1000)）

| 属性 | siid.piid | eid | 类型 | 范围 | 云端 |
|---|---|---|---|---|---|
| battery-level | 3.1003 | **0x4C03** | uint8 | 0~100 | 3:1003 |
| charging-state | 3.1034 | **0x4C22** | u8 enum | 0充电/1满/2未充 | 3:1034 |
| brush-head-left | 4.1041 | **0x5029** | uint8 | **0~100** | 4:1041 |

> ★0x6800 的 brush-head 范围是 **[0,100]**（T501 是 [0,180]）。发 120 会云端拒收
> `-704220043`，必须 ≤100。

URN：`urn:miot-spec-v2:device:toothbrush:0000A07E:xiaomi-24p:1`

## 串口命令

```
batt 0~100    设电量 %
chg 0|1|2     充电状态（0充电中 1已充满 2未充电）
head 0~100    刷头剩余寿命
fire 0|1|2    立刻发对应槽事件帧
info          状态（PID/MAC/bound/三属性）
objb/objc/objh 临时改对象号（排查用）
factory       擦 NVS 重启（删设备重加前必须执行）
```
短按 BOOT = 轮换触发一槽，长按 5s = 恢复出厂。

## 编译 / 烧录

```powershell
. E:\ESP\esp_env.ps1
cd C:\Users\MR\AppData\Local\Temp\pr3-smoke-fix\esp32\brushpro_adv
python $env:IDF_PATH\tools\idf.py build
python $env:IDF_PATH\tools\idf.py -p COM36 erase-flash flash   # 首次必须擦
```

## 实测（2026-11-20）

- ✅ 米家「添加设备」页「发现的设备」自动列出「米家伺服扫振电动牙刷 Pro」，绑定成功
- ✅ 扫描帧 `70 2a 00 68 04 05 85 ce 3b 00 10`，PID `0x6800`，MAC `...85:05`
- ✅ 上云 did `blt.3.1qu7mbaes4c02`，isOnline=true（网关型）
- ✅ 云端双向：电量 `3:1003` 78→66、充电 `3:1034` 2→1、刷头 `4:1041` 80，全部 code=0
- ✅ 设备页渲染「本周刷牙情况 / 当前模式 柔和 / 覆盖率 / 参数设置 / 个性模式」

## 与 T501（0x1FF3）的差异

| 项 | T501 | 伺服扫振 Pro |
|---|---|---|
| pdid | 8179 | 26624（更新，2025+ 旗舰） |
| brush-head 范围 | [0,180] | **[0,100]** |
| 设备页 UI | 电量/刷头/AI调节/健康报告 | 本周刷牙情况/当前模式/覆盖率/参数设置/个性模式（更丰富） |

## 为什么手环不行（选型边界）

手环（如小米手环9 Pro `miwear.watch.n67cn`）虽 pid=6，但走 **MiWear 私有协议栈**（spec_type 空），
不在本 SDK 模板范围。牙刷是标准 MiBeacon spec，故可做。
