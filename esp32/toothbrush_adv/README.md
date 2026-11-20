# 米家声波电动牙刷 T501（soocare.toothbrush.1501 / 0x1FF3）—— 广播型固件

**全新品类：电动牙刷。仓库首个 toothbrush 类设备，3 属性上云双向。**

烧完板子在你的米家里就是一台「米家声波电动牙刷 T501」：米家 App 能发现、绑定，
设备页渲染（电量 / 刷头剩余 / 充电状态 / AI 调节 / 健康报告）。

- 目标芯片：ESP32-C3（USB-Serial/JTAG 直连）
- 模拟型号：`soocare.toothbrush.1501`，PID **0x1FF3**（pdid 8179，pid=6，bt_bind_style=2，resolve_type=3，bt_match=1）
- 上报内容：电量 / 充电状态 / 刷头剩余寿命（三属性轮换广播 → 网关 → 云端）
- 通道：MiBeacon V5 加密广播（网关收）+ 0xFE95 GATT（绑定）
- display_type=64（标准 spec 设备）

> 本工程由 `plant_adv`（四槽轮换广播模板）改造而来。骨架相同，只有 PID / 对象号 / 槽位不同。

## 关键参数

| 项 | 值 |
|---|---|
| model | `soocare.toothbrush.1501` |
| PID | **0x1FF3**（pdid 8179） |
| 扫描名 | `SOOCARE-T501` |
| NVS | `toothbrush` |
| MAC | base `10:00:3B:CE:84:03` → 空中 `10:00:3B:CE:84:05`（84 后缀空闲） |
| 绑定 | 标准认证（0xA2 → ECDH → did → 0x11） |

> ★MAC 必须用账号内未占用后缀。本工程用 `84:05`。

## 对象号（新式 eid = 0x4000|(siid<<10)|(piid-1000)）

| 属性 | siid.piid | eid | 类型 | 范围 | 云端 |
|---|---|---|---|---|---|
| battery-level | 3.1003 | **0x4C03** | uint8 | 0~100 | 3:1003 |
| charging-state | 3.1034 | **0x4C22** | u8 enum | 0充电中/1满/2未充 | 3:1034 |
| brush-head-left | 4.1041 | **0x5029** | uint8 | 0~180 | 4:1041 |

> ★注意 piid≥1000 用新式 eid 公式，不是老式标准对象。
> `3.1034` → `0x4000|0x0C00|34` = **0x4C22**（34=0x22）。

URN：`urn:miot-spec-v2:device:toothbrush:0000A07E:soocare-1501:1`

## 串口命令

```
batt 0~100    设电量 %
chg 0|1|2     充电状态（0充电中 1已充满 2未充电）
head 0~180    刷头剩余寿命
fire 0|1|2    立刻发对应槽事件帧
info          状态（PID/MAC/bound/三属性）
objb/objc/objh 临时改对象号（排查用）
factory       擦 NVS 重启（删设备重加前必须执行）
```
短按 BOOT = 轮换触发一槽，长按 5s = 恢复出厂。

## 编译 / 烧录

```powershell
. E:\ESP\esp_env.ps1
cd C:\Users\MR\AppData\Local\Temp\pr3-smoke-fix\esp32\toothbrush_adv
python $env:IDF_PATH\tools\idf.py build
python $env:IDF_PATH\tools\idf.py -p COM36 erase-flash flash   # 首次必须擦
```

## 实测（2026-11-20）

- ✅ 米家「添加设备」页「发现的设备」自动列出「米家声波电动牙刷 T501」，绑定成功
- ✅ 扫描帧 `70 2a f3 1f 09 05 84 ce 3b 00 10`，PID `0x1ff3`=0x1FF3，MAC `...84:05`
- ✅ 上云 did `blt.3.1qu799k444000`，isOnline=true（网关型）
- ✅ 云端双向三轮全过：
  - 电量 `3:1003` 78→35 code=0
  - 充电 `3:1034` 2→1 code=0
  - 刷头 `4:1041` 120→90 code=0
- ✅ 设备页渲染「电量 / 刷头剩余 / 180次 / AI调节 / 查看健康报告」

## 为什么手环不行（选型边界）

手环（如小米手环9 Pro `miwear.watch.n67cn`）虽然也是 pid=6，但走 **MiWear 私有协议栈**
（spec_type 为空），绑定/数据通道与 MiBeacon spec 完全不同，不在本 SDK 模板能力范围。
牙刷是标准 MiBeacon spec（有正式 URN + notify/read 属性），故可做。
