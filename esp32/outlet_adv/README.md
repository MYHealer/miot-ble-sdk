# 闪极100W迷你充电器（蓝牙版）（sharge.plug.sj01 / 0x1E1C）—— 控制类 BLE 固件

**仓库首个「主动控制类」独立工程：App 详情页点开/关 → 设备收 set → 回推 → 开关态上广播。**

烧完板子在你的米家里就是一台「闪极 100W 迷你充电器（蓝牙版）」：米家 App 能发现、绑定，
详情页渲染插座开关，点开关会经 GATT 下发 `set_properties`，设备落值后回推 `properties_changed`。

- 目标芯片：ESP32-C3（USB-Serial/JTAG 直连）
- 模拟型号：`sharge.plug.sj01`，PID **0x1E1C**（pdid 7708，pid=6，bt_bind_style=2，resolve_type=3）
- 通道：0xFE95 GATT（标准认证 + 属性读写）+ MiBeacon V5 加密广播（开关态 `0x1001`）
- **直连型设备**：控制走手机 GATT，不经过网关；因此云端 `isOnline` 常为 false，属正常现象

> 本工程由 `nl1_adv`（具备完整 set→on_set→回推闭环）改造而来。骨架相同，只有 PID / 属性表 / 广播对象不同。

## 关键参数

| 项 | 值 |
|---|---|
| model | `sharge.plug.sj01` |
| PID | **0x1E1C**（pdid 7708） |
| 扫描名 | `SHARGE` |
| NVS | `outlet` |
| MAC | base `10:00:3B:CE:7D:03` → 空中 `10:00:3B:CE:7D:05`（7D 后缀账号内空闲） |
| 绑定 | 标准认证（0xA2 → ECDH → did → 0x11） |

> ★MAC 必须用账号内未占用后缀。选前先云查 `device_list_page` 的 mac。
> 已占用：78:05 / 77:05 / 7C:05 等；本工程用 `7D:05`。

## GATT 属性表（sharge-spec 私有 spec）

| 属性 | siid.piid | 类型 | 范围 | 读写 |
|---|---|---|---|---|
| on | 2.1 | bool | 0/1 | **RW** |
| timer | 2.7 | uint8 | 0~24 h | **RW** |
| timer-off | 2.8 | uint8 | 0~24 h | **RW** |
| left-time | 2.9 | uint8 | 0~24 | R |

广播对象：`2.1 on` → `0x1001`（u8 0/1），随开关态轮换。

## 控制闭环（本工程核心）

```
App 详情页点开/关
   → GATT 0x0010/0x001A 写 set_properties（2.1）
   → miot_device_on_rx → miot_props_apply 落值到 v_on
   → outlet_on_set 回调记 dirty(2.1)
   → 主循环检测 dirty → push_prop → send_props_changed 回推 App
   → 同时 build_frame 把新开关态写进 0x1001 广播帧
```

与传感器类的本质区别：传感器属性是**设备侧上报**（串口设值→上云），
控制类属性是 **App 侧下发**（set→落值→回推）。

## 串口命令

```
on 0|1       设开关（回推 App + 广播）
toggle       切换开关
timer 0~24   定时开启（小时）
info         状态（PID/MAC/bound/on/timer）
obj XXXX     临时改广播对象号（排查用）
factory      擦 NVS 重启（删设备重加前必须执行）
```
短按 BOOT = 开关切换，长按 5s = 恢复出厂。

## 编译 / 烧录

```powershell
. E:\ESP\esp_env.ps1
cd C:\Users\MR\AppData\Local\Temp\pr3-smoke-fix\esp32\outlet_adv
python $env:IDF_PATH\tools\idf.py build
python $env:IDF_PATH\tools\idf.py -p COM36 erase-flash flash   # 首次必须擦
```

> ★`sdkconfig.defaults` 的 `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192`：会话建立即推 4 条属性初值，
> 默认 3584B 会 Stack protection fault。

## 实测（2026-11-20）

- ✅ 米家「添加设备」页「发现的设备」自动列出「闪极 100W 迷你充电器（蓝牙版）」
- ✅ 标准认证 → 连接设备成功 → 设备绑定账号成功 → 添加成功（3/3）
- ✅ 扫描帧 `70 2a 1c 1e 0d 05 7d ce 3b 00 10`，PID `0x1c1e`=0x1E1C，MAC `...7D:05`
- ✅ `miot_props` 控制表 PC 单测：2.1 set 落值 + on_set 回调 + 2.9 只读拒绝 + 2.7 clamp 全通过
- 上云 did：`blt.3.1qu6o7kf04002`（直连型，网关不转发，云读需 App 在线会话）

## 直连型 vs 网关型（选型结论）

| 类型 | 控制通道 | 云端 isOnline | 例子 |
|---|---|---|---|
| 标准 BLE 直连（pid=6） | 手机 GATT 直连 | 常 false | 本插座、夜灯、风扇 |
| BLE Mesh（pid=16） | 网关 | true，可云控 | 需绑定证书私钥，无私钥不可达 |
| WiFi（pid=8） | 云端 | true | 公牛插座等，非 BLE |

全库 BLE 插座仅 `sharge.plug.sj01`(0x1E1C) 一款；灯类控制为 `yeelink.light.nl1`(0x07F6)
和 `yeelink.light.ble1`(0x0053，卡授权)。故本工程是 BLE 主动控制类最简可行样本。
