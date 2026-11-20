# 小米汽车磁吸补光灯（careco.nlight.o654 / 0x6266）—— 可调光 BLE 固件

**仓库首个「可调光可调色温」的本地 BLE 直连控制工程：App 详情页拖亮度/色温滑块、点场景 → 设备收 set → 回推 → 亮度/色温/开关态实时上广播。**

烧完板子在你的米家里就是一台「小米汽车磁吸补光灯」：米家 App 能发现、绑定，
详情页渲染开关 / 亮度滑块 / 色温滑块 / 场景按钮，操作会经 GATT 下发 `set_properties`，
设备落值后回推 `properties_changed`。

- 目标芯片：ESP32-C3（USB-Serial/JTAG 直连）
- 模拟型号：`careco.nlight.o654`，PID **0x6266**（resolve_type=3，广播可识别）
- spec：`urn:miot-spec-v2:device:night-light:0000A0AB:careco-o654:1`
- 通道：0xFE95 GATT（标准认证 + 属性读写）+ MiBeacon V5 加密广播（开关态 `0x0701`）
- **直连型设备**：控制走手机 GATT，不经过网关；云端 `isOnline` 常为 false 属正常

> 本工程由 `alarm_adv`（12 特征完整版 GATT 骨架）改造而来。骨架相同，只有 PID / 属性表 / 广播对象不同。

## 关键参数

| 项 | 值 |
|---|---|
| model | `careco.nlight.o654` |
| PID | **0x6266** |
| 扫描名 | `CAR-LIGHT` |
| NVS | `carlight` |
| MAC | base `10:00:3B:CE:87:03` → 空中 `10:00:3B:CE:87:05`（87 后缀账号内空闲） |
| 绑定 | 标准认证（0xA2 → ECDH → did → 0x11） |

## GATT 属性表（careco-o654 spec）

| 属性 | siid.piid | 类型 | 范围 | 读写 |
|---|---|---|---|---|
| battery-level | 5.1003 | uint8 | 0~100 | R |
| charging-state | 5.1 | uint8 | 0/1 | R |
| on | 7.1 | bool | 0/1 | **RW** |
| brightness | 7.2 | uint8 | 1~100 | **RW** |
| color-temperature | 7.3 | uint16 | 2700~5000 | **RW** |
| light-mode | 7.4 | uint8 | 0~4 | **RW** |
| device-mode | 8.1 | uint8 | 0~2 | **RW** |
| device-status.test | 8.2 | string | — | R（on_get 回空串） |
| time | 9.1 | uint32 | — | **RW** |

广播对象：`7.1 on` → `0x0701`（u8 0/1），随开关态轮换。

## 控制闭环（本工程核心）

```
App 详情页拖亮度/色温滑块 或 点场景
   → GATT 0x001A 写 set_properties（7.2 / 7.3 / 7.4 / 7.1）
   → miot_device_on_rx → miot_props_apply 落值到 v_bright / v_ct / v_light_mode / v_on
   → light_on_set 回调记 dirty(siid.piid)
   → 主循环检测 dirty → push_prop → send_props_changed 回推 App
```

实测（串口 + btsnoop 双证）：

| 操作 | 下发 | 回推 |
|---|---|---|
| 拖亮度滑块 | `spec SET 7.2 val=100` | `properties_changed 7.2` |
| 拖色温滑块 | `spec SET 7.3 val=3907→4409→4817`（实时连续） | `properties_changed 7.3` |
| 点「月光」场景 | `spec SET 7.4 val=2` | `properties_changed 7.4` |
| App 打开页面 | `spec GET 7.1/7.2/7.3/7.4/8.1/5.1003` 全 ok | 6 属性值回包 |
| App 对时 | `spec SET 9.1 val=<epoch>` | `properties_changed 9.1` |

## 两个关键坑（本工程踩过并修复）

### 1. NimBLE host 任务栈溢出（导致「连接失败 / 设备页一直转圈」）

App 连上发版本交换 F0 后，`miot_device_dispatch_plain` 在 `nimble_host` 任务栈上
开 `miot_spec_packet_t`(~1.7KB) + `handle_get` 的 `vals[]`，默认 4096B 栈直接：

```
Guru Meditation Error: Core 0 panic'ed (Stack protection fault)
Detected in task "nimble_host"
```

设备一收到 spec 请求就 panic 重启 → App 反复重连 → 表现为「连接失败」。
修复：`sdkconfig.defaults` 加 `CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE=8192`。

### 2. 登录完成瞬间 spec 应答投错特征（导致 App 收不到 F0）

登录完成的最后一条写是 0x0019（认证通道）。若 `s_rx_chan` 被认证帧更新成 0x0019，
设备紧接着主动发的 F0 版本交换 + 属性初值会被 `pick_chan` 投到 **0x0019**，
而磁吸灯 App 的 spec 引擎只在 **0x001A** 监听 → 收不到 F0 → 不发 spec START → 1s 后 0x213 断开。
修复：`gatt_miot.c` 里 `s_rx_chan` 只在「登录后的 spec 帧」才更新，认证帧不污染来路面。

### 3. 8.2 string 属性缺失（导致 App 初始化判失败、控件点击无反应）

App 初始化会 GET 8.2（`device-status.test`，string 类型）。属性表用 int32 存储表达不了 string，
若回 ERR，App 判设备异常，后续开关/亮度/色温点了都不发 SET。
修复：`dev_on_get` 回调对 8.2 回空串，让初始化流程走完。

## 串口命令

`on 0|1` / `bright N` / `ct N` / `mode N` / `info` / `factory`

## 编译烧录

```
. E:\ESP\esp_env.ps1
& $PYTHON $IDF_PY build
& $PYTHON $IDF_PY -p COM36 flash
```

烧录**不擦 NVS**（`idf.py flash`，不要 `erase-flash`），保留绑定身份，重刷后仍「已绑定」。
