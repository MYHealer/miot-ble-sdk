# smoke2_adv — 小米烟感卫士2（xiaomi.sensor_smoke.yg2 / 0x7ABA）

广播型 BLE 模拟：被官方米家 App 发现 → 绑定 → status / battery-state / temperature 三属性上云双向。

## 实测结论（2026-11-20）

- ✅ 米家「添加设备」自动列出「小米烟感卫士2」，绑定 3/3
- ✅ 设备页显示「监测正常 / 持续监测中 / 电池 电量充足」
- ✅ 云端双向：`2:1004` status 1↔2、`6:1132` battery 0↔1
- 上云 did：`blt.3.1qu5tn44o4c00`

## 关键参数
| 项 | 值 |
|---|---|
| model | `xiaomi.sensor_smoke.yg2` |
| PID | **0x7ABA**（pdid 31418） |
| 扫描名 | `SMOKEYG2` |
| NVS | `smoke2` |
| MAC | base `10:00:3B:CE:7C:03` → 空中 `10:00:3B:CE:7C:05` |
| resolve_type | 3 · bt_bind_style 2 |

> ★MAC 必须用账号内**未被占用**的后缀。`78:05`(宠物饮水机)/`77:05`(烟雾传感器) 已被占用，
> 米家按 MAC 去重会直接过滤 → 表现为「扫不到」。本工程用 `7C:05`。

## 对象号（eid = 0x4000\|(siid<<10)\|(piid-1000)）
| 属性 | siid.piid | eid | 载荷 |
|---|---|---|---|
| status | 2.1004 | **0x4804** | u8 enum **1=监测正常 2=烟雾告警** |
| battery-state | 6.1132 | **0x5884** | u8 enum **0=充足 1=不足** |
| temperature | 2.1001 | **0x4801** | f32 ℃ |

> 与小白版 s500（0x392E / fire-alarm 0x482B bool）不同：yg2 主状态是 **enum**，不是 bool。

## 串口命令
```
status 1|2     监测正常 / 烟雾告警
batt 0|1       电量充足 / 不足
temp 25.5      温度
toggle         status 在 1↔2 切换
info           PID/MAC/bound/状态
factory        擦绑定身份重启
```
短按 BOOT = toggle；长按 5s = 出厂。

## 云读（PC）
```python
miot_device_status_read(did, prop="2:1004")  # status: 1正常 2告警
miot_device_status_read(did, prop="6:1132")  # battery-state: 0充足 1不足
```
