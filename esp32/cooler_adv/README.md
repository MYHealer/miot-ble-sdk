# 小米磁吸散热背夹 可绑定固件 —— ESP32-C3

**可以直接编译烧录、被米家 App 绑定、设备页完整渲染并逐属性双向控制的固件。**
2026-11-23 实测：米家「添加设备」发现「磁吸散热背夹」→ 绑定 `REGISTER COMPLETE 0x11`
→ 设备页温度 25℃ 实时显示 + 智能/极寒模式切换 + 灯效全控。

- 目标芯片：ESP32-C3（USB-Serial/JTAG）
- 模拟型号：磁吸散热背夹 `xiaomi.cooler.r02p`，PID **0x52AA**（pdid 21162）
  - 官方产品库 `resolve_type=3` 广播可识别（米家自家）
  - spec：`urn:miot-spec-v2:device:cooler:0000A0DC:xiaomi-r02p:2`（14 属性）
- 通道：MiBeacon V5 加密广播（4 帧轮播 cool-mode/light-on/light-mode/temp → 网关）
  + 0xFE95 GATT（手机设备页）

## 属性与控制（全部实测）

| siid.piid | 名称 | App 控件 | 实测 |
|---|---|---|---|
| 3.1 | mode | 智能/极寒 | ✅ SET 3.1 val=1 → UI 切极寒 |
| 3.2 | extreme-cold-limit | 极寒防结露 | ok |
| 3.3 | temperature | 温度显示 | ✅ GET 实时 25℃；串口 `temp 45` → properties_changed 3.3 → UI 顶部大数字实时变 45℃ |
| 4.1 | light on | 灯效开关 | ✅ GET/回推 ok |
| 4.2 | light mode | 彩虹/呼吸/流光/明亮 | ✅ SET 4.2 val=1 → UI 切呼吸 |
| 4.3 | color | 主色 | ✅ GET ok |
| 4.4 | flow-speed | 慢速/中速/快速 | ✅ SET 4.4 val=0 → App set 落值 |
| 4.5 | color-combination | 全彩/三色/单色 | ✅ SET 4.5 val=1 → UI 切三色 |
| 4.6/4.7/4.8 | color1/2/3 | 取色器 | ✅ SET 4.6 type=5 U32 val=7530952 → 回推换色 |
| 5.1 | delay | 延时关闭开关 | ✅ SET 5.1 type=0 val=1 → 落值+回推 |
| 5.2 | delay-time | 延时时长 | ✅ SET 5.2 type=3 val=10 → 10分钟 |
| 6.1 | highlights | 高亮模式 | ok |

★两处 SDK 通用坑（本设备实测暴露）：
1. **I8 编码缺失**：`miot_spec_encode_value` / `fill_value` / `miot_props_encode`
   都没有 `MIOT_TYPE_I8` case → GET 3.3 回包和初值推送整帧被丢 → App 温度 `--`、
   模式不显示。补齐 I8/I16 case 后全通。
2. **BOOL SET 解析误判**：延时关闭 5.1 的新格式帧 `rawt=(0<<12)|1=0x0001`
   高 4 位为 0，被旧格式兼容分支误判 → 越界 `rc=-4` → 整帧丢弃 → App 无限重发
   卡页面。用 `remain` 是否精确等于新格式长度判别（host 旧格式 remain=7、
   App 新格式 remain=6），两种格式兼容。

## 编译 / 烧录

```bash
cd esp32/cooler_adv
idf.py build
idf.py -p COM36 flash        # 不要 erase-flash（丢绑定身份）
```

## 米家添加

米家 → **+** → **添加设备** → 发现列表出现「磁吸散热背夹」→ 点卡片 →
`REGISTER COMPLETE 0x11` → 完成房间/命名。

## 串口命令

| 命令 | 作用 |
|---|---|
| `mode 0\|1` | 智能/极寒 |
| `temp N` | 传感器温度 -30~80℃（触发 properties_changed 回推） |
| `info` / `factory` | 状态 / 擦身份重绑 |

BOOT 键：短按切换开关；长按 5s 恢复出厂。

## GATT / SDK 修复

GATT 层与 sdkconfig 来自 `carlight_adv` 完整修复版：12 特征值真机对齐、
登录后 `s_rx_chan` 只在 spec 帧更新、NimBLE host/main 栈 8192。
u16 属性（5.2 delay-time）走 SDK 按 size 精确读写。
