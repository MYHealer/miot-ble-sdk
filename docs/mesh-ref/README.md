# mesh-ref — 米家 BLE Mesh 参考资料

这里放的是**外部资料与协议研究**，不是 SDK 的实现代码。仓库主体（`src/` + `esp32/`）做的是
BLE GATT 直连设备模拟（24 台设备，见根 [README](../../README.md)），mesh 是另一套形态，
本目录只做资料沉淀，供需要时查阅。

## 官方文档

| 文件 | 内容 |
|---|---|
| [08-mesh-intro.md](08-mesh-intro.md) | 小米 BLE Mesh 接入介绍（官方）：网络特性、模组能力、模组 API |
| [09-mesh-dev.md](09-mesh-dev.md) | 小米 BLE Mesh 开发者指南（官方）：MIoT Spec 与 Mesh Spec 的映射、模组 + 固件 SDK |
| [official-findings.md](official-findings.md) | 官方资料要点汇总（含广嵌 GQ-MKM6 方案手册里查不到的实现细节） |

## 协议研究

| 目录 / 文件 | 内容 |
|---|---|
| [provisioning/](provisioning/) | ★ 配网协议逆向笔记：7 篇，从广播握手到 mesh 密钥下发，含实测边界（20s 链路寿命、MAC 校验范围）与中继工程实践 |
| [blemesh_app_mesh.c](blemesh_app_mesh.c) | nRF Mesh SDK 侧的 app_mesh 骨架（Zephyr/Nordic 参考实现） |
| [mijia-sdk/](mijia-sdk/) | 米家 mesh 模组 SDK 的最小工程骨架（CMake + main.c + 配置头） |
| [gw3_mesh.py](gw3_mesh.py) | XiaomiGateway3 网关侧 mesh 数据库层解析脚本 |
| [gw3_conv_mesh.py](gw3_conv_mesh.py) | 同上，负责 mesh 消息 ↔ 网关 JSON 的转换 |

## 两种形态的取舍

| | BLE GATT 直连（仓库主体） | BLE Mesh（本目录） |
|---|---|---|
| 云端下发 | ❌ 只有手机在附近才能控制 | ✅ 网关转发，可远程 |
| 身份门槛 | 设备证书（已解决，24 台实机通过） | 设备私钥签 `pkg4`（软件不可复刻） |
| 可复刻性 | ✅ 高，协议已完整逆向 | ⚠️ 只能走官方模组（自带合法证书） |
| 适用场景 | 手边有手机、要接米家 App 和自动化 | 要做真正能远程/语音控制的量产品 |

结论写在 [provisioning/06-security-analysis.md](provisioning/06-security-analysis.md)：mesh 的身份校验
把私钥作为唯一不可替代的输入，空口链路不泄露，派生死路已证伪，剩下的只有拆机 dump 或官方模组。
