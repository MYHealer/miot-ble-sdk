# 米家夜灯2（MUE4094RT / 0x03DD）小白版独立固件 —— ESP32-C3

> **状态：编译通过（623,280 字节 ≈ 609KB），链路来自 `esp32/multi_dev` 的实机验证，尚未在本工程上复测。**
> 这是全仓库唯一一台**详情页不走标准 spec 通道**的设备：绑定走 legacy token，
> 控制走 ag2 通道上的 **NUS 文本协议**。所以它比门磁/人体那几台长得多，也更容易踩坑。

- 目标芯片：ESP32-C3（USB-Serial/JTAG 直连）
- 模拟型号：米家夜灯2 `MUE4094RT`，PID **0x03DD**
- 属性：`2.1` 亮度 %(1~100) · `2.2` 延时 ms(5000~60000) · `2.3` 开关 · `3.1` 电量
- 通道：0xFE95 绑定（legacy token + RC4 应答） + ag2（001A/001B）建链 +
  NUS `6E400001/2/3` 收发文本命令
- 派生 MAC：`10:00:3B:CE:70:03`，NVS 命名空间 `nightl2`（与 multi_dev 的 `miot` 隔离）

---

## 一、这台设备为什么特殊

| 阶段 | 夜灯2 走什么 | 门磁/人体走什么 |
|---|---|---|
| 广播 BondAbility | **1（旧版绑定）** | 2（标准认证） |
| 绑定 | App 下发 12B token → BOND → 20B DeviceID → GETRAND → LOGIN | 0xA2 → ECDH → did → 0x11 |
| 建链 | 订阅 001A/001B 后设备主动发 ag2 `mng` → App `mng_ack` → 测试包 → READY | 直接 spec 通道 |
| 数据 | device info（did + 24B payload），分帧 80ms/帧 | properties_changed 帧 |
| 控制 | NUS 文本：`get_prop ...` / `set_bright 58` / `apply_fixed_scene N` | spec GET/SET |

对应代码：`gatt_miot.c` 里的 NUS 段（约 1545 行起）与 `app_main.c` 的 legacy/ag2 分支。
**改这个工程前先读 [`docs/nightl2-ag2-handoff.md`](../../docs/nightl2-ag2-handoff.md)**，
那份文档把「试过哪些死路」写全了，能省掉几天。

## 二、编译

```bash
cd esp32/nightl2_adv
idf.py build
```

Docker（没装 IDF）：

```bash
docker run --rm -v "D:/workspace/temp/miot-ble-sdk:/project" -w /project/esp32/nightl2_adv \
  espressif/idf:v6.0 bash -c "idf.py build"
```

> 本工程**不需要** Mesh 移植补丁（`build_mesh.sh` 那套），`sdkconfig.defaults` 里已关掉 Mesh。

## 三、烧录（必须擦）

```bash
idf.py -p COM6 erase-flash
idf.py -p COM6 flash monitor
```

启动日志：

```
I (xxx) NIGHTL: ===== 夜灯2 小白版（MUE4094RT / 0x03DD）=====
I (xxx) NIGHTL: 用法：米家 App → + → 添加设备 → 点 MUE4094RT → 绑定
I (xxx) NIGHTL: MAC for [0] 夜灯2 (MUE4094RT): 10:00:3B:CE:70:03
I (xxx) NIGHTL: profile loaded: 夜灯2 (MUE4094RT) (4 props)
```

## 四、在米家里添加与验收

| 步骤 | 期望 | 不通过先查 |
|---|---|---|
| 1 | 「附近设备」出现 **MUE4094RT** | `erase-flash` 了吗；MAC 是否 70:03（与其它工程不撞） |
| 2 | 点添加 → 绑定成功 | 串口应有 `AUTH_INIT` → `token` → `BOND` → `DeviceID` → `LOGIN`；缺哪步看哪步 |
| 3 | 进详情页（可能第一次超时） | **偶发「连接超时」是已知现象**，退回再进一次；串口看 `ag2 mng → READY` |
| 4 | 拖亮度到 58 | 串口 `NUS rx: set_bright 58` → `NL tx: {"flag":"set_bright","result":{"bright":58}}` |
| 5 | 开关灯 / 改延时 / 点场景 | 同上，分别对应 `set_power` / `set_dur` / `apply_fixed_scene` |
| 6 | 删设备重加 | **必须先串口 `factory`**（带 token 时广播 BondAbility=0x29，App 会过滤掉设备） |

> 页面显示「未连接蓝牙网关」不影响手机直连控制；但自动化触发需要网关。

## 五、串口命令

| 命令 | 作用 |
|---|---|
| `info` | 绑定/登录/广播状态 |
| `nlstat` | 插件侧状态：亮度、延时、场景、已收命令数 |
| `nl set_bright 58` | 本机模拟插件命令（不开手机也能验证解析与应答） |
| `nl get_prop power bright lux snm dur` | 查询四属性 |
| `nl apply_fixed_scene 2` | 固定场景 1~4 |
| `advu 1/0` | 强制待绑定 / 已绑定广播（默认 1，见下） |
| `factory` | 擦 NVS 重启，回到未绑定 |
| `newmac` | 换一个全新 BLE 地址（地址被米家拉黑时用） |

## 六、与 multi_dev 的差异

| 项 | multi_dev | 本工程 |
|---|---|---|
| 型号 | 11 个档案运行期切换 | 只有夜灯2 |
| Mesh | NimBLE Mesh + 5 个移植补丁 | **不编译** |
| NVS | `miot` / `miotN` | `nightl2` |
| MAC | `…:40+sel:xx` | `…:70:03`（`newmac` 可换） |
| 默认广播 | 已绑定形态 | **强制待绑定形态（`s_adv_unbound=1`）** |

最后一项是现场结论：唯一一次全链路登录成功，发生在设备广播待绑定帧的时候；
广播已绑定帧时 App 订阅完 0x0001 就 1 秒断开。若你的账号下页面反而打不开，
试 `advu 0` 对比。

## 七、已知边界（真机同样存在）

1. **详情页偶发「连接超时」**：打开页面瞬间抢不到连接，退一次再进即可。
2. **删设备重加必须先 `factory`**：否则广播带 token，App 判断「已绑定」直接过滤。
3. **没有「如果」触发条件**：云端产品定义只挂了属性，没有 event。
4. **重新绑定走 legacy PSK**：本工程实现的是 legacy token 流程；若你的米家版本
   在重绑时要求 PSK（`smartcfg-*`），会卡住 —— 见 [psk-auth.md](../../docs/psk-auth.md)。
5. **没有真实 LED 驱动**：亮度/延时只改属性与 NUS 应答，接灯珠在 `nl_on_any()` 里加。

## 免责声明

见仓库根目录 [NOTICE.md](../../NOTICE.md)。本项目仅用于学习研究，设备状态为模拟数据；
不得用于冒充真实商品设备、绕过米家平台安全机制或其他违反平台协议的用途。
