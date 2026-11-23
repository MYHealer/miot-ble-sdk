/**
 * @file app_main.c
 * @brief PTX 无线开关（蓝牙版）（090615.remote.btsw1 / PID 0x38BB）
 *
 * 目标：被米家 App 绑定后，按键事件通过加密 MiBeacon 广播上报 → 云端 →
 * 自动化编辑器里出现「单击 / 双击 / 长按」触发条件。
 *
 * 产品库实证（米家 APK assets/device_config/plugin_config_cn-zh_cn，
 * 另导出 E:/逆向工程/小米互通互联服务/03-实测证据/OFFICIAL-BLE-PID-FULL.csv）：
 *   pd_id=14523 (0x38BB) model=090615.remote.btsw1
 *   name=「PTX 无线开关（蓝牙版）」desc=「单按键无线开关，支持单击，双击，长按情景」
 *   bt_bind_style=2（BLE 直连） / resolve_type=3（广播可扫描识别）
 * ★与 akswr1(0x3910) 的区别：本型号多一个「双击」事件。
 * spec: urn:miot-spec-v2:device:remote-control:0000A021:090615-btsw1:1
 *   siid 2 = switch-sensor-for-ble，siid 3 = battery
 *   ★两个服务在 spec 里 properties 表为空 → 属性/事件对象号只能实测。
 *
 * ★四个自动化触发事件（用户实证）：单击 / 双击 / 长按 / 低电量。
 * 前三个是 MiBeacon 标准按键对象 0x4E0C/0x4E0D/0x4E0E；
 * 低电量按 MiBeacon2 公式算：eid = 0x4000|(3<<10)|(1001-1000) = 0x4C01。
 * 四个事件对象号都做成**运行时可改**（串口 obj 命令），配一次连接遍历：
 *   单击  s_obj_click  （默认 0x4E0C）
 *   双击  s_obj_double （默认 0x4E0D）
 *   长按  s_obj_hold   （默认 0x4E0E）
 *   低电量 s_obj_lowbat（默认 0x4C01，可盲测改）
 *   电量  s_obj_batt   （默认 0x100A 标准电量心跳）
 * 每个对象号配唯一可辨的值，改一次 → 触发一次 → 读云端，命中即定。
 *
 * GATT 层来自 carlight_adv（12 特征值与真机逐条一致 + 0x0019 登录后转 device
 * + pick_chan 来路面），sdkconfig 也带齐两处栈 8192 的修复。
 */
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "miot_ble/miot_auth.h"
#include "miot_ble/miot_device.h"
#include "miot_ble/miot_beacon.h"
#include "miot_ble/miot_props.h"
#include "miot_ble/miot_spec.h"

#include "gatt_miot.h"

void miot_port_esp32_crypto_init(void);

static const char *TAG = "PTXSW1";

/* 扫描响应里的名字。米家认型号靠 PID（帧第 3-4 字节），不靠名字；
 * 这里用产品库里的短名，便于串口/抓包时人工辨认。 */
#define PTX_PID        0x38BB
#define PTX_NAME       "PTX-BTSW1"

#define NVS_NS         "ptxbsw1"
#define NVS_KEY_ID     "ident"
#define NVS_KEY_FCNT   "fcnt"

/* ★MAC 必须每工程独立：米家按 MAC 去重，绑过的 MAC 不会再出现在「发现的设备」 */
#define USE_CUSTOM_MAC 1
#if USE_CUSTOM_MAC
#define BASE_MAC_B4    0x8B
#define BASE_MAC_B5    0x11
#endif

#define BOOT_PIN       9
#define ADV_PERIOD_MS  3000     /* 常态心跳 */
#define REPEAT_MS      2500     /* 事件帧重复广播时长（网关轮询周期内至少一帧） */
#define WAKE_WINDOW_MS 120000
#define ADV_CONN_ALWAYS 1
/* ★低电量提示：电量低于该阈值时，电量心跳额外按事件帧快广播补发，
 * 让网关/米家可靠收到低电量（普通 3s 心跳网关可能漏采）。小米生态的低电量
 * 提示走 0x100A 电量对象本身（网关按百分比判低电），无需单独事件对象。 */
#define LOW_BATT_THRESHOLD 15

/* 本地状态 */
static uint32_t s_fcnt;
static uint8_t  v_batt   = 87;
static uint32_t s_wake_until;

/* 待广播的按键事件（0=无事件，回到电量心跳） */
static uint16_t s_btn_obj;
static uint32_t s_btn_until;
static const char *s_btn_name = "IDLE";
/* 低电量快广播补发截止时刻：低于阈值时让 0x100A 低电量帧在网关轮询窗口内必被采到 */
static uint32_t s_lowbatt_until;

/* 可运行时覆盖的对象号（盲测用） */
static uint16_t s_obj_click  = MB_OBJ_BTN_CLICK;   /* 0x4E0C */
static uint16_t s_obj_double = MB_OBJ_BTN_DOUBLE;  /* 0x4E0D —— 本型号独有 */
static uint16_t s_obj_hold   = MB_OBJ_BTN_HOLD;    /* 0x4E0E */
static uint16_t s_obj_lowbat = 0x4C01;             /* 低电量事件 3.e.1001（公式算，待实测） */
static uint16_t s_obj_batt   = MB_OBJ_BATTERY;     /* 0x100A */

static miot_auth_t   s_auth;
static miot_device_t s_dev;
static miot_port_t   s_port;

static uint8_t  s_mac_air[6];
static uint8_t  s_frame[32];
static uint8_t  s_flen;
static int      s_logged_in;
static int      s_was_bound;
static bool     s_win_prev;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static bool win_active(void) { return s_wake_until && (int32_t)(s_wake_until - now_ms()) > 0; }
static bool low_batt_active(void) { return s_lowbatt_until && (int32_t)(s_lowbatt_until - now_ms()) > 0; }
static uint32_t port_time_now(void *user) { (void)user; return now_ms(); }
static int gap_event_cb(struct ble_gap_event *event, void *arg);

/* ---------------- 身份持久化 ---------------- */
static void id_save(void *user, const miot_auth_identity_t *id)
{
    (void)user;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    esp_err_t err = nvs_set_blob(h, NVS_KEY_ID, id, sizeof(*id));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "绑定身份已保存（did %02X %02X %02X %02X）%s",
             id->did[0], id->did[1], id->did[2], id->did[3],
             err == ESP_OK ? "" : " 写入失败");
}

static bool id_load(void *user, miot_auth_identity_t *id)
{
    (void)user;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = sizeof(*id);
    esp_err_t err = nvs_get_blob(h, NVS_KEY_ID, id, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof(*id)) return false;
    ESP_LOGI(TAG, "读到已绑定身份（did %02X %02X %02X %02X）",
             id->did[0], id->did[1], id->did[2], id->did[3]);
    return true;
}

static void fcnt_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) { s_fcnt = 1; return; }
    uint32_t v = 0;
    if (nvs_get_u32(h, NVS_KEY_FCNT, &v) != ESP_OK) v = 1;
    nvs_close(h);
    s_fcnt = v;
}

static void fcnt_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, NVS_KEY_FCNT, s_fcnt);
    nvs_commit(h);
    nvs_close(h);
}

/* ---------------- 日志与随机数 ---------------- */
static void port_log(void *user, int level, const char *tag, const char *fmt, ...)
{
    (void)user;
    va_list ap;
    va_start(ap, fmt);
    char buf[192];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (level == MIOT_LOG_ERROR)      ESP_LOGE(tag, "%s", buf);
    else if (level == MIOT_LOG_WARN)  ESP_LOGW(tag, "%s", buf);
    else                              ESP_LOGI(tag, "%s", buf);
}

static int port_random(void *user, uint8_t *out, size_t len)
{
    (void)user;
    esp_fill_random(out, len);
    return 0;
}

/* ---------------- spec 属性表（盲测期做宽应答） ----------------
 * 该型号 spec 的 properties 表为空，App 具体 GET 哪些 siid.piid 未知。
 * 若对未知项一律回 result=1（不认识），App 初始化可能判设备异常、后续
 * 控件不再发请求（carlight 的 8.2 string 就是这个坑）。
 * 所以这里对未知项统一回 uint8=1，已知电量项回真实电量，保证初始化走完。 */
static miot_prop_t n_props[] = {
    { 3, 1003, MIOT_TYPE_U8, MIOT_PROP_R, 1, &v_batt, 0, 100, NULL, "battery" },
};

static bool dev_on_get(void *user, uint8_t siid, uint16_t piid, miot_spec_value_t *out)
{
    (void)user;
    memset(out, 0, sizeof(*out));
    out->siid = siid;
    out->piid = piid;
    out->type = MIOT_TYPE_U8;
    out->result = 0;
    /* 电量服务的常见 piid 回真值，其余回 1（存在感应答，别回 ERR） */
    if (siid == 3 && (piid == 1003 || piid == 1004 || piid == 1005)) out->val.u8 = v_batt;
    else out->val.u8 = 1;
    return true;
}

static bool dev_on_set(void *user, uint8_t siid, uint16_t piid, const miot_spec_value_t *in)
{
    (void)user;
    ESP_LOGI(TAG, "<<< App 下发 set %u.%u type=%u", siid, piid, in->type);
    return false;      /* 返回 false → 应答里带 result，让 App 看到「不可写」而不是超时 */
}

/* ---------------- 广播 ----------------
 * 已绑定：有按键事件先发事件帧（网关按边沿判定，优先级最高），
 * 否则发电量心跳。 */
static int build_frame(void)
{
    miot_beacon_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.pid = PTX_PID;
    memcpy(cfg.mac_air, s_mac_air, 6);

    if (!s_auth.bound) {
        int fl = miot_beacon_frame_hello(&cfg, s_fcnt, 2 /* 后绑定 */, s_frame, sizeof(s_frame));
        if (fl < 0) { ESP_LOGE(TAG, "hello 帧失败 rc=%d", fl); return fl; }
        /* FC 高字节 bit9(solicited)：App 点卡片后等这一位才发起连接 */
        s_frame[1] = 0x2A;
        s_flen = (uint8_t)fl;
        return fl;
    }

    memcpy(cfg.beacon_key, s_auth.id.bind_key, 16);

    uint8_t objs[16];
    size_t n;
    if (s_btn_obj) {
        /* 事件载荷 1B：按键号，单键恒 1 */
        n = miot_beacon_obj_u8(objs, s_btn_obj, 1);
        ESP_LOGI(TAG, "事件帧 对象=0x%04X (%s) 载荷=1", s_btn_obj, s_btn_name);
    } else {
        n = miot_beacon_obj_u8(objs, s_obj_batt, v_batt);
    }

    int fl = miot_beacon_frame_v5_nomac(&cfg, s_fcnt, objs, n, s_frame, sizeof(s_frame));
    if (fl < 0) { ESP_LOGE(TAG, "加密帧失败 rc=%d", fl); return fl; }
    if (win_active()) s_frame[1] = 0x5B;
    s_flen = (uint8_t)fl;
    return fl;
}

static void adv_start(void)
{
    if (ble_gap_adv_active()) return;

    static const ble_uuid16_t uuid_fe95 = BLE_UUID16_INIT(0xFE95);

    uint8_t sd[32];
    int n = 0;
    sd[n++] = 0x95; sd[n++] = 0xFE;
    memcpy(sd + n, s_frame, s_flen);
    n += s_flen;

    struct ble_hs_adv_fields f;
    memset(&f, 0, sizeof(f));
    f.uuids16 = &uuid_fe95;
    f.num_uuids16 = 1;
    f.uuids16_is_complete = 1;
    f.svc_data_uuid16 = sd;
    f.svc_data_uuid16_len = (uint8_t)n;
    int rc = ble_gap_adv_set_fields(&f);
    if (rc != 0) { ESP_LOGE(TAG, "广播字段设置失败 rc=%d", rc); return; }

    /* V2 信标帧：0x20 = 未注册，绑定后 0x00 */
    static uint8_t mfr[16];
    int m = 0;
    mfr[m++] = 0x8F; mfr[m++] = 0x03;
    mfr[m++] = 0x30;
    mfr[m++] = (uint8_t)(s_auth.bound ? 0x00 : 0x20);
    mfr[m++] = (uint8_t)(PTX_PID & 0xFF);
    mfr[m++] = (uint8_t)(PTX_PID >> 8);
    mfr[m++] = (uint8_t)s_fcnt;
    for (int i = 0; i < 6; i++) mfr[m++] = s_mac_air[i];
    mfr[m++] = 0x08;

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = (const uint8_t *)PTX_NAME;
    rsp.name_len = sizeof(PTX_NAME) - 1;
    rsp.name_is_complete = 1;
    rsp.mfg_data = mfr;
    rsp.mfg_data_len = (uint8_t)m;
    if (ble_gap_adv_rsp_set_fields(&rsp) != 0) ESP_LOGW(TAG, "扫描响应设置失败");

    struct ble_gap_adv_params p;
    memset(&p, 0, sizeof(p));
#if ADV_CONN_ALWAYS
    p.conn_mode = BLE_GAP_CONN_MODE_UND;
#else
    p.conn_mode = win_active() ? BLE_GAP_CONN_MODE_UND : BLE_GAP_CONN_MODE_NON;
#endif
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    /* ★事件帧 / 低电量补发期间快广播：网关轮询窗口短，必须被扫到 */
    if (s_btn_obj || low_batt_active()) { p.itvl_min = 48; p.itvl_max = 80; }
    else { p.itvl_min = s_auth.bound ? 800 : 160; p.itvl_max = s_auth.bound ? 800 : 320; }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &p, gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) ESP_LOGE(TAG, "广播启动失败 rc=%d", rc);
}

static void adv_refresh(void)
{
    if (gatt_miot_is_connected()) return;
    if (ble_gap_adv_active()) ble_gap_adv_stop();
    adv_start();
}

/* 触发一次按键事件：单击/长按共用 */
static void fire_event(uint16_t obj, const char *name)
{
    s_btn_obj   = obj;
    s_btn_name  = name;
    s_btn_until = now_ms() + REPEAT_MS;
    s_wake_until = now_ms() + WAKE_WINDOW_MS;
    s_fcnt++;
    fcnt_save();
    if (build_frame() >= 0) adv_refresh();
    ESP_LOGW(TAG, ">>> 按键事件 0x%04X (%s)  重复 %dms", obj, name, REPEAT_MS);
}

/* ---------------- GAP 事件 ---------------- */
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            gatt_miot_on_connect(event->connect.conn_handle, BLE_ATT_MTU_DFLT);
            ble_gap_adv_stop();
            ESP_LOGI(TAG, "手机连上来了（广播已暂停）");
        } else {
            ESP_LOGW(TAG, "连接失败 status=%d", event->connect.status);
            adv_start();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        gatt_miot_on_disconnect();
        miot_auth_on_disconnect(&s_auth);
        miot_device_session_reset(&s_dev);
        s_logged_in = 0;
        if (s_auth.bound != s_was_bound) { s_was_bound = s_auth.bound; build_frame(); }
        adv_start();
        ESP_LOGI(TAG, "手机断开（reason=0x%02x），恢复广播", event->disconnect.reason);
        return 0;
    case BLE_GAP_EVENT_MTU:
        gatt_miot_on_mtu(event->mtu.value);
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        adv_start();
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "App 订阅 attr=%u notify=%d",
                 event->subscribe.attr_handle, event->subscribe.cur_notify);
        gatt_miot_on_subscribe(event->subscribe.attr_handle, event->subscribe.cur_notify);
        return 0;
    default:
        return 0;
    }
}

/* ---------------- 启动 ---------------- */
static void on_sync(void)
{
    uint8_t addr[6];
    if (ble_hs_util_ensure_addr(0) == 0 &&
        ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr, NULL) == 0) {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_BT);
        /* 帧里 MAC 必须是 LSB 序（nonce = mac||pid||cnt||counter_ext） */
        for (int i = 0; i < 6; i++) s_mac_air[i] = mac[5 - i];
    }
    ESP_LOGI(TAG, "协议栈就绪  MAC %02X:%02X:%02X:%02X:%02X:%02X  %s",
             s_mac_air[5], s_mac_air[4], s_mac_air[3], s_mac_air[2], s_mac_air[1], s_mac_air[0],
             s_auth.bound ? "已绑定" : "未绑定（等待米家添加）");
    build_frame();
    adv_start();
    gatt_miot_dump_handles();
    s_was_bound = s_auth.bound;
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ---------------- BOOT 键（GPIO9）----------------
 * 本工程里 BOOT 键就是被模拟的按键，本型号支持三种按法：
 *   单击 —— 按下后 <1s 松开，400ms 内无第二次按下 → 0x4E0C
 *   双击 —— 松开后 400ms 内再次按下 → 0x4E0D（btsw1 独有）
 *   长按 —— 按住 ≥1s → 0x4E0E
 *   按住 ≥5s —— 恢复出厂
 */
static void btn_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (gpio_get_level(BOOT_PIN) == 0) {
            int ms = 0;
            while (gpio_get_level(BOOT_PIN) == 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                ms += 100;
                if (ms >= 5000) {
                    ESP_LOGW(TAG, "BOOT 按住 5s → 擦身份重启");
                    nvs_flash_erase();
                    vTaskDelay(pdMS_TO_TICKS(200));
                    esp_restart();
                }
            }
            if (ms < 60) { vTaskDelay(pdMS_TO_TICKS(30)); continue; }

            if (ms >= 1000) {
                fire_event(s_obj_hold, "HOLD");
            } else {
                /* 短按：等 400ms 看有没有第二次按下（双击判定） */
                bool dbl = false;
                for (int t0 = 0; t0 < 400; t0 += 20) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                    if (gpio_get_level(BOOT_PIN) == 0) { dbl = true; break; }
                }
                if (dbl) {
                    while (gpio_get_level(BOOT_PIN) == 0) vTaskDelay(pdMS_TO_TICKS(30));
                    fire_event(s_obj_double, "DOUBLE");
                } else {
                    fire_event(s_obj_click, "CLICK");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/* ---------------- 串口 CLI ----------------
 * click / double / hold / lowbatt   触发单击 / 双击 / 长按 / 低电量
 * obj C D H L B               运行时改五个对象号（十六进制，盲测用）
 * batt N                  改电量值
 * info                    打印当前配置
 * factory                 擦身份重启
 */
static uint16_t hex16(const char *s)
{
    return (uint16_t)strtoul(s, NULL, 16);
}

static void cli_task(void *arg)
{
    (void)arg;
    char line[80];
    for (;;) {
        if (fgets(line, sizeof(line), stdin) == NULL) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        for (char *p = line; *p; p++) { if (*p == '\r' || *p == '\n') { *p = 0; break; } }
        if (!strncmp(line, "click", 5)) {
            fire_event(s_obj_click, "CLICK");
        } else if (!strncmp(line, "double", 6)) {
            fire_event(s_obj_double, "DOUBLE");
        } else if (!strncmp(line, "hold", 4)) {
            fire_event(s_obj_hold, "HOLD");
        } else if (!strncmp(line, "lowbatt", 7)) {
            /* 低电量事件：按当前对象号发事件帧（自动化「低电量」触发条件） */
            if (v_batt > LOW_BATT_THRESHOLD) {
                v_batt = LOW_BATT_THRESHOLD;   /* 事件载荷用低电值 */
            }
            fire_event(s_obj_lowbat, "LOWBATT");
        } else if (!strncmp(line, "obj", 3)) {
            char a[16] = {0}, b[16] = {0}, c[16] = {0}, e[16] = {0}, d[16] = {0};
            if (sscanf(line + 3, "%15s %15s %15s %15s %15s", a, b, c, e, d) >= 1) {
                if (a[0]) s_obj_click  = hex16(a);
                if (b[0]) s_obj_double = hex16(b);
                if (c[0]) s_obj_hold   = hex16(c);
                if (e[0]) s_obj_lowbat = hex16(e);
                if (d[0]) s_obj_batt   = hex16(d);
                ESP_LOGW(TAG, "对象号改为 click=0x%04X double=0x%04X hold=0x%04X "
                              "lowbat=0x%04X batt=0x%04X",
                         s_obj_click, s_obj_double, s_obj_hold, s_obj_lowbat, s_obj_batt);
                s_fcnt++;
                if (build_frame() >= 0) adv_refresh();
            }
        } else if (!strncmp(line, "batt", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) { int n = atoi(a); v_batt = (uint8_t)(n < 0 ? 0 : (n > 100 ? 100 : n));
                      s_fcnt++;
                      if (v_batt <= LOW_BATT_THRESHOLD) {
                          s_lowbatt_until = now_ms() + REPEAT_MS;
                          ESP_LOGW(TAG, ">>> 电量 %u ≤ %u：低电量提示快广播 %dms",
                                   v_batt, LOW_BATT_THRESHOLD, REPEAT_MS);
                          /* 低电量事件帧（自动化「低电量」触发条件） */
                          fire_event(s_obj_lowbat, "LOWBATT");
                      } else {
                          s_lowbatt_until = 0;
                      }
                      if (build_frame() >= 0) adv_refresh();
                      ESP_LOGW(TAG, "电量改为 %u", v_batt); }
        } else if (!strncmp(line, "info", 4)) {
            ESP_LOGI(TAG, "PID=0x%04X name=%s bound=%d logged_in=%d batt=%u "
                          "click=0x%04X double=0x%04X hold=0x%04X lowbat=0x%04X "
                          "battobj=0x%04X fcnt=%lu",
                     PTX_PID, PTX_NAME, s_auth.bound, s_logged_in, v_batt,
                     s_obj_click, s_obj_double, s_obj_hold, s_obj_lowbat,
                     s_obj_batt, (unsigned long)s_fcnt);
        } else if (!strncmp(line, "factory", 7)) {
            ESP_LOGW(TAG, "串口 factory → 擦身份重启");
            nvs_flash_erase();
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        }
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

#if USE_CUSTOM_MAC
    /* ★必须在任何 BLE / 协议栈初始化之前落定 MAC */
    { uint8_t base[6] = { 0x10, 0x00, 0x3B, 0xCE, BASE_MAC_B4, BASE_MAC_B5 }; esp_base_mac_addr_set(base); }
#endif

    gpio_config_t io = { .pin_bit_mask = 1ULL << BOOT_PIN, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&io);
    xTaskCreate(btn_task, "btn", 3072, NULL, 3, NULL);
    xTaskCreate(cli_task, "cli", 4096, NULL, 3, NULL);

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    for (int i = 0; i < 6; i++) s_mac_air[i] = mac[5 - i];

    miot_port_esp32_crypto_init();
    fcnt_load();

    ESP_LOGI(TAG, "===== PTX 无线开关（蓝牙版）（090615.remote.btsw1 / 0x%04X）=====", PTX_PID);
    ESP_LOGI(TAG, "用法：米家 App → + → 添加设备 → 点「PTX 无线开关（蓝牙版）」→ 绑定");
    ESP_LOGI(TAG, "      串口 click / double / hold / lowbatt / obj <C> <D> <H> <L> <B> / batt N / info / factory");

    memset(&s_port, 0, sizeof(s_port));
    s_port.send_notify = gatt_miot_notify_spec;
    s_port.time_now    = port_time_now;
    s_port.random      = port_random;
    s_port.log         = port_log;

    miot_device_init(&s_dev, &s_port);
    miot_device_attach(&s_dev, n_props, sizeof(n_props) / sizeof(n_props[0]),
                       dev_on_get, dev_on_set, NULL);
    ESP_LOGI(TAG, "spec table: %u props + on_get 通配应答",
             (unsigned)(sizeof(n_props) / sizeof(n_props[0])));

    miot_auth_port_t ap;
    memset(&ap, 0, sizeof(ap));
    ap.notify_0010 = gatt_miot_notify_auth_0010;
    ap.notify_0019 = gatt_miot_notify_auth_0019;
    ap.save        = id_save;
    ap.load        = id_load;
    ap.random      = port_random;
    miot_auth_init(&s_auth, &ap);

    uint32_t t0 = now_ms();
    miot_auth_prepare(&s_auth);
    ESP_LOGI(TAG, "ECDH 预生成 %lums", (unsigned long)(now_ms() - t0));

    if (nimble_port_init() != ESP_OK) { ESP_LOGE(TAG, "蓝牙协议栈初始化失败"); return; }
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = NULL;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(PTX_NAME);
    gatt_miot_register_svcs();
    gatt_miot_bind(&s_auth, &s_dev);
    gatt_miot_set_pid(PTX_PID);
    ble_att_set_preferred_mtu(247);

    nimble_port_freertos_init(host_task);

    uint32_t last_adv = 0;

    for (;;) {
        uint32_t t = now_ms();
        miot_auth_poll(&s_auth, t);

        if (s_auth.logged_in && !s_logged_in) {
            s_logged_in = 1;
            miot_device_set_session_keys(&s_dev, miot_auth_session_keys(&s_auth));
            ESP_LOGI(TAG, "*** 会话建立（spec 通道就绪）***");
            miot_device_send_version_reply(&s_dev, s_dev.tid++);
            {
                miot_spec_value_t v;
                memset(&v, 0, sizeof(v));
                v.siid = 3; v.piid = 1003; v.type = MIOT_TYPE_U8; v.val.u8 = v_batt;
                miot_device_send_props_changed(&s_dev, &v, 1);
                ESP_LOGI(TAG, "初值已推：3.1003 电量=%u", v_batt);
            }
        }

        if (s_auth.bound != s_was_bound) {
            s_was_bound = s_auth.bound;
            ESP_LOGI(TAG, "*** %s ***", s_was_bound ? "绑定成功，广播切到已绑定形态"
                                                     : "绑定状态被清除");
            build_frame();
            adv_refresh();
        }

        {
            bool win_now = win_active();
            if (s_win_prev && !win_now) { build_frame(); adv_refresh(); }
            s_win_prev = win_now;
        }

        /* 事件重复期结束 → 回到电量心跳 */
        if (s_btn_obj && (int32_t)(t - s_btn_until) >= 0) {
            ESP_LOGI(TAG, "事件结束（0x%04X %s）→ 回到电量帧", s_btn_obj, s_btn_name);
            s_btn_obj = 0;
            s_fcnt++;
            if (build_frame() >= 0) adv_refresh();
        }

        if (!gatt_miot_is_connected() && !s_btn_obj && t - last_adv >= ADV_PERIOD_MS) {
            last_adv = t;
            s_fcnt++;
            if ((s_fcnt % 10) == 0) fcnt_save();
            if (build_frame() >= 0) adv_refresh();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
