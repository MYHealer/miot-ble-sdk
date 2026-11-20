/**
 * @file app_main.c
 * @brief 小米汽车磁吸补光灯（careco.nlight.o654 / PID 0x6266）—— 本地控制类
 *
 * 目标：被米家 App 绑定后，**通过 BLE 本地控制**亮度/色温/开关/模式（朋友确认能调）。
 * spec（urn:miot-spec-v2:device:night-light:0000A0AB:careco-o654:1）：
 *   siid 5 battery : 5.1003 电量 u8 / 5.1 充电状态 enum(0充/1未充)
 *   siid 7 light   : 7.1 on bool / 7.2 brightness u8(1-100) /
 *                    7.3 color-temp u16(2700-5000) / 7.4 mode u8(0自定义/1前/2抽屉/3暖/4夜灯)
 *   siid 8 device-status : 8.1 mode u8(0关/1自动/2开)
 *   siid 9 log : 9.1 time u32
 * 全部属性设 RW，App 才能下发 set；on_set 记脏号 → 主循环回推闭环。
 *
 * 由 alarm_adv 改造（gatt_miot.c 已是 12 特征 + 0x0019 登录后转 device 的完整版）。
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

static const char *TAG = "CARLIGHT";

#define CARLIGHT_PID   0x6266
#define CARLIGHT_NAME  "CAR-LIGHT"

#define NVS_NS          "carlight"
#define NVS_KEY_ID      "ident"
#define NVS_KEY_FCNT    "fcnt"

#define USE_CUSTOM_MAC  1
#if USE_CUSTOM_MAC
#define BASE_MAC_B4     0x87
#define BASE_MAC_B5     0x03
#endif

#define BOOT_PIN        9
#define ADV_PERIOD_MS   4000
#define EVENT_GAP_MS    400
#define EVENT_REPEAT    3
#define WAKE_WINDOW_MS  120000
#define ADV_CONN_ALWAYS 1

/* 本地状态 */
static uint32_t s_fcnt;
static uint8_t  v_on        = 0;      /* 7.1 */
static uint8_t  v_bright    = 50;     /* 7.2 1-100 */
static uint16_t v_ct        = 3900;   /* 7.3 2700-5000 */
static uint8_t  v_light_mode= 0;      /* 7.4 0-4 */
static uint8_t  v_batt      = 88;     /* 5.1003 */
static uint8_t  v_charge    = 1;      /* 5.1 0充/1未充 */
static uint8_t  v_dev_mode  = 2;      /* 8.1 0关/1自动/2开 */
static uint32_t v_time      = 0;      /* 9.1 */

static volatile uint8_t  s_dirty_siid;
static volatile uint16_t s_dirty_piid;

/* 场景预设（从 careco RN 插件 bundle 实证 lightDefaultSceneData）：
 *   mode1 明亮 bri100/cct5000 · mode2 月光 bri1/cct5000
 *   mode3 温馨 bri100/cct2700 · mode4 烛光 bri1/cct2700
 *   mode0 自定义 → 不改（用户手动调亮度/色温）。
 * ★真机切场景时硬件会同时把亮度/色温调到该场景值；App 点场景只发 7.4，
 *   随后重新 GET 7.2/7.3 同步 UI。若设备不联动改，App 读到旧值 → UI 归位
 *   （用户实测「切场景后 1s 归位」）。 */
static const struct { uint8_t mode; uint8_t bri; uint16_t cct; } s_scenes[] = {
    { 1, 100, 5000 }, { 2, 1, 5000 }, { 3, 100, 2700 }, { 4, 1, 2700 },
};
static volatile uint8_t s_scene_sync;   /* 收到场景 SET 后需追加回推 7.2/7.3 */

static void light_on_set(const struct miot_prop *p)
{
    s_dirty_siid = p->siid;
    s_dirty_piid = p->piid;
    /* 切场景(mode)时联动改亮度/色温到该场景预设值（v_light_mode 已落新值） */
    if (p->siid == 7 && p->piid == 4) {
        for (size_t i = 0; i < sizeof(s_scenes) / sizeof(s_scenes[0]); i++) {
            if (s_scenes[i].mode == v_light_mode) {
                v_bright = s_scenes[i].bri;
                v_ct     = s_scenes[i].cct;
                s_scene_sync = 1;
                break;
            }
        }
    }
    ESP_LOGI(TAG, "<<< App 下发 set：%d.%d 已落值", p->siid, p->piid);
}

/* ★8.2 device-status.test 是 spec 里的 string 属性，属性表用 int32 存储表达不了。
 * App 初始化会 GET 8.2，若回 ERR 它就判设备异常、后续所有控件（开关/亮度/色温）
 * 点了都不发 SET。用 on_get 回一个空串即可让初始化流程走完。 */
static bool dev_on_get(void *user, uint8_t siid, uint16_t piid, miot_spec_value_t *out)
{
    (void)user;
    if (siid == 8 && piid == 2) {
        memset(out, 0, sizeof(*out));
        out->siid = siid; out->piid = piid;
        out->type = MIOT_TYPE_STRING;
        out->result = 0;
        out->val.str[0] = 0;           /* 空串 */
        return true;
    }
    return false;
}

static miot_prop_t n_props[] = {
    { 5, 1003, MIOT_TYPE_U8,  MIOT_PROP_R,  1, &v_batt,       0, 100,  NULL,         "battery"      },
    { 5, 1,    MIOT_TYPE_U8,  MIOT_PROP_R,  1, &v_charge,     0, 1,    NULL,         "charging"     },
    { 7, 1,    MIOT_TYPE_BOOL,MIOT_PROP_RW, 1, &v_on,         0, 1,    light_on_set, "on"           },
    { 7, 2,    MIOT_TYPE_U8,  MIOT_PROP_RW, 1, &v_bright,     1, 100,  light_on_set, "brightness"   },
    { 7, 3,    MIOT_TYPE_U16, MIOT_PROP_RW, 2, &v_ct,         2700, 5000, light_on_set, "color-temp" },
    { 7, 4,    MIOT_TYPE_U8,  MIOT_PROP_RW, 1, &v_light_mode, 0, 4,    light_on_set, "light-mode"   },
    { 8, 1,    MIOT_TYPE_U8,  MIOT_PROP_RW, 1, &v_dev_mode,   0, 2,    light_on_set, "device-mode"  },
    { 9, 1,    MIOT_TYPE_U32, MIOT_PROP_RW, 4, &v_time,       0, 0,    light_on_set, "time"         },
};

static miot_auth_t   s_auth;
static miot_device_t s_dev;
static miot_port_t   s_port;

static uint8_t  s_mac_air[6];
static uint8_t  s_frame[32];
static int      s_idx;
static uint8_t  s_flen;
static int      s_logged_in;
static int      s_was_bound;
static uint32_t s_wake_until;
static int      s_ev_left;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static bool win_active(void) { return s_wake_until && (int32_t)(s_wake_until - now_ms()) > 0; }
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
    ESP_LOGI(TAG, "绑定身份已保存（did %02X %02X %02X %02X）",
             id->did[0], id->did[1], id->did[2], id->did[3]);
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

/* ---------------- 广播帧 ----------------
 * 已绑定：加密帧装 light.on（7.1 → 0x0701）让网关/云端同步开关态。 */
static uint16_t s_obj_on = 0x0701;

static int build_frame(int idx)
{
    (void)idx;
    miot_beacon_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.pid = CARLIGHT_PID;
    memcpy(cfg.mac_air, s_mac_air, 6);

    if (!s_auth.bound) {
        int fl = miot_beacon_frame_hello(&cfg, s_fcnt, 2, s_frame, sizeof(s_frame));
        if (fl < 0) { ESP_LOGE(TAG, "hello 帧失败 rc=%d", fl); return fl; }
        s_frame[1] = 0x2A;
        s_flen = (uint8_t)fl;
        return fl;
    }

    memcpy(cfg.beacon_key, s_auth.id.bind_key, 16);
    uint8_t objs[16];
    size_t n = miot_beacon_obj_u8(objs, s_obj_on, v_on);

    int fl = miot_beacon_frame_v5_nomac(&cfg, s_fcnt, objs, n, s_frame, sizeof(s_frame));
    if (fl < 0) { ESP_LOGE(TAG, "加密帧失败 rc=%d", fl); return fl; }
    ESP_LOGI(TAG, "广播帧 对象=0x%04X on=%u 帧长=%dB", s_obj_on, v_on, fl);
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

    static uint8_t mfr[16];
    int m = 0;
    mfr[m++] = 0x8F; mfr[m++] = 0x03;
    mfr[m++] = 0x30;
    mfr[m++] = (uint8_t)(s_auth.bound ? 0x00 : 0x20);
    mfr[m++] = (uint8_t)(CARLIGHT_PID & 0xFF);
    mfr[m++] = (uint8_t)(CARLIGHT_PID >> 8);
    mfr[m++] = (uint8_t)s_fcnt;
    for (int i = 0; i < 6; i++) mfr[m++] = s_mac_air[i];
    mfr[m++] = 0x08;

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = (const uint8_t *)CARLIGHT_NAME;
    rsp.name_len = sizeof(CARLIGHT_NAME) - 1;
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
    if (win_active()) { p.itvl_min = 48; p.itvl_max = 80; }
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
        if (s_auth.bound != s_was_bound) { s_was_bound = s_auth.bound; build_frame(s_idx); }
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
        ESP_LOGI(TAG, "App 订阅 attr=%u notify=%d", event->subscribe.attr_handle, event->subscribe.cur_notify);
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
    if (ble_hs_util_ensure_addr(0) == 0 && ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr, NULL) == 0) {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_BT);
        for (int i = 0; i < 6; i++) s_mac_air[i] = mac[5 - i];
    }
    ESP_LOGI(TAG, "协议栈就绪  MAC %02X:%02X:%02X:%02X:%02X:%02X  %s",
             s_mac_air[5], s_mac_air[4], s_mac_air[3], s_mac_air[2], s_mac_air[1], s_mac_air[0],
             s_auth.bound ? "已绑定" : "未绑定（等待米家添加）");
    s_idx = 0;
    build_frame(s_idx);
    adv_start();
    s_was_bound = s_auth.bound;
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ---------------- 按键 ---------------- */
static void btn_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (gpio_get_level(BOOT_PIN) == 0) {
            int ms = 0;
            while (gpio_get_level(BOOT_PIN) == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
                ms += 50;
                if (ms >= 5000) {
                    ESP_LOGW(TAG, "长按 5 秒 → 恢复出厂");
                    nvs_flash_erase();
                    vTaskDelay(pdMS_TO_TICKS(200));
                    esp_restart();
                }
            }
            if (ms >= 60) {
                v_on = v_on ? 0 : 1;
                s_dirty_siid = 7; s_dirty_piid = 1;
                s_wake_until = now_ms() + WAKE_WINDOW_MS;
                s_ev_left = EVENT_REPEAT; s_fcnt++;
                build_frame(s_idx); adv_refresh();
                ESP_LOGW(TAG, ">>> 按键：灯 on = %u", v_on);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/* 串口 CLI：on 0|1 / bright N / ct N / mode N / info / factory */
static void cli_task(void *arg)
{
    (void)arg;
    char line[64];
    for (;;) {
        if (fgets(line, sizeof(line), stdin) == NULL) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        for (char *p = line; *p; p++) { if (*p == '\r' || *p == '\n') { *p = 0; break; } }
        if (!strncmp(line, "on", 2)) {
            const char *a = line + 2; while (*a == ' ') a++;
            if (*a) { v_on = atoi(a) ? 1 : 0; s_dirty_siid = 7; s_dirty_piid = 1;
                      s_ev_left = EVENT_REPEAT; s_fcnt++; s_wake_until = now_ms() + WAKE_WINDOW_MS;
                      build_frame(s_idx); adv_refresh();
                      ESP_LOGW(TAG, ">>> 串口 on = %u", v_on); }
        } else if (!strncmp(line, "bright", 6)) {
            const char *a = line + 6; while (*a == ' ') a++;
            if (*a) { int n = atoi(a); v_bright = (uint8_t)(n<1?1:(n>100?100:n));
                      s_dirty_siid = 7; s_dirty_piid = 2;
                      ESP_LOGW(TAG, ">>> 串口 bright = %u", v_bright); }
        } else if (!strncmp(line, "ct", 2)) {
            const char *a = line + 2; while (*a == ' ') a++;
            if (*a) { int n = atoi(a); v_ct = (uint16_t)(n<2700?2700:(n>5000?5000:n));
                      s_dirty_siid = 7; s_dirty_piid = 3;
                      ESP_LOGW(TAG, ">>> 串口 ct = %u", v_ct); }
        } else if (!strncmp(line, "mode", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) { int n = atoi(a); v_light_mode = (uint8_t)(n<0?0:(n>4?4:n));
                      s_dirty_siid = 7; s_dirty_piid = 4;
                      ESP_LOGW(TAG, ">>> 串口 mode = %u", v_light_mode); }
        } else if (!strncmp(line, "info", 4)) {
            ESP_LOGI(TAG, "PID=0x%04X bound=%d on=%u bright=%u ct=%u mode=%u batt=%u",
                     CARLIGHT_PID, s_auth.bound, v_on, v_bright, v_ct, v_light_mode, v_batt);
        } else if (!strncmp(line, "factory", 7)) {
            ESP_LOGW(TAG, "串口 factory → 擦身份重启");
            nvs_flash_erase(); vTaskDelay(pdMS_TO_TICKS(200)); esp_restart();
        }
    }
}

/** 回推某条属性给 App */
static void push_prop(uint8_t siid, uint16_t piid)
{
    if (!s_auth.logged_in || !gatt_miot_is_connected()) return;
    static miot_spec_value_t v;
    memset(&v, 0, sizeof(v));
    v.siid = siid; v.piid = piid;
    if (siid == 7 && piid == 1)  { v.type = MIOT_TYPE_BOOL; v.val.u8  = v_on;        }
    else if (siid == 7 && piid == 2) { v.type = MIOT_TYPE_U8;  v.val.u8  = v_bright;   }
    else if (siid == 7 && piid == 3) { v.type = MIOT_TYPE_U16; v.val.u16 = v_ct;       }
    else if (siid == 7 && piid == 4) { v.type = MIOT_TYPE_U8;  v.val.u8  = v_light_mode;}
    else if (siid == 8 && piid == 1) { v.type = MIOT_TYPE_U8;  v.val.u8  = v_dev_mode; }
    else { v.type = MIOT_TYPE_U8; v.val.u8 = 0; }
    miot_device_send_props_changed(&s_dev, &v, 1);
    ESP_LOGI(TAG, "properties_changed %u.%u", siid, piid);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

#if USE_CUSTOM_MAC
    { uint8_t base[6] = { 0x10, 0x00, 0x3B, 0xCE, BASE_MAC_B4, BASE_MAC_B5 }; esp_base_mac_addr_set(base); }
#endif

    gpio_config_t io = { .pin_bit_mask = 1ULL << BOOT_PIN, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&io);
    xTaskCreate(btn_task, "btn", 3072, NULL, 3, NULL);
    xTaskCreate(cli_task, "cli", 4096, NULL, 3, NULL);

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    for (int i = 0; i < 6; i++) s_mac_air[i] = mac[5 - i];

    miot_port_esp32_crypto_init();
    fcnt_load();

    ESP_LOGI(TAG, "===== 小米汽车磁吸补光灯（CARLIGHT / 0x%04X）=====", CARLIGHT_PID);
    ESP_LOGI(TAG, "用法：米家 App → + → 添加设备 → 点补光灯 → 绑定");
    ESP_LOGI(TAG, "      串口 on 0|1 / bright N / ct N / mode N / info / factory");

    memset(&s_port, 0, sizeof(s_port));
    s_port.send_notify = gatt_miot_notify_spec;
    s_port.time_now    = port_time_now;
    s_port.random      = port_random;
    s_port.log         = port_log;

    miot_device_init(&s_dev, &s_port);
    miot_device_attach(&s_dev, n_props, sizeof(n_props) / sizeof(n_props[0]), dev_on_get, NULL, NULL);
    ESP_LOGI(TAG, "spec table: %u props", (unsigned)(sizeof(n_props) / sizeof(n_props[0])));

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
    ble_svc_gap_device_name_set(CARLIGHT_NAME);
    gatt_miot_register_svcs();
    gatt_miot_bind(&s_auth, &s_dev);
    gatt_miot_set_pid(CARLIGHT_PID);
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
            /* 主动发 F0 版本交换帧（优化模型必需），再推全属性初值。 */
            miot_device_send_version_reply(&s_dev, s_dev.tid++);
            ESP_LOGI(TAG, "spec version exchange frame sent (op=0xF0)");
            {
                static miot_spec_value_t v[8];
                memset(v, 0, sizeof(v));
                v[0].siid=5; v[0].piid=1003; v[0].type=MIOT_TYPE_U8;   v[0].val.u8  = v_batt;
                v[1].siid=5; v[1].piid=1;    v[1].type=MIOT_TYPE_U8;   v[1].val.u8  = v_charge;
                v[2].siid=7; v[2].piid=1;    v[2].type=MIOT_TYPE_BOOL; v[2].val.u8  = v_on;
                v[3].siid=7; v[3].piid=2;    v[3].type=MIOT_TYPE_U8;   v[3].val.u8  = v_bright;
                v[4].siid=7; v[4].piid=3;    v[4].type=MIOT_TYPE_U16;  v[4].val.u16 = v_ct;
                v[5].siid=7; v[5].piid=4;    v[5].type=MIOT_TYPE_U8;   v[5].val.u8  = v_light_mode;
                v[6].siid=8; v[6].piid=1;    v[6].type=MIOT_TYPE_U8;   v[6].val.u8  = v_dev_mode;
                v[7].siid=9; v[7].piid=1;    v[7].type=MIOT_TYPE_U32;  v[7].val.u32 = v_time;
                miot_device_send_props_changed(&s_dev, v, 8);
                ESP_LOGI(TAG, "initial props pushed (all 8 props)");
            }
        }

        {
            uint8_t siid = s_dirty_siid;
            uint16_t piid = s_dirty_piid;
            if (piid) { s_dirty_siid = 0; s_dirty_piid = 0; push_prop(siid, piid); }
        }

        /* 切场景联动：把改后的亮度/色温也回推给 App，否则 App 重新 GET
         * 读到旧值会把滑块归位（用户实测「切场景后 1s 归位」）。 */
        if (s_scene_sync) {
            s_scene_sync = 0;
            push_prop(7, 2);
            push_prop(7, 3);
            ESP_LOGI(TAG, "场景联动回推 bri=%u cct=%u", v_bright, v_ct);
        }

        if (s_auth.bound != s_was_bound) {
            s_was_bound = s_auth.bound;
            ESP_LOGI(TAG, "*** %s ***", s_was_bound ? "绑定成功，广播切到已绑定形态" : "绑定状态被清除");
            build_frame(s_idx);
            adv_refresh();
        }

        {
            static bool win_was;
            bool win_now = win_active();
            if (win_was && !win_now) { build_frame(s_idx); adv_refresh(); }
            win_was = win_now;
        }

        if (!gatt_miot_is_connected()) {
            uint32_t period = s_ev_left > 0 ? EVENT_GAP_MS : ADV_PERIOD_MS;
            if (t - last_adv >= period) {
                last_adv = t;
                if (s_ev_left > 0) { s_ev_left--; s_fcnt++; }
                else { s_idx = 0; s_fcnt++; if ((s_fcnt % 10) == 0) fcnt_save(); }
                if (build_frame(s_idx) >= 0) adv_refresh();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
