/**
 * @file app_main.c
 * @brief 小米磁吸散热背夹（xiaomi.cooler.r02p / PID 0x52AA）—— 本地控制型
 *
 * 目标：被米家 App 绑定后设备页完整渲染，制冷模式/灯效/延时可下发并回推。
 *
 * 产品库实证：pd_id=21130 (0x52AA) model=xiaomi.cooler.r02p
 *   name=小米磁吸散热背夹 / resolve_type=3（广播可扫描识别）/ 米家自家
 * spec: urn:miot-spec-v2:device:cooler:0000A0DC:xiaomi-r02p:2
 *   siid3 cooler      : 3.1 mode u8(0智能/1极寒) / 3.2 extreme-cold-limit bool /
 *                       3.3 temperature i8(-30~80)
 *   siid4 cooler-light: 4.1 on bool / 4.2 mode u8(0彩虹/1呼吸/2流光/3明亮) /
 *                       4.3 color u32 / 4.4 flow-speed u8(0-2) /
 *                       4.5 color-combination u8(0-2) / 4.6-4.8 color1-3 u32
 *                       action 4.1 reset-color
 *   siid5 delay       : 5.1 delay bool / 5.2 delay-time u16(0-359)
 *   siid6 extra       : 6.1 highlights bool
 *
 * ★spec 全部标空 rw，但按电水壶/香氛机先例 App 当可写下发 → 全部注册 RW。
 * GATT/sdkconfig 来自 carlight_adv 完整修复版。
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

static const char *TAG = "COOLER";

#define COOLER_PID    0x52AA
#define COOLER_NAME   "MI-COOLER"

#define NVS_NS        "coolerr02p"
#define NVS_KEY_ID    "ident"
#define NVS_KEY_FCNT  "fcnt"

#define USE_CUSTOM_MAC 1
#if USE_CUSTOM_MAC
#define BASE_MAC_B4   0x02
#define BASE_MAC_B5   0x02
#endif

#define BOOT_PIN       9
#define ADV_PERIOD_MS  4000
#define EVENT_REPEAT   3
#define EVENT_GAP_MS   400
#define WAKE_WINDOW_MS 120000
#define ADV_CONN_ALWAYS 1

/* 本地状态 */
static uint32_t s_fcnt;
static uint8_t  v_cmode     = 0;       /* 3.1 0智能/1极寒 */
static uint8_t  v_climit    = 0;       /* 3.2 */
static int8_t   v_temp      = 25;      /* 3.3 int8 ℃ */
static uint8_t  v_light_on  = 1;       /* 4.1 */
static uint8_t  v_lmode     = 0;       /* 4.2 0彩虹/1呼吸/2流光/3明亮 */
static uint32_t v_color     = 0x00FF00;/* 4.3 */
static uint8_t  v_flowspd   = 1;       /* 4.4 0-2 */
static uint8_t  v_comb      = 0;       /* 4.5 0-2 */
static uint32_t v_color1    = 0xFF0000;/* 4.6 */
static uint32_t v_color2    = 0x00FF00;/* 4.7 */
static uint32_t v_color3    = 0x0000FF;/* 4.8 */
static uint8_t  v_delay     = 0;       /* 5.1 */
static uint16_t v_delaytime = 0;       /* 5.2 */
static uint8_t  v_highlight = 1;       /* 6.1 */

static volatile uint8_t  s_dirty_siid;
static volatile uint16_t s_dirty_piid;

static void cooler_on_set(const struct miot_prop *p)
{
    s_dirty_siid = p->siid;
    s_dirty_piid = p->piid;
    ESP_LOGI(TAG, "<<< App set %u.%u 已落值", p->siid, p->piid);
}

static miot_prop_t c_props[] = {
    { 3, 1, MIOT_TYPE_U8,   MIOT_PROP_RW, 1, &v_cmode,     0, 1,        cooler_on_set, "cooler-mode"   },
    { 3, 2, MIOT_TYPE_BOOL, MIOT_PROP_RW, 1, &v_climit,    0, 1,        cooler_on_set, "extreme-limit" },
    { 3, 3, MIOT_TYPE_I8,   MIOT_PROP_R,  1, &v_temp,     -30, 80,      NULL,          "temperature"   },
    { 4, 1, MIOT_TYPE_BOOL, MIOT_PROP_RW, 1, &v_light_on,  0, 1,        cooler_on_set, "light-on"      },
    { 4, 2, MIOT_TYPE_U8,   MIOT_PROP_RW, 1, &v_lmode,     0, 3,        cooler_on_set, "light-mode"    },
    { 4, 3, MIOT_TYPE_U32,  MIOT_PROP_RW, 4, &v_color,     0, 16777215, cooler_on_set, "color"         },
    { 4, 4, MIOT_TYPE_U8,   MIOT_PROP_RW, 1, &v_flowspd,   0, 2,        cooler_on_set, "flow-speed"    },
    { 4, 5, MIOT_TYPE_U8,   MIOT_PROP_RW, 1, &v_comb,      0, 2,        cooler_on_set, "color-comb"    },
    { 4, 6, MIOT_TYPE_U32,  MIOT_PROP_RW, 4, &v_color1,    0, 16777215, cooler_on_set, "color1"        },
    { 4, 7, MIOT_TYPE_U32,  MIOT_PROP_RW, 4, &v_color2,    0, 16777215, cooler_on_set, "color2"        },
    { 4, 8, MIOT_TYPE_U32,  MIOT_PROP_RW, 4, &v_color3,    0, 16777215, cooler_on_set, "color3"        },
    { 5, 1, MIOT_TYPE_BOOL, MIOT_PROP_RW, 1, &v_delay,     0, 1,        cooler_on_set, "delay"         },
    { 5, 2, MIOT_TYPE_U16,  MIOT_PROP_RW, 2, &v_delaytime, 0, 359,      cooler_on_set, "delay-time"    },
    { 6, 1, MIOT_TYPE_BOOL, MIOT_PROP_RW, 1, &v_highlight, 0, 1,        cooler_on_set, "highlights"    },
};

static miot_auth_t   s_auth;
static miot_device_t s_dev;
static miot_port_t   s_port;

static uint8_t  s_mac_air[6];
static uint8_t  s_frame[32];
static uint8_t  s_flen;
static int      s_logged_in;
static int      s_was_bound;
static uint32_t s_wake_until;
static int      s_ev_left;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static bool win_active(void) { return s_wake_until && (int32_t)(s_wake_until - now_ms()) > 0; }
static uint32_t port_time_now(void *user) { (void)user; return now_ms(); }
static int gap_event_cb(struct ble_gap_event *event, void *arg);

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

static void port_log(void *user, int level, const char *tag, const char *fmt, ...)
{
    (void)user;
    va_list ap; va_start(ap, fmt);
    char buf[192]; vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    if (level == MIOT_LOG_ERROR)      ESP_LOGE(tag, "%s", buf);
    else if (level == MIOT_LOG_WARN)  ESP_LOGW(tag, "%s", buf);
    else                              ESP_LOGI(tag, "%s", buf);
}
static int port_random(void *user, uint8_t *out, size_t len)
{
    (void)user; esp_fill_random(out, len); return 0;
}

/* 广播轮播：模式 / 灯开关 / 灯模式 / 温度 */
#define FRAME_SLOTS 4
static int s_idx;
static const char *const k_names[FRAME_SLOTS] = { "cool-mode", "light-on", "light-mode", "temp" };

static int build_frame(void)
{
    miot_beacon_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.pid = COOLER_PID;
    memcpy(cfg.mac_air, s_mac_air, 6);

    if (!s_auth.bound) {
        int fl = miot_beacon_frame_hello(&cfg, s_fcnt, 2, s_frame, sizeof(s_frame));
        if (fl < 0) { ESP_LOGE(TAG, "hello 帧失败 rc=%d", fl); return fl; }
        s_frame[1] = 0x2A; s_flen = (uint8_t)fl; return fl;
    }

    memcpy(cfg.beacon_key, s_auth.id.bind_key, 16);
    uint8_t objs[16];
    size_t n;
    switch (s_idx) {
    case 0:  n = miot_beacon_obj_u8 (objs, 0x4C01, v_cmode);    break;  /* 3.1 */
    case 1:  n = miot_beacon_obj_u8 (objs, 0x5001, v_light_on); break;  /* 4.1 */
    case 2:  n = miot_beacon_obj_u8 (objs, 0x5002, v_lmode);    break;  /* 4.2 */
    default: n = miot_beacon_obj_u8 (objs, 0x4C03, (uint8_t)v_temp); break;  /* 3.3 int8 走 1B 字节 */
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
    f.uuids16 = &uuid_fe95; f.num_uuids16 = 1; f.uuids16_is_complete = 1;
    f.svc_data_uuid16 = sd; f.svc_data_uuid16_len = (uint8_t)n;
    int rc = ble_gap_adv_set_fields(&f);
    if (rc != 0) { ESP_LOGE(TAG, "广播字段失败 rc=%d", rc); return; }

    static uint8_t mfr[16];
    int m = 0;
    mfr[m++] = 0x8F; mfr[m++] = 0x03; mfr[m++] = 0x30;
    mfr[m++] = (uint8_t)(s_auth.bound ? 0x00 : 0x20);
    mfr[m++] = (uint8_t)(COOLER_PID & 0xFF);
    mfr[m++] = (uint8_t)(COOLER_PID >> 8);
    mfr[m++] = (uint8_t)s_fcnt;
    for (int i = 0; i < 6; i++) mfr[m++] = s_mac_air[i];
    mfr[m++] = 0x08;

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = (const uint8_t *)COOLER_NAME; rsp.name_len = sizeof(COOLER_NAME) - 1;
    rsp.name_is_complete = 1;
    rsp.mfg_data = mfr; rsp.mfg_data_len = (uint8_t)m;
    if (ble_gap_adv_rsp_set_fields(&rsp) != 0) ESP_LOGW(TAG, "扫描响应设置失败");

    struct ble_gap_adv_params p;
    memset(&p, 0, sizeof(p));
#if ADV_CONN_ALWAYS
    p.conn_mode = BLE_GAP_CONN_MODE_UND;
#else
    p.conn_mode = win_active() ? BLE_GAP_CONN_MODE_UND : BLE_GAP_CONN_MODE_NON;
#endif
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    if (s_ev_left > 0) { p.itvl_min = 48; p.itvl_max = 80; }
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

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            gatt_miot_on_connect(event->connect.conn_handle, BLE_ATT_MTU_DFLT);
            ble_gap_adv_stop(); ESP_LOGI(TAG, "手机连上来了");
        } else { adv_start(); }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        gatt_miot_on_disconnect();
        miot_auth_on_disconnect(&s_auth);
        miot_device_session_reset(&s_dev);
        s_logged_in = 0;
        if (s_auth.bound != s_was_bound) { s_was_bound = s_auth.bound; build_frame(); }
        adv_start(); ESP_LOGI(TAG, "手机断开 reason=0x%02x", event->disconnect.reason);
        return 0;
    case BLE_GAP_EVENT_MTU: gatt_miot_on_mtu(event->mtu.value); return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE: adv_start(); return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "App 订阅 attr=%u notify=%d",
                 event->subscribe.attr_handle, event->subscribe.cur_notify);
        gatt_miot_on_subscribe(event->subscribe.attr_handle, event->subscribe.cur_notify);
        return 0;
    default: return 0;
    }
}

static void on_sync(void)
{
    uint8_t addr[6];
    if (ble_hs_util_ensure_addr(0) == 0 &&
        ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr, NULL) == 0) {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_BT);
        for (int i = 0; i < 6; i++) s_mac_air[i] = mac[5 - i];
    }
    ESP_LOGI(TAG, "协议栈就绪 MAC %02X:%02X:%02X:%02X:%02X:%02X %s",
             s_mac_air[5], s_mac_air[4], s_mac_air[3], s_mac_air[2], s_mac_air[1], s_mac_air[0],
             s_auth.bound ? "已绑定" : "未绑定");
    build_frame(); adv_start(); gatt_miot_dump_handles();
    s_was_bound = s_auth.bound;
}
static void host_task(void *param)
{
    (void)param; nimble_port_run(); nimble_port_freertos_deinit();
}

static void btn_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (gpio_get_level(BOOT_PIN) == 0) {
            int ms = 0;
            while (gpio_get_level(BOOT_PIN) == 0) {
                vTaskDelay(pdMS_TO_TICKS(100)); ms += 100;
                if (ms >= 5000) {
                    ESP_LOGW(TAG, "按住 5s → 恢复出厂");
                    nvs_flash_erase(); vTaskDelay(pdMS_TO_TICKS(200)); esp_restart();
                }
            }
            if (ms >= 60) {
                v_cmode = v_cmode ? 0 : 1;
                s_dirty_siid = 3; s_dirty_piid = 1;
                s_wake_until = now_ms() + WAKE_WINDOW_MS;
                s_ev_left = EVENT_REPEAT; s_fcnt++;
                build_frame(); adv_refresh();
                ESP_LOGW(TAG, ">>> 按键：制冷模式=%u", v_cmode);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

static void cli_task(void *arg)
{
    (void)arg;
    char line[64];
    for (;;) {
        if (fgets(line, sizeof(line), stdin) == NULL) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        for (char *p = line; *p; p++) { if (*p == '\r' || *p == '\n') { *p = 0; break; } }
        if (!strncmp(line, "mode", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) { int n = atoi(a); v_cmode = (uint8_t)(n < 0 ? 0 : (n > 1 ? 1 : n));
                      s_dirty_siid = 3; s_dirty_piid = 1;
                      s_ev_left = EVENT_REPEAT; s_fcnt++; build_frame(); adv_refresh();
                      ESP_LOGW(TAG, ">>> 串口 制冷模式=%u", v_cmode); }
        } else if (!strncmp(line, "temp", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) { int n = atoi(a); v_temp = (int8_t)(n < -30 ? -30 : (n > 80 ? 80 : n));
                      s_dirty_siid = 3; s_dirty_piid = 3;
                      s_ev_left = EVENT_REPEAT; s_fcnt++; build_frame(); adv_refresh();
                      ESP_LOGW(TAG, ">>> 串口 传感器温度=%d℃", v_temp); }
        } else if (!strncmp(line, "info", 4)) {
            ESP_LOGI(TAG, "PID=0x%04X bound=%d cmode=%u temp=%d light=%u lmode=%u color=0x%06X",
                     COOLER_PID, s_auth.bound, v_cmode, v_temp, v_light_on, v_lmode, v_color);
        } else if (!strncmp(line, "factory", 7)) {
            ESP_LOGW(TAG, "串口 factory → 擦身份重启");
            nvs_flash_erase(); vTaskDelay(pdMS_TO_TICKS(200)); esp_restart();
        }
    }
}

static void push_prop(uint8_t siid, uint16_t piid)
{
    if (!s_auth.logged_in || !gatt_miot_is_connected()) return;
    static miot_spec_value_t v;
    memset(&v, 0, sizeof(v));
    v.siid = siid; v.piid = piid;
    switch (siid) {
    case 3:
        if (piid == 1)      { v.type = MIOT_TYPE_U8;   v.val.u8  = v_cmode;  }
        else if (piid == 2) { v.type = MIOT_TYPE_BOOL; v.val.u8  = v_climit; }
        else if (piid == 3) { v.type = MIOT_TYPE_I8;   v.val.u8  = (uint8_t)v_temp; }
        else return;
        break;
    case 4:
        switch (piid) {
        case 1: v.type = MIOT_TYPE_BOOL; v.val.u8  = v_light_on; break;
        case 2: v.type = MIOT_TYPE_U8;   v.val.u8  = v_lmode;     break;
        case 3: v.type = MIOT_TYPE_U32;  v.val.u32 = v_color;     break;
        case 4: v.type = MIOT_TYPE_U8;   v.val.u8  = v_flowspd;   break;
        case 5: v.type = MIOT_TYPE_U8;   v.val.u8  = v_comb;      break;
        case 6: v.type = MIOT_TYPE_U32;  v.val.u32 = v_color1;    break;
        case 7: v.type = MIOT_TYPE_U32;  v.val.u32 = v_color2;    break;
        case 8: v.type = MIOT_TYPE_U32;  v.val.u32 = v_color3;    break;
        default: return;
        }
        break;
    case 5:
        if (piid == 1)      { v.type = MIOT_TYPE_BOOL; v.val.u8  = v_delay;     }
        else if (piid == 2) { v.type = MIOT_TYPE_U16;  v.val.u16 = v_delaytime; }
        else return;
        break;
    case 6:
        if (piid == 1) { v.type = MIOT_TYPE_BOOL; v.val.u8 = v_highlight; }
        else return;
        break;
    default: return;
    }
    miot_device_send_props_changed(&s_dev, &v, 1);
    ESP_LOGI(TAG, "properties_changed %u.%u", siid, piid);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }
#if USE_CUSTOM_MAC
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

    ESP_LOGI(TAG, "===== 小米磁吸散热背夹（xiaomi.cooler.r02p / 0x%04X）=====", COOLER_PID);
    ESP_LOGI(TAG, "串口 mode 0|1 / temp N / info / factory");

    memset(&s_port, 0, sizeof(s_port));
    s_port.send_notify = gatt_miot_notify_spec;
    s_port.time_now    = port_time_now;
    s_port.random      = port_random;
    s_port.log         = port_log;

    miot_device_init(&s_dev, &s_port);
    miot_device_attach(&s_dev, c_props, sizeof(c_props) / sizeof(c_props[0]), NULL, NULL, NULL);
    ESP_LOGI(TAG, "spec 属性 %u 条", (unsigned)(sizeof(c_props) / sizeof(c_props[0])));

    miot_auth_port_t ap;
    memset(&ap, 0, sizeof(ap));
    ap.notify_0010 = gatt_miot_notify_auth_0010;
    ap.notify_0019 = gatt_miot_notify_auth_0019;
    ap.save = id_save; ap.load = id_load; ap.random = port_random;
    miot_auth_init(&s_auth, &ap);

    uint32_t t0 = now_ms();
    miot_auth_prepare(&s_auth);
    ESP_LOGI(TAG, "ECDH 预生成 %lums", (unsigned long)(now_ms() - t0));

    if (nimble_port_init() != ESP_OK) { ESP_LOGE(TAG, "蓝牙初始化失败"); return; }
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    ble_hs_cfg.sync_cb = on_sync; ble_hs_cfg.reset_cb = NULL;
    ble_svc_gap_init(); ble_svc_gatt_init();
    ble_svc_gap_device_name_set(COOLER_NAME);
    gatt_miot_register_svcs();
    gatt_miot_bind(&s_auth, &s_dev);
    gatt_miot_set_pid(COOLER_PID);
    ble_att_set_preferred_mtu(247);

    nimble_port_freertos_init(host_task);

    uint32_t last_adv = 0;
    for (;;) {
        uint32_t t = now_ms();
        miot_auth_poll(&s_auth, t);

        if (s_auth.logged_in && !s_logged_in) {
            s_logged_in = 1;
            miot_device_set_session_keys(&s_dev, miot_auth_session_keys(&s_auth));
            ESP_LOGI(TAG, "*** 会话建立 ***");
            miot_device_send_version_reply(&s_dev, s_dev.tid++);
            /* 全属性初值 */
            static miot_spec_value_t v[14];
            memset(v, 0, sizeof(v));
            int i = 0;
            v[i].siid=3; v[i].piid=1; v[i].type=MIOT_TYPE_U8;   v[i++].val.u8=v_cmode;
            v[i].siid=3; v[i].piid=2; v[i].type=MIOT_TYPE_BOOL; v[i++].val.u8=v_climit;
            v[i].siid=3; v[i].piid=3; v[i].type=MIOT_TYPE_I8;   v[i++].val.u8=(uint8_t)v_temp;
            v[i].siid=4; v[i].piid=1; v[i].type=MIOT_TYPE_BOOL; v[i++].val.u8=v_light_on;
            v[i].siid=4; v[i].piid=2; v[i].type=MIOT_TYPE_U8;   v[i++].val.u8=v_lmode;
            v[i].siid=4; v[i].piid=3; v[i].type=MIOT_TYPE_U32;  v[i++].val.u32=v_color;
            v[i].siid=4; v[i].piid=4; v[i].type=MIOT_TYPE_U8;   v[i++].val.u8=v_flowspd;
            v[i].siid=4; v[i].piid=5; v[i].type=MIOT_TYPE_U8;   v[i++].val.u8=v_comb;
            v[i].siid=4; v[i].piid=6; v[i].type=MIOT_TYPE_U32;  v[i++].val.u32=v_color1;
            v[i].siid=4; v[i].piid=7; v[i].type=MIOT_TYPE_U32;  v[i++].val.u32=v_color2;
            v[i].siid=4; v[i].piid=8; v[i].type=MIOT_TYPE_U32;  v[i++].val.u32=v_color3;
            v[i].siid=5; v[i].piid=1; v[i].type=MIOT_TYPE_BOOL; v[i++].val.u8=v_delay;
            v[i].siid=5; v[i].piid=2; v[i].type=MIOT_TYPE_U16;  v[i++].val.u16=v_delaytime;
            v[i].siid=6; v[i].piid=1; v[i].type=MIOT_TYPE_BOOL; v[i++].val.u8=v_highlight;
            miot_device_send_props_changed(&s_dev, v, (size_t)i);
            ESP_LOGI(TAG, "initial props pushed (%d)", i);
        }

        {
            uint8_t siid = s_dirty_siid;
            uint16_t piid = s_dirty_piid;
            if (piid) { s_dirty_siid = 0; s_dirty_piid = 0; push_prop(siid, piid); }
        }

        if (s_auth.bound != s_was_bound) {
            s_was_bound = s_auth.bound;
            ESP_LOGI(TAG, "*** %s ***", s_was_bound ? "绑定成功" : "绑定清除");
            build_frame(); adv_refresh();
        }

        if (s_ev_left > 0 && (int32_t)(t - last_adv) >= EVENT_GAP_MS) {
            last_adv = t; s_ev_left--; s_fcnt++;
            if (build_frame() >= 0) adv_refresh();
        } else if (!gatt_miot_is_connected() && s_ev_left <= 0 && t - last_adv >= ADV_PERIOD_MS) {
            last_adv = t; s_fcnt++;
            if ((s_fcnt % 10) == 0) fcnt_save();
            s_idx = (s_idx + 1) % FRAME_SLOTS;
            if (build_frame() >= 0) adv_refresh();
            ESP_LOGI(TAG, "broadcast %s", k_names[s_idx]);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
