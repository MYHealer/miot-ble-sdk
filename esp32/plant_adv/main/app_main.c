/**
 * @file app_main.c
 * @brief 花花草草监测仪（hhcc.plantmonitor.v1 / PID 0x0098）—— 小白版最小固件
 *
 * ─────────────────────────────────────────────────────────────────────
 * ① 被米家 App 发现并绑定（0xFE95 + 标准认证，见 gatt_miot.c）
 * ② 串口改温湿度 / 光照 / 土壤 EC，短按轮换触发事件帧
 * ③ 空闲每 4 秒轮换播一帧（温 / 湿 / 光照 u24 / 土壤 EC），网关随时有最新值
 *
 * 长按 BOOT 5 秒 = 恢复出厂。
 *
 * ★老式标准对象（piid 均 <1000，不能用 eid 公式）：
 *   0x1004 温度 int16 0.1℃ → 云端 3:2
 *   0x1008 土壤湿度 uint8 % → 云端 2:1（UI「土壤湿度」；勿用 0x1006）
 *   0x1007 光照 uint24 lx（★必须 u24） → 云端 2:3
 *   0x1009 土壤 EC uint16 µS/cm → 云端 2:2
 * spec：2.1 Humidity/Soil Moisture / 2.2 Soil EC / 2.3 Illumination / 3.2 Temperature
 * ─────────────────────────────────────────────────────────────────────
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

#include "gatt_miot.h"

void miot_port_esp32_crypto_init(void);

static const char *TAG = "PLANT";

/* ★★★ 改参数只动这一块 ★★★ */
#define PLANT_PID        0x0098          /* 花花草草监测仪 HHCCJCY01 */
#define PLANT_NAME       "HHCCJCY01"     /* 扫描响应名字（米家主要按 PID 认）*/

#define NVS_NS          "plant"
#define NVS_KEY_ID      "ident"
#define NVS_KEY_FCNT    "fcnt"

/* ★派生 MAC：每工程独立后缀。plant = 0x81:0x03 → 空中 …:81:05 */
#define USE_CUSTOM_MAC  1
#if USE_CUSTOM_MAC
#define BASE_MAC_B4     0x81
#define BASE_MAC_B5     0x03
#endif

#define BOOT_PIN        9
#define ADV_PERIOD_MS   4000
#define EVENT_GAP_MS    400
#define EVENT_REPEAT    3
#define WAKE_WINDOW_MS  120000
#define ADV_CONN_ALWAYS 1
#define PLANT_SLOTS     4               /* 温 / 湿 / 光 / EC */

/* 标准对象号（见文件头；0x1007 是 u24）
 * 设备页四槽 = 土壤湿度 / 土壤肥力(EC) / 光照 / 空气温度。
 * 湿度槽走 0x1008 土壤湿度 uint8 %（不是空气湿度 0x1006——UI 无空气湿度项，
 * 发 0x1006 云端 2:1 恒 -704010000）。 */
static uint16_t s_obj_temp = 0x1004;
static uint16_t s_obj_hum  = 0x1008;
static uint16_t s_obj_lux  = 0x1007;
static uint16_t s_obj_ec   = 0x1009;
#define MB_OBJ_TEMP  s_obj_temp
#define MB_OBJ_HUM   s_obj_hum
#define MB_OBJ_LUX   s_obj_lux
#define MB_OBJ_EC    s_obj_ec

static uint32_t s_fcnt;
static int16_t  v_temp = 256;           /* 0.1℃：256 = 25.6℃ */
static uint8_t  v_hum  = 45;            /* 土壤湿度 %（0x1008）*/
static uint32_t v_lux  = 320;           /* lx */
static uint16_t v_ec   = 480;           /* µS/cm */
static uint8_t  s_ev_obj;               /* 事件槽：0温 1湿 2光 3EC */
static int      s_ev_left;
static uint32_t s_wake_until;

static miot_auth_t   s_auth;
static miot_device_t s_dev;
static miot_port_t   s_port;

static uint8_t  s_mac_air[6];
static uint8_t  s_frame[32];
static int      s_idx;
static uint8_t  s_flen;
static int      s_logged_in;
static int      s_was_bound;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static bool win_active(void)
{
    return s_wake_until && (int32_t)(s_wake_until - now_ms()) > 0;
}

static uint32_t port_time_now(void *user)
{
    (void)user;
    return now_ms();
}

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
    ESP_LOGI(TAG, "读到已保存的绑定身份（did %02X %02X %02X %02X）",
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

static size_t encode_slot(uint8_t *objs, uint8_t slot)
{
    switch (slot) {
    case 0:  return miot_beacon_obj_i16(objs, MB_OBJ_TEMP, v_temp);
    case 1:  return miot_beacon_obj_u8(objs, MB_OBJ_HUM,  v_hum);
    case 2:  return miot_beacon_obj_u24(objs, MB_OBJ_LUX,  v_lux);
    default: return miot_beacon_obj_u16(objs, MB_OBJ_EC,   v_ec);
    }
}

static const char *slot_name(uint8_t s)
{
    switch (s) {
    case 0: return "温度";
    case 1: return "湿度";
    case 2: return "光照";
    default: return "土壤EC";
    }
}

static int build_frame(int idx)
{
    miot_beacon_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.pid = PLANT_PID;
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
    uint8_t slot = (s_ev_left > 0) ? s_ev_obj : (uint8_t)(idx % PLANT_SLOTS);
    size_t n = encode_slot(objs, slot);

    int fl = miot_beacon_frame_v5_nomac(&cfg, s_fcnt, objs, n, s_frame, sizeof(s_frame));
    if (fl < 0) {
        ESP_LOGE(TAG, "加密帧失败 rc=%d", fl);
        return fl;
    }
    ESP_LOGI(TAG, "广播帧 槽=%u(%s) 帧长=%dB", slot, slot_name(slot), fl);
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
    if (rc != 0) {
        ESP_LOGE(TAG, "广播字段设置失败 rc=%d（帧 %dB）", rc, n);
        return;
    }

    static uint8_t mfr[16];
    int m = 0;
    mfr[m++] = 0x8F; mfr[m++] = 0x03;
    mfr[m++] = 0x30;
    mfr[m++] = (uint8_t)(s_auth.bound ? 0x00 : 0x20);
    mfr[m++] = (uint8_t)(PLANT_PID & 0xFF);
    mfr[m++] = (uint8_t)(PLANT_PID >> 8);
    mfr[m++] = (uint8_t)s_fcnt;
    for (int i = 0; i < 6; i++) mfr[m++] = s_mac_air[i];
    mfr[m++] = 0x08;

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = (const uint8_t *)PLANT_NAME;
    rsp.name_len = sizeof(PLANT_NAME) - 1;
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
    if (win_active()) {
        p.itvl_min = 48;
        p.itvl_max = 80;
    } else {
        p.itvl_min = s_auth.bound ? 800 : 160;
        p.itvl_max = s_auth.bound ? 800 : 320;
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &p,
                           gap_event_cb, NULL);
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
        if (s_auth.bound != s_was_bound) {
            s_was_bound = s_auth.bound;
            build_frame(s_idx);
        }
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
        return 0;

    default:
        return 0;
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
    ESP_LOGI(TAG, "协议栈就绪  MAC %02X:%02X:%02X:%02X:%02X:%02X  %s",
             s_mac_air[5], s_mac_air[4], s_mac_air[3],
             s_mac_air[2], s_mac_air[1], s_mac_air[0],
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

static void fire_slot(uint8_t slot, const char *src)
{
    s_ev_obj = slot;
    s_ev_left = EVENT_REPEAT;
    s_fcnt++;
    s_wake_until = now_ms() + WAKE_WINDOW_MS;
    ESP_LOGW(TAG, ">>> %s触发 %s（槽 %u）", src, slot_name(slot), slot);
    build_frame(s_idx);
    adv_refresh();
}

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
                /* 短按：轮换触发一个槽，方便验证 */
                static uint8_t rot;
                fire_slot(rot, "按键");
                rot = (uint8_t)((rot + 1) % PLANT_SLOTS);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/* 串口 CLI：
 *   temp N   设温度 0.1℃（如 256 = 25.6）
 *   hum N    设湿度 0.1%（如 552 = 55.2）
 *   lux N    设光照 lx
 *   ec N     设土壤 EC µS/cm
 *   fire 0|1|2|3   立刻发对应槽事件帧
 *   info / factory / objt|objh|objl|obje */
static void cli_task(void *arg)
{
    (void)arg;
    char line[64];
    for (;;) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        for (char *p = line; *p; p++) { if (*p == '\r' || *p == '\n') { *p = 0; break; } }

        if (!strncmp(line, "temp", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) { v_temp = (int16_t)atoi(a);
                      ESP_LOGW(TAG, ">>> 温度 = %.1f℃", v_temp / 10.0);
                      fire_slot(0, "串口"); }
        } else if (!strncmp(line, "hum", 3)) {
            const char *a = line + 3; while (*a == ' ') a++;
            if (*a) { int h = atoi(a); if (h < 0) h = 0; if (h > 100) h = 100;
                      v_hum = (uint8_t)h;
                      ESP_LOGW(TAG, ">>> 土壤湿度 = %u%%", v_hum);
                      fire_slot(1, "串口"); }
        } else if (!strncmp(line, "lux", 3)) {
            const char *a = line + 3; while (*a == ' ') a++;
            if (*a) { v_lux = (uint32_t)atoi(a);
                      ESP_LOGW(TAG, ">>> 光照 = %u lx", (unsigned)v_lux);
                      fire_slot(2, "串口"); }
        } else if (!strncmp(line, "ec", 2) && (line[2] == ' ' || line[2] == 0)) {
            const char *a = line + 2; while (*a == ' ') a++;
            if (*a) { v_ec = (uint16_t)atoi(a);
                      ESP_LOGW(TAG, ">>> 土壤EC = %u", v_ec);
                      fire_slot(3, "串口"); }
        } else if (!strncmp(line, "fire", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) fire_slot((uint8_t)atoi(a), "串口");
        } else if (!strncmp(line, "info", 4)) {
            ESP_LOGI(TAG, "PID=0x%04X MAC %02X:%02X:%02X:%02X:%02X:%02X bound=%d "
                     "temp=%.1f soil=%u%% lux=%u ec=%u",
                     PLANT_PID,
                     s_mac_air[5], s_mac_air[4], s_mac_air[3],
                     s_mac_air[2], s_mac_air[1], s_mac_air[0],
                     s_auth.bound, v_temp / 10.0, v_hum,
                     (unsigned)v_lux, v_ec);
        } else if (!strncmp(line, "objt", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) { s_obj_temp = (uint16_t)strtoul(a, NULL, 16);
                      ESP_LOGW(TAG, "temp 对象 -> 0x%04X", s_obj_temp); }
        } else if (!strncmp(line, "objh", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) { s_obj_hum = (uint16_t)strtoul(a, NULL, 16);
                      ESP_LOGW(TAG, "hum 对象 -> 0x%04X", s_obj_hum); }
        } else if (!strncmp(line, "objl", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) { s_obj_lux = (uint16_t)strtoul(a, NULL, 16);
                      ESP_LOGW(TAG, "lux 对象 -> 0x%04X", s_obj_lux); }
        } else if (!strncmp(line, "obje", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) { s_obj_ec = (uint16_t)strtoul(a, NULL, 16);
                      ESP_LOGW(TAG, "ec 对象 -> 0x%04X", s_obj_ec); }
        } else if (!strncmp(line, "factory", 7)) {
            ESP_LOGW(TAG, "串口 factory → 擦身份重启");
            nvs_flash_erase(); vTaskDelay(pdMS_TO_TICKS(200)); esp_restart();
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
    {
        uint8_t base[6] = { 0x10, 0x00, 0x3B, 0xCE, BASE_MAC_B4, BASE_MAC_B5 };
        esp_base_mac_addr_set(base);
    }
#endif

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BOOT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);
    xTaskCreate(btn_task, "btn", 3072, NULL, 3, NULL);
    xTaskCreate(cli_task, "cli", 3072, NULL, 3, NULL);

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    for (int i = 0; i < 6; i++) s_mac_air[i] = mac[5 - i];

    miot_port_esp32_crypto_init();
    fcnt_load();

    ESP_LOGI(TAG, "===== 花花草草监测仪 小白版（HHCCJCY01 / 0x%04X）=====", PLANT_PID);
    ESP_LOGI(TAG, "用法：米家 App → + → 添加设备 → 点 HHCCJCY01 → 绑定");
    ESP_LOGI(TAG, "      串口 temp/hum/lux/ec + fire   短按轮换   长按 5s 恢复出厂");

    memset(&s_port, 0, sizeof(s_port));
    s_port.send_notify = gatt_miot_notify_spec;
    s_port.time_now    = port_time_now;
    s_port.random      = port_random;
    s_port.log         = port_log;

    miot_device_init(&s_dev, &s_port);
    miot_device_attach(&s_dev, NULL, 0, NULL, NULL, NULL);

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

    if (nimble_port_init() != ESP_OK) {
        ESP_LOGE(TAG, "蓝牙协议栈初始化失败");
        return;
    }
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = NULL;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(PLANT_NAME);
    gatt_miot_register_svcs();
    gatt_miot_bind(&s_auth, &s_dev);
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
        }

        if (s_auth.bound != s_was_bound) {
            s_was_bound = s_auth.bound;
            ESP_LOGI(TAG, "*** %s ***", s_was_bound ? "绑定成功，广播切到已绑定形态"
                                                     : "绑定状态被清除");
            build_frame(s_idx);
            adv_refresh();
        }

        {
            static bool win_was;
            bool win_now = win_active();
            if (win_was && !win_now) {
                build_frame(s_idx);
                adv_refresh();
            }
            win_was = win_now;
        }

        if (!gatt_miot_is_connected()) {
            uint32_t period = s_ev_left > 0 ? EVENT_GAP_MS : ADV_PERIOD_MS;
            if (t - last_adv >= period) {
                last_adv = t;
                if (s_ev_left > 0) {
                    s_ev_left--;
                    s_fcnt++;
                    ESP_LOGI(TAG, "事件帧 %s（剩 %d 次）", slot_name(s_ev_obj), s_ev_left);
                } else {
                    s_idx = (s_idx + 1) % PLANT_SLOTS;
                    s_fcnt++;
                    if ((s_fcnt % 10) == 0) fcnt_save();
                }
                if (build_frame(s_idx) >= 0) adv_refresh();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
