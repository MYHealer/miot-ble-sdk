/**
 * @file app_main.c
 * @brief 树色空气检测仪（shuse.airm.ss006 / PID 0x6651）—— 小白版最小固件
 *
 * ─────────────────────────────────────────────────────────────────────
 * ① 被米家 App 发现并绑定（0xFE95 + 标准认证，见 gatt_miot.c）
 * ② 串口改温度/湿度/AQI/TVOC/CO2/甲醛，短按轮换触发事件帧
 * ③ 空闲每 4 秒六槽轮播，网关随时有最新值
 *
 * 长按 BOOT 5 秒 = 恢复出厂。
 *
 * ★标准新式 spec（air-monitor），eid 公式：
 *   eid = 0x4000 | (siid<<10) | (piid-1000)
 *   温度 3.1001 float  → 0x4C01
 *   湿度 3.1008 float  → 0x4C08
 *   AQI  3.1028 uint16 → 0x4C1C
 *   TVOC 3.1031 float  → 0x4C1F
 *   CO2  3.1029 uint16 → 0x4C1D
 *   HCHO 3.1030 float  → 0x4C1E
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

static const char *TAG = "AIRM";

/* ★★★ 改参数只动这一块 ★★★ */
#define AIRM_PID         0x6651          /* 树色空气检测仪（BLE） */
#define AIRM_NAME        "SS006"         /* 扫描响应名字 */

#define NVS_NS          "airm"
#define NVS_KEY_ID      "ident"
#define NVS_KEY_FCNT    "fcnt"

/* ★派生 MAC：0x82:0x03 → 空中 …:82:05 */
#define USE_CUSTOM_MAC  1
#if USE_CUSTOM_MAC
#define BASE_MAC_B4     0x82
#define BASE_MAC_B5     0x03
#endif

#define BOOT_PIN        9
#define ADV_PERIOD_MS   4000
#define EVENT_GAP_MS    400
#define EVENT_REPEAT    3
/* 气体属性（HCHO/TVOC）网关落库延迟约 2 分钟：事件帧拉长到 ~72s，
 * 每 400ms 一帧连续补发，配合云端轮询才能稳定写入（2026-11-19 实测 129s）。 */
#define EVENT_REPEAT_GAS 180
/* 气体属性：单次连发不够，需跨 ~2 分钟多次补发（实测 129s 落库） */
#define GAS_REFIRE_MAX   5
#define GAS_REFIRE_MS    45000
#define WAKE_WINDOW_MS  120000
#define ADV_CONN_ALWAYS 1
#define AIRM_SLOTS      6

static uint16_t s_obj_temp = 0x4C01;
static uint16_t s_obj_hum  = 0x4C08;
static uint16_t s_obj_aqi  = 0x4C1C;
static uint16_t s_obj_tvoc = 0x4C1F;
static uint16_t s_obj_co2  = 0x4C1D;
static uint16_t s_obj_hcho = 0x4C1E;
#define MB_OBJ_TEMP  s_obj_temp
#define MB_OBJ_HUM   s_obj_hum
#define MB_OBJ_AQI   s_obj_aqi
#define MB_OBJ_TVOC  s_obj_tvoc
#define MB_OBJ_CO2   s_obj_co2
#define MB_OBJ_HCHO  s_obj_hcho

static uint32_t s_fcnt;
static float    v_temp = 25.6f;
static float    v_hum  = 55.0f;
static uint16_t v_aqi  = 42;
static float    v_tvoc = 0.12f;
static uint16_t v_co2  = 520;
static float    v_hcho = 0.03f;
static uint8_t  s_ev_obj;
static uint8_t  s_dirty_siid;
static uint16_t s_dirty_piid;
static int      s_ev_left;
static uint8_t  s_gas_mask;          /* 本轮待发：bit3=TVOC bit5=HCHO */
static uint8_t  s_gas_want;          /* 用户请求全集（补发时恢复用） */
static int      s_gas_rounds;
static uint32_t s_gas_refire_at;
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

/* GATT 属性表：App 直连 GET/SET；HCHO/TVOC 的 beacon 对象网关不认，
 * 必须靠会话建立后 properties_changed + 页面 GET 才能上云/显示。 */
static miot_prop_t s_props[] = {
    { 3, 1001, MIOT_TYPE_FLOAT, MIOT_PROP_R, 4, &v_temp, 0, 0, NULL, "temperature" },
    { 3, 1008, MIOT_TYPE_FLOAT, MIOT_PROP_R, 4, &v_hum,  0, 0, NULL, "humidity" },
    { 3, 1028, MIOT_TYPE_U16,   MIOT_PROP_R, 4, &v_aqi,  0, 500, NULL, "aqi" },
    { 3, 1031, MIOT_TYPE_FLOAT, MIOT_PROP_R, 4, &v_tvoc, 0, 0, NULL, "tvoc" },
    { 3, 1029, MIOT_TYPE_U16,   MIOT_PROP_R, 4, &v_co2,  0, 8000, NULL, "co2" },
    { 3, 1030, MIOT_TYPE_FLOAT, MIOT_PROP_R, 4, &v_hcho, 0, 0, NULL, "hcho" },
};

static void push_all_props(void)
{
    if (!s_auth.logged_in || !gatt_miot_is_connected()) return;
    static miot_spec_value_t v[6];
    memset(v, 0, sizeof(v));
    v[0].siid=3; v[0].piid=1001; v[0].type=MIOT_TYPE_FLOAT; v[0].val.f=v_temp;
    v[1].siid=3; v[1].piid=1008; v[1].type=MIOT_TYPE_FLOAT; v[1].val.f=v_hum;
    v[2].siid=3; v[2].piid=1028; v[2].type=MIOT_TYPE_U16;   v[2].val.u16=v_aqi;
    v[3].siid=3; v[3].piid=1031; v[3].type=MIOT_TYPE_FLOAT; v[3].val.f=v_tvoc;
    v[4].siid=3; v[4].piid=1029; v[4].type=MIOT_TYPE_U16;   v[4].val.u16=v_co2;
    v[5].siid=3; v[5].piid=1030; v[5].type=MIOT_TYPE_FLOAT; v[5].val.f=v_hcho;
    miot_device_send_props_changed(&s_dev, v, 6);
    ESP_LOGI(TAG, "push_all_props T=%.1f H=%.1f AQI=%u TVOC=%.2f CO2=%u HCHO=%.3f",
             v_temp, v_hum, v_aqi, v_tvoc, v_co2, v_hcho);
}

static void push_one_prop(uint8_t siid, uint16_t piid)
{
    if (!s_auth.logged_in || !gatt_miot_is_connected()) return;
    static miot_spec_value_t v;
    memset(&v, 0, sizeof(v));
    v.siid=siid; v.piid=piid;
    if (piid==1001) { v.type=MIOT_TYPE_FLOAT; v.val.f=v_temp; }
    else if (piid==1008) { v.type=MIOT_TYPE_FLOAT; v.val.f=v_hum; }
    else if (piid==1028) { v.type=MIOT_TYPE_U16; v.val.u16=v_aqi; }
    else if (piid==1031) { v.type=MIOT_TYPE_FLOAT; v.val.f=v_tvoc; }
    else if (piid==1029) { v.type=MIOT_TYPE_U16; v.val.u16=v_co2; }
    else { v.type=MIOT_TYPE_FLOAT; v.val.f=v_hcho; }
    miot_device_send_props_changed(&s_dev, &v, 1);
    ESP_LOGI(TAG, "properties_changed %u.%u", siid, piid);
}

static const char *slot_name(uint8_t s)
{
    switch (s) {
    case 0: return "温度";
    case 1: return "湿度";
    case 2: return "AQI";
    case 3: return "TVOC";
    case 4: return "CO2";
    default: return "甲醛";
    }
}

static size_t encode_slot(uint8_t *objs, uint8_t slot)
{
    switch (slot) {
    case 0:  return miot_beacon_obj_f32(objs, MB_OBJ_TEMP, v_temp);
    case 1:  return miot_beacon_obj_f32(objs, MB_OBJ_HUM,  v_hum);
    case 2:  return miot_beacon_obj_u16(objs, MB_OBJ_AQI,  v_aqi);
    case 3:  return miot_beacon_obj_f32(objs, MB_OBJ_TVOC, v_tvoc);
    case 4:  return miot_beacon_obj_u16(objs, MB_OBJ_CO2,  v_co2);
    default:
        if (MB_OBJ_HCHO == 0x1010)
            return miot_beacon_obj_u16(objs, MB_OBJ_HCHO, (uint16_t)(v_hcho * 100.0f + 0.5f));
        return miot_beacon_obj_f32(objs, MB_OBJ_HCHO, v_hcho);
    }
}

static int build_frame(int idx)
{
    miot_beacon_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.pid = AIRM_PID;
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
    uint8_t slot = (s_ev_left > 0) ? s_ev_obj : (uint8_t)(idx % AIRM_SLOTS);
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
    mfr[m++] = (uint8_t)(AIRM_PID & 0xFF);
    mfr[m++] = (uint8_t)(AIRM_PID >> 8);
    mfr[m++] = (uint8_t)s_fcnt;
    for (int i = 0; i < 6; i++) mfr[m++] = s_mac_air[i];
    mfr[m++] = 0x08;

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = (const uint8_t *)AIRM_NAME;
    rsp.name_len = sizeof(AIRM_NAME) - 1;
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
        if (s_dirty_piid) {
            uint16_t pid = s_dirty_piid;
            s_dirty_siid = 0; s_dirty_piid = 0;
            push_one_prop(3, pid);
        }

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
    s_fcnt++;
    if (slot == 3 || slot == 5) {
        /* 气体：顺序排队不打断——已在窗内只记 mask，当前槽跑满 180 帧后再发下一槽 */
        s_gas_mask |= (uint8_t)(1u << slot);
        s_gas_want |= (uint8_t)(1u << slot);
        s_gas_rounds = GAS_REFIRE_MAX;
        s_gas_refire_at = now_ms() + GAS_REFIRE_MS;
        if (s_ev_left > 0 && (s_ev_obj == 3 || s_ev_obj == 5)) {
            ESP_LOGW(TAG, ">>> %s触发 %s（排队 mask=0x%02X，当前 %s 继续）",
                     src, slot_name(slot), s_gas_mask, slot_name(s_ev_obj));
        } else {
            s_ev_obj = slot;
            s_ev_left = EVENT_REPEAT_GAS;
            /* 本槽占用 mask 位，发完再轮下一槽 */
            s_gas_mask &= (uint8_t)~(1u << slot);
            ESP_LOGW(TAG, ">>> %s触发 %s（槽 %u 长连发 want=0x%02X）",
                     src, slot_name(slot), slot, s_gas_want);
        }
    } else {
        s_ev_obj = slot;
        s_ev_left = EVENT_REPEAT;
        ESP_LOGW(TAG, ">>> %s触发 %s（槽 %u）", src, slot_name(slot), slot);
    }
    s_wake_until = now_ms() + WAKE_WINDOW_MS;
    /* GATT 双通路：连着 App 时立刻 properties_changed（HCHO/TVOC beacon 网关不认） */
    {
        static const uint16_t pids[6] = {1001,1008,1028,1031,1029,1030};
        s_dirty_siid = 3;
        s_dirty_piid = pids[slot % 6];
        push_one_prop(3, s_dirty_piid);
    }
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
                static uint8_t rot;
                fire_slot(rot, "按键");
                rot = (uint8_t)((rot + 1) % AIRM_SLOTS);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/* 串口：temp hum aqi tvoc co2 hcho fire info factory */
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
            if (*a) { v_temp = (float)atof(a);
                      ESP_LOGW(TAG, ">>> 温度 = %.1f", v_temp);
                      fire_slot(0, "串口"); }
        } else if (!strncmp(line, "hum", 3)) {
            const char *a = line + 3; while (*a == ' ') a++;
            if (*a) { v_hum = (float)atof(a);
                      ESP_LOGW(TAG, ">>> 湿度 = %.1f", v_hum);
                      fire_slot(1, "串口"); }
        } else if (!strncmp(line, "aqi", 3)) {
            const char *a = line + 3; while (*a == ' ') a++;
            if (*a) { v_aqi = (uint16_t)atoi(a);
                      ESP_LOGW(TAG, ">>> AQI = %u", v_aqi);
                      fire_slot(2, "串口"); }
        } else if (!strncmp(line, "tvoc", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) { v_tvoc = (float)atof(a);
                      ESP_LOGW(TAG, ">>> TVOC = %.3f", v_tvoc);
                      fire_slot(3, "串口"); }
        } else if (!strncmp(line, "co2", 3)) {
            const char *a = line + 3; while (*a == ' ') a++;
            if (*a) { v_co2 = (uint16_t)atoi(a);
                      ESP_LOGW(TAG, ">>> CO2 = %u", v_co2);
                      fire_slot(4, "串口"); }
        } else if (!strncmp(line, "hcho", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) { v_hcho = (float)atof(a);
                      ESP_LOGW(TAG, ">>> 甲醛 = %.3f", v_hcho);
                      fire_slot(5, "串口"); }
        } else if (!strncmp(line, "fire", 4)) {
            const char *a = line + 4; while (*a == ' ') a++;
            if (*a) fire_slot((uint8_t)atoi(a), "串口");
        } else if (!strncmp(line, "info", 4)) {
            ESP_LOGI(TAG, "PID=0x%04X MAC %02X:%02X:%02X:%02X:%02X:%02X bound=%d "
                     "T=%.1f H=%.1f AQI=%u TVOC=%.3f CO2=%u HCHO=%.3f",
                     AIRM_PID,
                     s_mac_air[5], s_mac_air[4], s_mac_air[3],
                     s_mac_air[2], s_mac_air[1], s_mac_air[0],
                     s_auth.bound, v_temp, v_hum, v_aqi, v_tvoc, v_co2, v_hcho);
        } else if (!strncmp(line, "objhcho", 7)) {
            const char *a = line + 7; while (*a == ' ') a++;
            if (*a) { s_obj_hcho = (uint16_t)strtoul(a, NULL, 16);
                      ESP_LOGW(TAG, "hcho 对象 -> 0x%04X", s_obj_hcho); }
            else ESP_LOGI(TAG, "hcho 对象 0x%04X", s_obj_hcho);
        } else if (!strncmp(line, "objtvoc", 7)) {
            const char *a = line + 7; while (*a == ' ') a++;
            if (*a) { s_obj_tvoc = (uint16_t)strtoul(a, NULL, 16);
                      ESP_LOGW(TAG, "tvoc 对象 -> 0x%04X", s_obj_tvoc); }
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

    ESP_LOGI(TAG, "===== 空气检测仪 小白版（SS006 / 0x%04X）=====", AIRM_PID);
    ESP_LOGI(TAG, "用法：米家 App → + → 添加设备 → 点 SS006/空气检测仪 → 绑定");
    ESP_LOGI(TAG, "      串口 temp/hum/aqi/tvoc/co2/hcho + fire   短按轮换   长按 5s 恢复出厂");

    memset(&s_port, 0, sizeof(s_port));
    s_port.send_notify = gatt_miot_notify_spec;
    s_port.time_now    = port_time_now;
    s_port.random      = port_random;
    s_port.log         = port_log;

    miot_device_init(&s_dev, &s_port);
    miot_device_attach(&s_dev, s_props, sizeof(s_props)/sizeof(s_props[0]),
                       NULL, NULL, NULL);

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
    ble_svc_gap_device_name_set(AIRM_NAME);
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
            push_all_props();
            /* App 往往在 session 后 ~300ms 才 SUBSCRIBE，连推三拍防错过初值窗口 */
            vTaskDelay(pdMS_TO_TICKS(300));
            push_all_props();
            vTaskDelay(pdMS_TO_TICKS(700));
            push_all_props();
            ESP_LOGI(TAG, "delayed push_all x3 done");
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

        /* 气体顺序补发：当前槽跑满后立刻发 mask 里下一槽；本轮发完等 45s 再整轮 */
        if (s_ev_left == 0 && (s_gas_mask || s_gas_want) && s_gas_rounds > 0) {
            if (s_gas_mask) {
                int b = 0;
                while (b < 8 && !(s_gas_mask & (1u << b))) b++;
                if (b < 8) {
                    s_ev_obj = (uint8_t)b;
                    s_gas_mask &= (uint8_t)~(1u << b);
                    s_ev_left = EVENT_REPEAT_GAS;
                    s_fcnt++;
                    ESP_LOGI(TAG, "气体接续 剩%d轮 mask=0x%02X 槽=%u",
                             s_gas_rounds, s_gas_mask, s_ev_obj);
                    build_frame(s_idx);
                    adv_refresh();
                }
            } else if ((int32_t)(now_ms() - s_gas_refire_at) >= 0) {
                /* 本轮 want 已发完：恢复 mask，进入下一轮 */
                s_gas_mask = s_gas_want;
                s_gas_rounds--;
                s_gas_refire_at = now_ms() + GAS_REFIRE_MS;
                int b = 0;
                while (b < 8 && !(s_gas_mask & (1u << b))) b++;
                if (b < 8 && s_gas_rounds > 0) {
                    s_ev_obj = (uint8_t)b;
                    s_gas_mask &= (uint8_t)~(1u << b);
                    s_ev_left = EVENT_REPEAT_GAS;
                    s_fcnt++;
                    ESP_LOGI(TAG, "气体补发 剩%d轮 want=0x%02X 槽=%u",
                             s_gas_rounds, s_gas_want, s_ev_obj);
                    build_frame(s_idx);
                    adv_refresh();
                } else if (s_gas_rounds <= 0) {
                    s_gas_mask = 0;
                    s_gas_want = 0;
                }
            }
        }

        if (!gatt_miot_is_connected()) {
            uint32_t period = s_ev_left > 0 ? EVENT_GAP_MS : ADV_PERIOD_MS;
            if (t - last_adv >= period) {
                last_adv = t;
                if (s_ev_left > 0) {
                    s_ev_left--;
                    s_fcnt++;
                    ESP_LOGI(TAG, "事件帧 %s（剩 %d 次）",
                             slot_name(s_ev_obj), s_ev_left);
                } else {
                    s_idx = (s_idx + 1) % AIRM_SLOTS;
                    s_fcnt++;
                    if ((s_fcnt % 10) == 0) fcnt_save();
                }
                if (build_frame(s_idx) >= 0) adv_refresh();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
