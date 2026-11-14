/**
 * @file app_main.c
 * @brief 米家 LED 智能灯泡 蓝牙 Mesh 版（MJDP003）—— 可绑定 / 可调光调色（ESP32-C3）
 *
 * 为什么选这台做「新一点的 BLE 灯」：
 *   夜灯 3（MJYD06YL）与夜灯 4（2026-09 众筹）都没有蓝牙，官方明说「不支持
 *   蓝牙连接、不支持米家 APP」，只能纯本地。而 2026-08-31 上架的这支 LED 灯泡
 *   蓝牙 Mesh 版（29.9 元）官方写明「支持手机蓝牙直连，无需蓝牙网关即可在
 *   米家 App 直接控制」—— 也就是说它有一条真正的 GATT 通道，能被我们模拟。
 *
 * 它不是「广播型传感器」，也不是夜灯 2 那种 NUS 文本协议，而是**标准 spec 设备**：
 *   2.1 on                 bool    开关
 *   2.2 brightness         uint16  1 ~ 65535（真机是无级调速）
 *   2.3 color-temperature  uint32  2700 ~ 6500 K
 *   3.3 delay-off          int16   0 ~ 60（延时关灯，可选实现）
 *   3.5 smart-switch       int32   0 ~ 1（断电记忆，可选实现）
 *   3.11 powerup-setting   枚举    上电状态，可选实现
 * 本工程实现 2.1 / 2.2 / 2.3 三条（App 页面上真正会动的就是这三条），其余留表位。
 *
 * 链路（与 esp32/thermo_gatt 完全一致，已被纯净 App 实测验证）：
 *   广播        未绑定 hello 形态（12B，registered=0）→ 绑定后 v5 加密帧
 *   绑定        0xA2 → ECDH → 云下发 did(92B) → 回 0x11
 *   会话        0x24 → appRand/devRand → 0x21
 *   数据        spec 属性帧推 0x0010（同发 0x001B，覆盖优化连接模型）
 *
 * ★PID 是唯一的未知量：
 *   公开渠道（小米官网、说明书、miot-spec）都只给型号 MJDP003，不给米家的
 *   蓝牙 PID。已按「换代沿用 pdid」推断填 0x06EB（= MJDP09YL 的 pdid 1771），
 *   见 BULB_PID_DEFAULT 的注释；仍不对才需要真机扫一帧
 *   （0xFE95 Service Data 第 3~4 字节，小端）。
 *   这里做成**运行期可改**：串口 `pid 0x1A2B` 写进 NVS 并重启，不用重新编译。
 *
 * 串口命令（USB-Serial/JTAG，115200）：
 *   info            当前状态（PID / 绑定 / 开关 / 亮度 / 色温）
 *   on / off        开关（等价于 App 点开关，会回推 properties_changed）
 *   bright <0~100>  按百分比调亮度（内部换算成 1~65535）
 *   ct <2700~6500>  设色温
 *   pid [0xXXXX]    查看 / 写入蓝牙 PID（写后自动重启）
 *   factory         擦 NVS 重启（回到未绑定；删设备重加前必须执行）
 *   reboot          重启
 */
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
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

#include "gatt_miot.h"

void miot_port_esp32_crypto_init(void);

static const char *TAG = "BULB";

/* ======================================================================
 * 改参数只动这一段
 * ==================================================================== */

/** 编译期默认 PID。
 *  ★ 0x06EB = pdid 1771 =「米家 Mesh 灯泡 MJDP09YL」——MJDP003 是它的替代型号
 *    （同为 E27 5W 500lm 2700~6500K，官方 BHR4861CN）。米家同品类换代沿用 pdid，
 *    所以先填这个值试绑定；若米家列表不出现，依次退 0x0926（Yeelight Mesh Bulb M2）
 *    / 0x06EC（米家 Mesh 筒灯），仍不行才需要真机扫一帧。
 *  0 = 未填。注意 NVS 里的 `pid` 键优先级高于本默认值，换默认值后必须 erase-flash。 */
#define BULB_PID_DEFAULT  0x06EB
/** 扫描响应里的设备名（米家列表显示的是插件名，这里只是 BLE local name） */
#define BULB_NAME         "MJDP003"
/** 广播周期（绑定后 500ms 保活，未绑定 100~200ms，由 adv_start 内决定） */
#define ADV_PERIOD_MS     1500

/** 灯输出脚：ESP32-C3 SuperMini 板载蓝灯 */
#define BULB_LED_GPIO        8
/** 1 = 低电平点亮；外接高电平灯带/灯珠改成 0 */
#define BULB_LED_ACTIVE_LOW  1
#define BULB_LED_PWM_HZ      5000
#define BULB_LED_DUTY_BITS   LEDC_TIMER_10_BIT
#define BULB_LED_DUTY_MAX    1023

/** BOOT 键：短按开关灯，长按 3 秒恢复出厂 */
#define BULB_BTN_GPIO        9
#define BULB_LONG_PRESS_MS   3000

#define NVS_NS       "bulb"
#define NVS_KEY_ID   "ident"
#define NVS_KEY_FCNT "fcnt"
#define NVS_KEY_PID  "pid"

/* ======================================================================
 * 属性表：2.1 开关 / 2.2 亮度 / 2.3 色温
 * ==================================================================== */

static uint8_t b_on     = 0;
static int32_t b_bright = 32768;   /* 1 ~ 65535 */
static int32_t b_ct     = 4000;    /* 2700 ~ 6500 K */

/** 待回推的属性号（set 成功后由主循环发 properties_changed） */
static volatile uint16_t s_dirty_piid;

static void bulb_on_set(const struct miot_prop *p)
{
    s_dirty_piid = p->piid;        /* 真机行为：set 之后立刻回报这一条 */
}

static miot_prop_t b_props[] = {
    { 2, 1, MIOT_TYPE_BOOL, MIOT_PROP_RW, 1, &b_on,     0,    1,     bulb_on_set, "on"         },
    { 2, 2, MIOT_TYPE_U16,  MIOT_PROP_RW, 4, &b_bright, 1,    65535, bulb_on_set, "brightness" },
    { 2, 3, MIOT_TYPE_U32,  MIOT_PROP_RW, 4, &b_ct,     2700, 6500,  bulb_on_set, "color_temp" },
};

/* ======================================================================
 * 状态
 * ==================================================================== */

static miot_auth_t   s_auth;
static miot_device_t s_dev;
static miot_port_t   s_port;

static uint8_t  s_mac_air[6];
static uint8_t  s_frame[32];
static int      s_idx;
static uint8_t  s_flen;
static int      s_logged_in;
static int      s_was_bound;

static uint16_t s_pid;            /* 运行期 PID（NVS 优先） */

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static uint32_t port_time_now(void *user)
{
    (void)user;
    return now_ms();
}

static int gap_event_cb(struct ble_gap_event *event, void *arg);

/* ======================================================================
 * LED：亮度百分比 → PWM。单色灯表达不了色温，只记值 + 打日志
 * ==================================================================== */

static void led_init(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = BULB_LED_DUTY_BITS,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = BULB_LED_PWM_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tcfg);

    ledc_channel_config_t ccfg = {
        .gpio_num   = BULB_LED_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = BULB_LED_ACTIVE_LOW ? BULB_LED_DUTY_MAX : 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ccfg);
    ESP_LOGI(TAG, "LED: GPIO%d %s, %dHz PWM（2.2 亮度 → 占空比）",
             BULB_LED_GPIO, BULB_LED_ACTIVE_LOW ? "低电平点亮" : "高电平点亮", BULB_LED_PWM_HZ);
}

static void led_sync(void)
{
    int pct = 0;
    if (b_on) {
        pct = (int)((b_bright * 100 + 32767) / 65535);
        if (pct < 1) pct = 1;              /* 亮度 1/65535 也要给一点光 */
    }
    int duty = (BULB_LED_DUTY_MAX * pct) / 100;
    if (BULB_LED_ACTIVE_LOW) duty = BULB_LED_DUTY_MAX - duty;
    if (duty < 0) duty = 0;
    if (duty > BULB_LED_DUTY_MAX) duty = BULB_LED_DUTY_MAX;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

/* ======================================================================
 * 身份 / 帧计数 / PID 持久化
 * ==================================================================== */

static void id_save(void *user, const miot_auth_identity_t *id)
{
    (void)user;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    esp_err_t err = nvs_set_blob(h, NVS_KEY_ID, id, sizeof(*id));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "identity saved (%s), did[0..3]=%02X %02X %02X %02X",
             err == ESP_OK ? "ok" : "FAIL", id->did[0], id->did[1], id->did[2], id->did[3]);
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
    ESP_LOGI(TAG, "identity loaded, did[0..3]=%02X %02X %02X %02X",
             id->did[0], id->did[1], id->did[2], id->did[3]);
    return true;
}

static uint32_t s_fcnt;

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

static uint16_t pid_load(void)
{
    nvs_handle_t h;
    uint16_t v = 0;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return BULB_PID_DEFAULT;
    if (nvs_get_u16(h, NVS_KEY_PID, &v) != ESP_OK) v = BULB_PID_DEFAULT;
    nvs_close(h);
    return v;
}

static void pid_save(uint16_t pid)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u16(h, NVS_KEY_PID, pid);
    nvs_commit(h);
    nvs_close(h);
}

/* ======================================================================
 * 日志 / 随机数
 * ==================================================================== */

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

/* ======================================================================
 * 广播
 * ==================================================================== */

/** 绑定后广播里带的那条对象：开关状态。对象号按 MiBeacon 习惯取 0x1012，
 *  真机若用别的号，只影响 App 列表里显示的开关状态，不影响 GATT 控制。 */
#define BULB_OBJ_ON  0x1012

static int build_frame(int idx)
{
    (void)idx;
    miot_beacon_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.pid = s_pid;
    memcpy(cfg.mac_air, s_mac_air, 6);

    if (!s_auth.bound) {
        /* 未绑定：只发 12B 身份心跳，App「附近设备」的唯一依据 */
        int fl = miot_beacon_frame_hello(&cfg, s_fcnt, 2, s_frame, sizeof(s_frame));
        if (fl < 0) { ESP_LOGE(TAG, "hello rc=%d", fl); return fl; }
        s_flen = (uint8_t)fl;
        return fl;
    }

    memcpy(cfg.beacon_key, s_auth.id.bind_key, 16);
    uint8_t objs[8];
    size_t n = miot_beacon_obj_u8(objs, BULB_OBJ_ON, b_on ? 1 : 0);

    int fl = miot_beacon_frame_v5(&cfg, s_fcnt, objs, n, s_frame, sizeof(s_frame));
    if (fl < 0) { ESP_LOGE(TAG, "frame rc=%d (crypto backend?)", fl); return fl; }
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
        ESP_LOGE(TAG, "adv_set_fields rc=%d (svc_data=%dB)", rc, n);
        return;
    }

    /* 0x038F V2 信标：0x20 = 未注册（App 据此判断要不要走绑定） */
    static uint8_t mfr[16];
    int m = 0;
    mfr[m++] = 0x8F; mfr[m++] = 0x03;
    mfr[m++] = 0x30;
    mfr[m++] = (uint8_t)(s_auth.bound ? 0x00 : 0x20);
    mfr[m++] = (uint8_t)(s_pid & 0xFF);
    mfr[m++] = (uint8_t)(s_pid >> 8);
    mfr[m++] = (uint8_t)s_fcnt;
    for (int i = 0; i < 6; i++) mfr[m++] = s_mac_air[i];
    mfr[m++] = 0x08;

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = (const uint8_t *)BULB_NAME;
    rsp.name_len = sizeof(BULB_NAME) - 1;
    rsp.name_is_complete = 1;
    rsp.mfg_data = mfr;
    rsp.mfg_data_len = (uint8_t)m;
    if (ble_gap_adv_rsp_set_fields(&rsp) != 0) ESP_LOGW(TAG, "adv_rsp set failed");

    struct ble_gap_adv_params p;
    memset(&p, 0, sizeof(p));
    p.conn_mode = BLE_GAP_CONN_MODE_UND;
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    p.itvl_min = s_auth.bound ? 800 : 160;
    p.itvl_max = s_auth.bound ? 800 : 320;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &p,
                           gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) ESP_LOGE(TAG, "adv_start rc=%d", rc);
}

static void adv_refresh(void)
{
    if (gatt_miot_is_connected()) return;
    if (ble_gap_adv_active()) ble_gap_adv_stop();
    adv_start();
}

/* ======================================================================
 * GAP 事件
 * ==================================================================== */

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            gatt_miot_on_connect(event->connect.conn_handle, BLE_ATT_MTU_DFLT);
            ble_gap_adv_stop();
            ESP_LOGI(TAG, "phone connected -> adv stopped");
        } else {
            ESP_LOGW(TAG, "connect failed status=%d", event->connect.status);
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
            build_frame(0);
        }
        adv_start();
        ESP_LOGI(TAG, "disconnected reason=0x%02x -> adv resumed", event->disconnect.reason);
        return 0;

    case BLE_GAP_EVENT_MTU:
        gatt_miot_on_mtu(event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        adv_start();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "subscribe attr=%u notify=%d",
                 event->subscribe.attr_handle, event->subscribe.cur_notify);
        return 0;

    default:
        return 0;
    }
}

/* ======================================================================
 * 状态回推
 * ==================================================================== */

/** 把某一条属性推给 App（set 成功后调用，真机也是这么回的） */
static void push_prop(uint16_t piid)
{
    if (!s_auth.logged_in || !gatt_miot_is_connected()) return;

    miot_spec_value_t v;
    memset(&v, 0, sizeof(v));
    v.siid = 2;
    v.piid = piid;
    if (piid == 1) {
        v.type = MIOT_TYPE_BOOL; v.val.u8 = b_on ? 1 : 0;
    } else if (piid == 2) {
        v.type = MIOT_TYPE_U16;  v.val.u16 = (uint16_t)b_bright;
    } else {
        v.type = MIOT_TYPE_U32;  v.val.u32 = (uint32_t)b_ct;
    }
    miot_device_send_props_changed(&s_dev, &v, 1);
    ESP_LOGI(TAG, "properties_changed 2.%u", piid);
}

/* ======================================================================
 * 按键：短按开关灯，长按 3 秒恢复出厂
 * ==================================================================== */

static void btn_task(void *arg)
{
    (void)arg;
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BULB_BTN_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);

    int64_t t_down = 0;
    bool down = false;
    bool long_done = false;

    for (;;) {
        int lv = gpio_get_level(BULB_BTN_GPIO);
        int64_t t = esp_timer_get_time();

        if (lv == 0 && !down) {
            down = true; long_done = false; t_down = t;
        } else if (lv == 0 && down && !long_done &&
                   (t - t_down) / 1000 >= BULB_LONG_PRESS_MS) {
            long_done = true;
            ESP_LOGW(TAG, "BOOT held %dms -> factory reset", BULB_LONG_PRESS_MS);
            nvs_flash_erase();
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        } else if (lv != 0 && down) {
            down = false;
            if (!long_done) {
                b_on = b_on ? 0 : 1;
                led_sync();
                push_prop(1);
                ESP_LOGI(TAG, "BOOT 短按 -> 2.1 = %u", b_on);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ======================================================================
 * 串口命令
 * ==================================================================== */

static void cli_task(void *arg)
{
    (void)arg;
    char line[128];
    for (;;) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        size_t n = strlen(line);
        while (n && (line[n - 1] == '\r' || line[n - 1] == '\n')) line[--n] = 0;
        if (!n) continue;

        if (!strcmp(line, "info")) {
            ESP_LOGW(TAG, "pid=0x%04X bound=%d login=%d conn=%d on=%u bright=%ld ct=%ldK",
                     s_pid, (int)s_auth.bound, (int)s_auth.logged_in,
                     (int)gatt_miot_is_connected(), b_on,
                     (long)b_bright, (long)b_ct);
        } else if (!strcmp(line, "on") || !strcmp(line, "off")) {
            b_on = (line[1] == 'n') ? 1 : 0;
            led_sync();
            push_prop(1);
            ESP_LOGW(TAG, "2.1 = %u", b_on);
        } else if (!strncmp(line, "bright", 6)) {
            const char *a = line + 6;
            while (*a == ' ') a++;
            if (*a) {
                int pct = atoi(a);
                if (pct < 0) pct = 0;
                if (pct > 100) pct = 100;
                b_bright = (pct == 0) ? 1 : (int32_t)((pct * 65535L) / 100);
                led_sync();
                push_prop(2);
                ESP_LOGW(TAG, "2.2 = %ld (%d%%)", (long)b_bright, pct);
            } else {
                ESP_LOGW(TAG, "bright 现在 %ld（%ld%%）", (long)b_bright,
                         (long)((b_bright * 100) / 65535));
            }
        } else if (!strncmp(line, "ct", 2)) {
            const char *a = line + 2;
            while (*a == ' ') a++;
            if (*a) {
                int v = atoi(a);
                if (v < 2700) v = 2700;
                if (v > 6500) v = 6500;
                b_ct = v;
                push_prop(3);
                ESP_LOGW(TAG, "2.3 = %ldK（单色灯只记值，不体现在灯珠上）", (long)b_ct);
            } else {
                ESP_LOGW(TAG, "ct 现在 %ldK", (long)b_ct);
            }
        } else if (!strncmp(line, "pid", 3)) {
            const char *a = line + 3;
            while (*a == ' ') a++;
            if (*a) {
                uint16_t v = (uint16_t)strtoul(a, NULL, 0);
                pid_save(v);
                ESP_LOGW(TAG, "PID 写为 0x%04X，重启生效", v);
                vTaskDelay(pdMS_TO_TICKS(300));
                esp_restart();
            } else {
                ESP_LOGW(TAG, "PID 现在 0x%04X", s_pid);
            }
        } else if (!strcmp(line, "factory")) {
            ESP_LOGW(TAG, "factory reset: erase NVS");
            nvs_flash_erase();
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        } else if (!strcmp(line, "reboot")) {
            esp_restart();
        } else {
            ESP_LOGW(TAG, "命令: info | on | off | bright 0-100 | ct 2700-6500 | pid [0xXXXX] | factory | reboot");
        }
    }
}

/* ======================================================================
 * 启动
 * ==================================================================== */

static void on_sync(void)
{
    uint8_t addr[6];
    if (ble_hs_util_ensure_addr(0) == 0 &&
        ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr, NULL) == 0) {
        /* 广播帧里的 MAC 必须是 LSB 序，nonce 是 mac||pid||cnt 拼的 */
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_BT);
        for (int i = 0; i < 6; i++) s_mac_air[i] = mac[5 - i];
    }
    ESP_LOGI(TAG, "host synced, MAC %02X:%02X:%02X:%02X:%02X:%02X  PID=0x%04X bound=%d",
             s_mac_air[5], s_mac_air[4], s_mac_air[3],
             s_mac_air[2], s_mac_air[1], s_mac_air[0], s_pid, s_auth.bound);

    s_idx = 0;
    build_frame(s_idx);
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

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    s_pid = pid_load();
    /* NVS 里的 `pid` 键优先级高于编译默认值（运行期 `pid 0xXXXX` 的遗产）。
     * 换默认值后不 erase-flash 会继续用旧值，这一行就是为了不让人误会。 */
    if (s_pid != BULB_PID_DEFAULT) {
        ESP_LOGW(TAG, "PID 0x%04X 来自 NVS，编译默认 0x%04X；要回默认值请 erase-flash",
                 s_pid, BULB_PID_DEFAULT);
    }

    /* ★独立 MAC：仓库里每个工程一个前缀，避免米家把「反复绑定失败过的地址」
     * 写进 DeviceBlackList 之后，所有工程一起扫不到。
     *   ESP_MAC_BT = 基础 MAC 末字节 +1，所以这里写 0x03 → 广播出去是 …:71:04。 */
    {
        uint8_t base[6] = { 0x10, 0x00, 0x3B, 0xCE, 0x71, 0x03 };
        esp_base_mac_addr_set(base);
    }

    ESP_LOGI(TAG, "===== 米家 LED 智能灯泡 蓝牙 Mesh 版（MJDP003）=====");
    if (s_pid == 0) {
        ESP_LOGE(TAG, "PID 还没填（当前 0x0000）—— 米家「附近设备」不会出现本设备");
        ESP_LOGE(TAG, "填法：扫真机 0xFE95 Service Data，第 3~4 字节（小端）即 PID，");
        ESP_LOGE(TAG, "      然后串口执行 `pid 0x1A2B`（会自动重启），不用重新编译。");
    } else {
        ESP_LOGI(TAG, "PID 0x%04X  MAC 后续打印", s_pid);
    }

    led_init();
    led_sync();

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    for (int i = 0; i < 6; i++) s_mac_air[i] = mac[5 - i];

    miot_port_esp32_crypto_init();
    fcnt_load();

    memset(&s_port, 0, sizeof(s_port));
    s_port.send_notify = gatt_miot_notify_spec;
    s_port.time_now    = port_time_now;
    s_port.random      = port_random;
    s_port.log         = port_log;

    miot_device_init(&s_dev, &s_port);
    miot_device_attach(&s_dev, b_props, sizeof(b_props) / sizeof(b_props[0]),
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
    ESP_LOGI(TAG, "ECDH prepare %lums", (unsigned long)(now_ms() - t0));

    if (nimble_port_init() != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed");
        return;
    }
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = NULL;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(BULB_NAME);
    gatt_miot_register_svcs();
    gatt_miot_bind(&s_auth, &s_dev);
    ble_att_set_preferred_mtu(247);

    nimble_port_freertos_init(host_task);
    xTaskCreate(btn_task, "btn", 3072, NULL, 3, NULL);
    xTaskCreate(cli_task, "cli", 4096, NULL, 4, NULL);

    uint32_t last_adv = 0;

    for (;;) {
        uint32_t t = now_ms();
        miot_auth_poll(&s_auth, t);

        if (s_auth.logged_in && !s_logged_in) {
            s_logged_in = 1;
            miot_device_set_session_keys(&s_dev, miot_auth_session_keys(&s_auth));
            ESP_LOGI(TAG, "*** session established: spec channel ready ***");
        }

        /* set 成功后回推（真机同款时序：先落值再报） */
        uint16_t piid = s_dirty_piid;
        if (piid) {
            s_dirty_piid = 0;
            led_sync();
            push_prop(piid);
        }

        if (s_auth.bound != s_was_bound) {
            s_was_bound = s_auth.bound;
            ESP_LOGI(TAG, "*** bind state -> %d ***", s_was_bound);
            build_frame(0);
            adv_refresh();
        }

        if (!gatt_miot_is_connected() && t - last_adv >= ADV_PERIOD_MS) {
            last_adv = t;
            s_fcnt++;
            if ((s_fcnt % 10) == 0) fcnt_save();
            if (build_frame(0) >= 0) adv_refresh();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
