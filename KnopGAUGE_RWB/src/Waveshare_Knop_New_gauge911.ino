#include "lcd_bsp.h"
#include "cst816.h"
#include "lcd_bl_pwm_bsp.h"
#include "lcd_config.h"
#include "ui.h"

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "lvgl.h"

//====== CUSTOM FONTS ======
extern const lv_font_t ui_font_Hollow85;
extern const lv_font_t ui_font_Hollow38;
extern const lv_font_t ui_font_Hollow22;
extern const lv_font_t ui_font_t20;

//==================================================
//               COLOR SCHEMES
//   TAP SCREEN TO CYCLE BETWEEN THEM
//==================================================
typedef struct {
    uint32_t bg;           // screen background
    uint32_t gauge_lime;   // tick + outer arc
    uint32_t scale_text;   // 0..8 numbers
    uint32_t inner_start;  // inner ring start
    uint32_t inner_end;    // inner ring end
    uint32_t needle;       // needle color
    uint32_t hub_fill;     // center hub fill
    uint32_t hub_border;   // center hub border
    uint32_t rpm_text;     // big numeric RPM in center
    uint32_t brand_text;   // "RAUH Welt BEGRIFF"
} ColorScheme;

// Skin 0 – RAUH Welt (lime / red)
static const ColorScheme color_schemes[] = {
    {
        .bg         = 0x000000,
        .gauge_lime = 0xB7FF00,
        .scale_text = 0x99CF11,
        .inner_start= 0x222222,
        .inner_end  = 0x000000,
        .needle     = 0xFF3B30,
        .hub_fill   = 0x111111,
        .hub_border = 0xB7FF00,
        .rpm_text   = 0x6C7780,
        .brand_text = 0x8F8988
    },
    // Skin 1 – White / red motorsport
    {
        .bg         = 0x101010,
        .gauge_lime = 0xFFFFFF, // white ticks/arc
        .scale_text = 0xFF3B30, // red numbers
        .inner_start= 0x444444,
        .inner_end  = 0x111111,
        .needle     = 0xFF3B30, // red needle
        .hub_fill   = 0x000000,
        .hub_border = 0xFFFFFF,
        .rpm_text   = 0xFFFFFF,
        .brand_text = 0x8F8988
    },
    // Skin 2 – Cyan / gold “retro digital”
    {
        .bg         = 0x000000,
        .gauge_lime = 0x00E0FF, // cyan ticks/arc
        .scale_text = 0x00E0FF,
        .inner_start= 0x003344,
        .inner_end  = 0x000000,
        .needle     = 0xFFC800, // warm gold needle
        .hub_fill   = 0x000000,
        .hub_border = 0x00E0FF,
        .rpm_text   = 0x00E0FF,
        .brand_text = 0x8F8988
    }
};

static int current_scheme = 0;

//==================================================
//            LABEL OFFSETS (TEXT LAYOUT)
//==================================================
static const int RPM_Y_OFFSET    =  50;

static const int NAME0_Y_OFFSET  = -72;  // "911"
static const int NAME_Y_OFFSET   = -33;  // "porsche"
static const int NAME1_Y_OFFSET  = 100;  // "rwb janine"
static const int NAME2_Y_OFFSET  = 130;  // "RAUH Welt BEGRIFF"

//==================================================
//            ESP-NOW PACKET STRUCT
//==================================================
typedef struct {
  uint16_t rpm;
  float batt;
  float motor;
  float dk;
  float gp;
  uint8_t funk;
} DashPacket;

DashPacket lastPacket;
volatile uint16_t g_rpm = 0;
unsigned long lastPacketTime = 0;

//==================================================
//                 GAUGE PARAMS
//==================================================
static const int32_t RPM_MIN   = 0;
static const int32_t RPM_MAX   = 8000;
static const int32_t SCALE_MIN = 0;
static const int32_t SCALE_MAX = 8;

static const int GAUGE_START_DEG = 135;
static const int GAUGE_SWEEP_DEG = 270;

static const int ARC_WIDTH = 14;

// Needle ratios relative to radius
static const float NEEDLE_INNER_RATIO = 0.20f;
static const float NEEDLE_OUTER_RATIO = 0.75f;

static const int      NEEDLE_WIDTH = 6;
static const lv_opa_t NEEDLE_OPA   = LV_OPA_50;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//==================================================
//                 GAUGE OBJECTS
//==================================================
static lv_obj_t  *g_meter       = nullptr;
static lv_obj_t  *g_needle_line = nullptr;
static lv_point_t g_needle_points[2];

static lv_obj_t *g_rpm_label   = nullptr;
static lv_obj_t *g_name0_label = nullptr;
static lv_obj_t *g_name_label  = nullptr;
static lv_obj_t *g_name1_label = nullptr;
static lv_obj_t *g_name2_label = nullptr;
static lv_obj_t *g_center      = nullptr;

//==================================================
//        FORWARD DECLARATION (TOUCH HANDLER)
//==================================================
static void Lvgl_CreateGauge(void);

//==================================================
//           NEEDLE UPDATE (LINE OBJECT)
//==================================================
static void update_needle_line(uint16_t rpm)
{
    if (!g_meter || !g_needle_line) return;

    int32_t val = rpm;
    if (val < RPM_MIN) val = RPM_MIN;
    if (val > RPM_MAX) val = RPM_MAX;

    float scale_val = (float)val / 1000.0f;
    if (scale_val < SCALE_MIN) scale_val = SCALE_MIN;
    if (scale_val > SCALE_MAX) scale_val = SCALE_MAX;

    float t = (scale_val - SCALE_MIN) / (float)(SCALE_MAX - SCALE_MIN);
    float angle_deg = GAUGE_START_DEG + t * GAUGE_SWEEP_DEG;
    float angle_rad = angle_deg * (float)M_PI / 180.0f;

    lv_coord_t dw = lv_disp_get_hor_res(NULL);
    lv_coord_t dh = lv_disp_get_ver_res(NULL);
    lv_coord_t cx = dw / 2;
    lv_coord_t cy = dh / 2;

    lv_coord_t meter_radius = lv_obj_get_width(g_meter) / 2;
    float outer_r = (float)meter_radius - ARC_WIDTH * 0.5f;
    float inner_r = outer_r * NEEDLE_INNER_RATIO;
    outer_r      *= NEEDLE_OUTER_RATIO;

    g_needle_points[0].x = cx + cosf(angle_rad) * inner_r;
    g_needle_points[0].y = cy + sinf(angle_rad) * inner_r;
    g_needle_points[1].x = cx + cosf(angle_rad) * outer_r;
    g_needle_points[1].y = cy + sinf(angle_rad) * outer_r;

    lv_line_set_points(g_needle_line, g_needle_points, 2);
}

//==================================================
//           TOUCH CALLBACK – CYCLE SKINS
//==================================================
static void touch_color_cycle_cb(lv_event_t * e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    current_scheme++;
    if (current_scheme >= (int)(sizeof(color_schemes) / sizeof(color_schemes[0]))) {
        current_scheme = 0;
    }

    Serial.printf("Switching to color scheme %d\n", current_scheme);
    Lvgl_CreateGauge();
}

//==================================================
//                 CREATE GAUGE
//==================================================
static void Lvgl_CreateGauge()
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);

    const ColorScheme *cs = &color_schemes[current_scheme];

    lv_color_t color_bg          = lv_color_hex(cs->bg);
    lv_color_t color_lime        = lv_color_hex(cs->gauge_lime);
    lv_color_t color_scale_text  = lv_color_hex(cs->scale_text);
    lv_color_t color_inner_start = lv_color_hex(cs->inner_start);
    lv_color_t color_inner_end   = lv_color_hex(cs->inner_end);
    lv_color_t color_needle      = lv_color_hex(cs->needle);
    lv_color_t color_hub_fill    = lv_color_hex(cs->hub_fill);
    lv_color_t color_hub_border  = lv_color_hex(cs->hub_border);
    lv_color_t color_rpm_text    = lv_color_hex(cs->rpm_text);
    lv_color_t color_brand_text  = lv_color_hex(cs->brand_text);

    // Screen background
    lv_obj_set_style_bg_color(scr, color_bg, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    // Meter
    g_meter = lv_meter_create(scr);
    lv_obj_set_size(g_meter, 360, 360);
    lv_obj_center(g_meter);

    lv_obj_set_style_bg_opa(g_meter, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_meter, 0, 0);

    // Numbers 0–8 use Hollow85 + scale color
    lv_obj_set_style_text_font(g_meter, &ui_font_Hollow85,
                               LV_PART_TICKS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(g_meter, color_scale_text,
                                LV_PART_TICKS | LV_STATE_DEFAULT);

    lv_meter_scale_t *scale = lv_meter_add_scale(g_meter);
    lv_meter_set_scale_range(g_meter, scale,
                             SCALE_MIN, SCALE_MAX,
                             GAUGE_SWEEP_DEG, GAUGE_START_DEG);

    // Ticks
    lv_meter_set_scale_ticks(g_meter, scale, 41, 2, 10, color_lime);
    lv_meter_set_scale_major_ticks(g_meter, scale,
                                   5, 4, 20, color_lime, 13);

    // Outer arc
    lv_meter_indicator_t *outer_arc =
        lv_meter_add_arc(g_meter, scale, ARC_WIDTH, color_lime, 0);
    lv_meter_set_indicator_start_value(g_meter, outer_arc, SCALE_MIN);
    lv_meter_set_indicator_end_value(g_meter, outer_arc, SCALE_MAX);

    // Inner ring
    lv_meter_indicator_t *inner_ring =
        lv_meter_add_scale_lines(g_meter, scale,
                                 color_inner_start,
                                 color_inner_end,
                                 false,
                                 0);
    lv_meter_set_indicator_start_value(g_meter, inner_ring, SCALE_MIN);
    lv_meter_set_indicator_end_value(g_meter, inner_ring, SCALE_MAX);

    // Needle line (as separate lv_line object)
    g_needle_line = lv_line_create(scr);
    lv_obj_remove_style_all(g_needle_line);
    lv_obj_set_style_line_width(g_needle_line, NEEDLE_WIDTH, 0);
    lv_obj_set_style_line_color(g_needle_line, color_needle, 0);
    lv_obj_set_style_line_opa(g_needle_line, NEEDLE_OPA, 0);
    lv_line_set_points(g_needle_line, g_needle_points, 2);

    // Center hub
    g_center = lv_obj_create(scr);
    lv_obj_remove_style_all(g_center);
    lv_obj_set_size(g_center, 26, 26);
    lv_obj_set_style_radius(g_center, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(g_center, color_hub_fill, 0);
    lv_obj_set_style_border_width(g_center, 3, 0);
    lv_obj_set_style_border_color(g_center, color_hub_border, 0);
    lv_obj_align(g_center, LV_ALIGN_CENTER, 0, 0);

    // Center RPM text
    g_rpm_label = lv_label_create(scr);
    lv_obj_set_style_text_color(g_rpm_label, color_rpm_text, 0);
    lv_obj_set_style_text_font(g_rpm_label, &ui_font_Hollow85, 0);
    lv_label_set_text(g_rpm_label, "0000");
    lv_obj_align(g_rpm_label, LV_ALIGN_CENTER, 0, RPM_Y_OFFSET);

    // "911"
    g_name0_label = lv_label_create(scr);
    lv_label_set_text(g_name0_label, "911");
    lv_obj_set_style_text_font(g_name0_label, &ui_font_Hollow85, 0);
    lv_obj_set_style_text_color(g_name0_label, lv_color_hex(0xB3B9C4), 0);
    lv_obj_align(g_name0_label, LV_ALIGN_CENTER, 0, NAME0_Y_OFFSET);

    // "porsche"
    g_name_label = lv_label_create(scr);
    lv_label_set_text(g_name_label, "porsche");
    lv_obj_set_style_text_font(g_name_label, &ui_font_Hollow38, 0);
    lv_obj_set_style_text_color(g_name_label, lv_color_hex(0x85807F), 0);
    lv_obj_align(g_name_label, LV_ALIGN_CENTER, 0, NAME_Y_OFFSET);

    // "rwb janine"
    g_name1_label = lv_label_create(scr);
    lv_label_set_text(g_name1_label, "rwb janine");
    lv_obj_set_style_text_font(g_name1_label, &ui_font_Hollow38, 0);
    lv_obj_set_style_text_color(g_name1_label, lv_color_hex(0x3B3333), 0);
    lv_obj_align(g_name1_label, LV_ALIGN_CENTER, 0, NAME1_Y_OFFSET);

    // "RAUH Welt BEGRIFF"
    g_name2_label = lv_label_create(scr);
    lv_label_set_text(g_name2_label, "RAUH Welt BEGRIFF");
    lv_obj_set_style_text_font(g_name2_label, &ui_font_t20, 0);
    lv_obj_set_style_text_color(g_name2_label, color_brand_text, 0);
    lv_obj_align(g_name2_label, LV_ALIGN_CENTER, 0, NAME2_Y_OFFSET);

    // Full-screen touch area to change scheme
    lv_obj_t *touch_btn = lv_btn_create(scr);
    lv_obj_remove_style_all(touch_btn);
    lv_obj_set_size(touch_btn, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_opa(touch_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(touch_btn, 0, 0);
    lv_obj_add_event_cb(touch_btn, touch_color_cycle_cb, LV_EVENT_CLICKED, NULL);

    // Start needle in correct place
    update_needle_line(g_rpm);
}

//==================================================
//           ESP-NOW RECEIVER CALLBACK
//==================================================
static void OnDataRecv(const esp_now_recv_info_t *recv_info,
                       const uint8_t *incomingData, int len)
{
    Serial.println("\n===== ESP-NOW PACKET RECEIVED =====");

    if (recv_info) {
        Serial.printf("From: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      recv_info->src_addr[0], recv_info->src_addr[1],
                      recv_info->src_addr[2], recv_info->src_addr[3],
                      recv_info->src_addr[4], recv_info->src_addr[5]);
    }
    Serial.printf("Len: %d\n", len);

    if (len != sizeof(DashPacket)) {
        Serial.printf("Invalid packet size (%d != %d)\n", len, sizeof(DashPacket));
        return;
    }

    memcpy(&lastPacket, incomingData, sizeof(DashPacket));

    Serial.printf("RPM  : %u\n", lastPacket.rpm);
    Serial.printf("Batt : %.2f\n", lastPacket.batt);
    Serial.printf("Motor: %.2f\n", lastPacket.motor);
    Serial.printf("DK   : %.1f\n", lastPacket.dk);
    Serial.printf("GP   : %.1f\n", lastPacket.gp);
    Serial.printf("Funk : %u\n",  lastPacket.funk);

    if (lastPacket.rpm > RPM_MAX) lastPacket.rpm = RPM_MAX;

    g_rpm = lastPacket.rpm;
    lastPacketTime = millis();
}

//==================================================
//           ESP-NOW INITIALIZATION
//==================================================
static void setupEspNow()
{
    Serial.println("Starting ESP-NOW Receiver...");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_err_t err = esp_now_init();
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        Serial.printf("esp_now_init FAILED: 0x%X\n", err);
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    Serial.print("WiFi STA MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.println("ESP-NOW READY (RX ONLY).");
}

//==================================================
//         LVGL TASK: UPDATE GAUGE FROM g_rpm
//==================================================
static void lvgl_task(void *arg)
{
    for (;;) {
        lv_timer_handler();

        static uint16_t last_rpm = 0;
        uint16_t rpm = g_rpm;

        if (rpm != last_rpm) {
            last_rpm = rpm;

            update_needle_line(rpm);

            if (g_rpm_label) {
                lv_label_set_text_fmt(g_rpm_label, "%u", rpm);
            }

            Serial.printf("Gauge update: RPM=%u\n", rpm);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

//==================================================
//                     SETUP
//==================================================
void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.printf("DashPacket sizeof() on RX: %d bytes\n", sizeof(DashPacket));

    Touch_Init();          // CST816 touch
    lcd_lvgl_Init();       // Waveshare LVGL+LCD init
    Lvgl_CreateGauge();    // our custom RPM gauge
    lcd_bl_pwm_bsp_init(40);  // backlight 0–255

    setupEspNow();

    // Start LVGL task
    xTaskCreate(lvgl_task, "lvgl",
                EXAMPLE_LVGL_TASK_STACK_SIZE,
                NULL,
                EXAMPLE_LVGL_TASK_PRIORITY,
                NULL);

    Serial.println("Setup complete. Waiting for ESP-NOW data...");
}

//==================================================
//                     LOOP
//==================================================
void loop()
{
    // Everything runs in tasks
}
