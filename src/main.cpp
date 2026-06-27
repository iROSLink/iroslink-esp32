#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_task_wdt.h"
extern "C" {
    #include "zenoh-pico.h"
}
#include "secret.h"

// ── LED ───────────────────────────────────────────────────────────────────────
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define LED_PIN LED_BUILTIN   // GPIO2 on most ESP32 boards

// ── Motor pins ────────────────────────────────────────────────────────────────
#define MOTOR_L_IN1  25   // Left motor
#define MOTOR_L_IN2  33
#define MOTOR_R_IN1  26   // Right motor
#define MOTOR_R_IN2  27

// ── Fan ESC ───────────────────────────────────────────────────────────────────
// Driven via ledcAttach (same mechanism as motors) to avoid ESP32Servo channel conflicts.
// 50 Hz, 16-bit: 1 count = 20000/65536 ≈ 0.305 μs
#define ESC_PIN      19
#define ESC_FREQ_HZ  50
#define ESC_RES_BITS 16
#define ESC_MIN_US   1050
#define ESC_MAX_US   2100
// duty = us * 65536 / 20000
static inline uint32_t esc_us_to_duty(int us) {
    return (uint32_t)constrain(us, ESC_MIN_US, ESC_MAX_US) * 65536UL / 20000UL;
}
static void esc_write_us(int us) { ledcWrite(ESC_PIN, esc_us_to_duty(us)); }

// ── Boot button ───────────────────────────────────────────────────────────────
#define BOOT_BTN 0   // GPIO0, active LOW, external pull-up on dev boards

// ── Battery ADC ───────────────────────────────────────────────────────────────
// Resistor divider: R1=100kΩ, R2=27kΩ → 15V × 27/127 = 3.19V (within 3.3V)
// GPIO34 = ADC1_CH6 (input-only, safe for ADC use)
#define BAT_PIN      34
#define BAT_SCALE    (127.0f / 27.0f)  // V_bat = V_adc × (R1+R2)/R2

// ── Zenoh config ──────────────────────────────────────────────────────────────
#define MODE             "client"
#define ROUTER_PORT      7447
#define ROUTER_CUSTOM_IP "192.168.2.1"  // last-resort if mDNS + gateway both fail

static char g_locator[64];  // built after WiFi+mDNS

#define ROS2_DOMAIN  "0"
#define CMD_KEYEXPR  ROS2_DOMAIN "/cmd_vel/geometry_msgs::msg::dds_::Twist_/**"
#define FAN_KEYEXPR  ROS2_DOMAIN "/suction_fan/speed/std_msgs::msg::dds_::Float32_/**"
#define BAT_KEYEXPR  ROS2_DOMAIN "/battery/voltage/std_msgs::msg::dds_::Float32_/**"
// ── CDR layout for geometry_msgs/msg/Twist ────────────────────────────────────
// Offset  0- 3: CDR header
// Offset  4-11: linear.x  (float64)
// Offset 12-19: linear.y  (float64)
// Offset 20-27: linear.z  (float64)
// Offset 28-35: angular.x (float64)
// Offset 36-43: angular.y (float64)
// Offset 44-51: angular.z (float64)
// Total: 52 bytes (no padding — rmw_zenoh_cpp packs tightly after header)
struct Twist {
    double linear_x, linear_y, linear_z;
    double angular_x, angular_y, angular_z;
};

static bool decode_twist(const uint8_t *buf, size_t len, Twist &t) {
    if (len < 52) return false;
    memcpy(&t.linear_x,  buf + 4,  8);
    memcpy(&t.linear_y,  buf + 12, 8);
    memcpy(&t.linear_z,  buf + 20, 8);
    memcpy(&t.angular_x, buf + 28, 8);
    memcpy(&t.angular_y, buf + 36, 8);
    memcpy(&t.angular_z, buf + 44, 8);
    return true;
}

// Command state — Zenoh read task writes, loop() reads and drives motors.
// 4-byte float writes on Xtensa are atomic; volatile prevents caching.
static volatile float    g_cmd_linear   = 0.0f;
static volatile float    g_cmd_angular  = 0.0f;
static volatile bool     g_cmd_fresh    = false;
static volatile int64_t  g_last_recv_ms = 0;
static volatile uint32_t g_recv_count   = 0;  // total messages received (for Hz calc)

static TaskHandle_t g_loop_task = NULL;  // notified by callback to wake loop() immediately
static volatile bool g_zenoh_connected = true;  // cleared by bat_publish_task on put failure

// ── Motor control ─────────────────────────────────────────────────────────────
// Use LEDC at 25 kHz (above hearing range) to eliminate PWM whine.
// Channel assignments: 0=L_IN1, 1=L_IN2, 2=R_IN1, 3=R_IN2
#define PWM_FREQ_HZ  25000
#define PWM_RES_BITS 8      // 0–255

static void pwm_setup() {
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcAttach(MOTOR_L_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
        ledcAttach(MOTOR_L_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
        ledcAttach(MOTOR_R_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
        ledcAttach(MOTOR_R_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
    #else
        ledcSetup(0, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(MOTOR_L_IN1, 0);
        ledcSetup(1, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(MOTOR_L_IN2, 1);
        ledcSetup(2, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(MOTOR_R_IN1, 2);
        ledcSetup(3, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(MOTOR_R_IN2, 3);
    #endif
}

static int g_left_cur  = 0;
static int g_right_cur = 0;

static void set_motor(uint8_t ch_in1, uint8_t ch_in2, int speed) {
    speed = constrain(speed, -200, 200);
    if (speed > 0) {
        ledcWrite(ch_in1, speed);
        ledcWrite(ch_in2, 0);
    } else if (speed < 0) {
        ledcWrite(ch_in1, 0);
        ledcWrite(ch_in2, -speed);
    } else {
        // Active brake: both HIGH — shorts motor terminals, kills coil ringing
        ledcWrite(ch_in1, 255);
        ledcWrite(ch_in2, 255);
    }
}

// Ramp toward target in small steps to limit inrush current
static void ramp_motors(int left_target, int right_target) {
    const int STEP     = 20;  // PWM units per step
    const int DELAY_MS = 5;   // ms between steps
    while ((g_left_cur != left_target || g_right_cur != right_target) && !g_cmd_fresh) {
        if (g_left_cur  < left_target)  g_left_cur  = min(g_left_cur  + STEP, left_target);
        if (g_left_cur  > left_target)  g_left_cur  = max(g_left_cur  - STEP, left_target);
        if (g_right_cur < right_target) g_right_cur = min(g_right_cur + STEP, right_target);
        if (g_right_cur > right_target) g_right_cur = max(g_right_cur - STEP, right_target);
        #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
            set_motor(MOTOR_L_IN1, MOTOR_L_IN2, g_left_cur);  // 3.x: ledcWrite takes pin
            set_motor(MOTOR_R_IN1, MOTOR_R_IN2, g_right_cur);
        #else
            set_motor(0, 1, g_left_cur);                       // 2.x: ledcWrite takes channel
            set_motor(2, 3, g_right_cur);
        #endif
        delay(DELAY_MS);
    }
}

static void drive(double linear_x, double angular_z) {
    const double LINEAR_SCALE  = 510.0;
    const double ANGULAR_SCALE = 255.0;
    int left  = constrain((int)(linear_x * LINEAR_SCALE - angular_z * ANGULAR_SCALE), -200, 200);
    int right = constrain((int)(linear_x * LINEAR_SCALE + angular_z * ANGULAR_SCALE), -200, 200);
    // Serial.printf("  motors  left=%4d  right=%4d\n", left, right);
    ramp_motors(left, right);
}

// ── Fan ESC control ───────────────────────────────────────────────────────────
static void on_fan_cmd(z_loaned_sample_t *sample, void *) {
    uint8_t buf[8];
    z_bytes_reader_t reader = z_bytes_get_reader(z_sample_payload(sample));
    size_t n = z_bytes_reader_read(&reader, buf, sizeof(buf));
    Serial.printf("fan cb: %u bytes  [%02x %02x %02x %02x | %02x %02x %02x %02x]\n",
                  (unsigned)n,
                  buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
    if (n < 8) return;
    float throttle;
    memcpy(&throttle, buf + 4, 4);
    throttle = constrain(throttle, 0.0f, 1.0f);
    int us = ESC_MIN_US + (int)(throttle * (ESC_MAX_US - ESC_MIN_US));
    Serial.printf("fan throttle=%.2f  us=%d\n", throttle, us);
    esc_write_us(us);
}

// ── Zenoh session ─────────────────────────────────────────────────────────────
z_owned_session_t    s;
z_owned_subscriber_t sub;
z_owned_subscriber_t fan_sub;
z_owned_publisher_t  bat_pub;

// Battery publish runs in its own task — keeps loop() free from zenoh mutex blocking
static TaskHandle_t g_bat_task = NULL;
static void bat_publish_task(void *) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        float v_bat = analogRead(BAT_PIN) * (3.3f / 4095.0f) * BAT_SCALE;
        // Serial.printf("bat=%.2fV\n", v_bat);
        uint8_t cdr[8] = {0x00, 0x01, 0x00, 0x00};
        memcpy(cdr + 4, &v_bat, 4);
        z_owned_bytes_t payload;
        z_bytes_copy_from_buf(&payload, cdr, sizeof(cdr));
        // Known limitation: z_publisher_put returns 1 (batched) on success, not 0.
        // With Z_FEATURE_BATCHING=1 it never returns <0 even when the router is offline —
        // the library silently absorbs failed sends during auto-reconnect.
        // Proper fix requires z_declare_background_transport_events_listener but that
        // crashes on ESP32 (InstrFetchProhibited), likely needs Z_FEATURE_UNSTABLE_API.
        g_zenoh_connected = (z_publisher_put(z_publisher_loan(&bat_pub), z_bytes_move(&payload), NULL) < 0)
                            ? false : g_zenoh_connected;
    }
}

static uint8_t s_cdr_buf[52];  // pre-allocated — avoids heap alloc in callback

static void on_cmd_vel(z_loaned_sample_t *sample, void *) {
    g_last_recv_ms = (int64_t)millis();
    g_recv_count+=1;

    z_bytes_reader_t reader = z_bytes_get_reader(z_sample_payload(sample));
    size_t n = z_bytes_reader_read(&reader, s_cdr_buf, sizeof(s_cdr_buf));

    Twist t;
    if (decode_twist(s_cdr_buf, n, t)) {
        g_cmd_linear  = (float)t.linear_x;
        g_cmd_angular = (float)t.angular_z;
    } else {
        g_cmd_linear  = 0.0f;
        g_cmd_angular = 0.0f;
    }

    g_cmd_fresh = true;
    if (g_loop_task) {
        // vTaskNotifyGiveFromISR(g_loop_task, NULL);  // wake loop() immediately
        xTaskNotifyGive(g_loop_task);
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(74880);

    // Configure WDT: 10s timeout, panic=true. loopTask is NOT registered yet —
    // registration happens in loop() after setup() completes, so blocking WiFi/zenoh
    // init here won't trigger it.
    esp_task_wdt_config_t wdt_cfg = { .timeout_ms = 10000, .idle_core_mask = 0, .trigger_panic = true };
    esp_task_wdt_reconfigure(&wdt_cfg);

    // LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Boot button
    pinMode(BOOT_BTN, INPUT_PULLUP);

    // Motor pins — ensure stopped before anything else
    pinMode(MOTOR_L_IN1, OUTPUT); pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT); pinMode(MOTOR_R_IN2, OUTPUT);
    pwm_setup();
    drive(0, 0);

    // ESC — arm: send min signal for 2 s via LEDC (avoids ESP32Servo channel conflict)
    Serial.println("Arming ESC...");
    ledcAttach(ESC_PIN, ESC_FREQ_HZ, ESC_RES_BITS);
    esc_write_us(ESC_MIN_US);
    delay(2000);
    Serial.println("ESC armed.");

    // ADC attenuation for 0–3.3 V range (battery divider output)
    analogSetAttenuation(ADC_11db);

    // WiFi — cycle through WIFI_NETWORKS until one connects, blink LED while trying
    struct { const char *ssid; const char *pass; } networks[] = { WIFI_NETWORKS };
    const int NET_COUNT = sizeof(networks) / sizeof(networks[0]);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // modem sleep buffers UDP; disable for low-latency
    delay(100);
    for (int i = 0; i < NET_COUNT; i++) {
        Serial.printf("Connecting to WiFi '%s'...", networks[i].ssid);
        WiFi.begin(networks[i].ssid, networks[i].pass);
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
        if (WiFi.status() == WL_CONNECTED) {
            digitalWrite(LED_PIN, LOW);
            Serial.print("OK — IP: ");
            Serial.println(WiFi.localIP());
            break;
        }
        Serial.println("FAILED");
        WiFi.disconnect(true);
        delay(200);
        if (i == NET_COUNT - 1) {
            Serial.println("All networks failed — rebooting");
            delay(100);
            ESP.restart();
        }
    }

    // Router discovery: mDNS → gateway → custom IP
    // MDNS.begin makes the ESP32 itself discoverable; hostByName resolves via lwIP
    MDNS.begin("ibot-drive");
    IPAddress router_ip;
    String mdns_fqdn = String(ROUTER_MDNS_HOST) + ".local";
    int mdns_ok = WiFi.hostByName(mdns_fqdn.c_str(), router_ip);
    if (mdns_ok == 1 && (uint32_t)router_ip != 0) {
        snprintf(g_locator, sizeof(g_locator), "tcp/%s:%d", router_ip.toString().c_str(), ROUTER_PORT);
        Serial.printf("mDNS: %s → %s\n", mdns_fqdn.c_str(), g_locator);
    } else {
        IPAddress gw = WiFi.gatewayIP();
        if ((uint32_t)gw != 0) {
            snprintf(g_locator, sizeof(g_locator), "tcp/%s:%d", gw.toString().c_str(), ROUTER_PORT);
            Serial.printf("mDNS failed — gateway: %s\n", g_locator);
        } else {
            snprintf(g_locator, sizeof(g_locator), "tcp/%s:%d", ROUTER_CUSTOM_IP, ROUTER_PORT);
            Serial.printf("mDNS+GW failed — custom: %s\n", g_locator);
        }
    }

    // Zenoh — retry until session opens and subscriber is declared
    while (true) {
        z_owned_config_t config;
        z_config_default(&config);
        zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, MODE);
        zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, g_locator);
        zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MULTICAST_SCOUTING_KEY, "false");

        Serial.print("Opening Zenoh Session...");
        if (z_open(&s, z_config_move(&config), NULL) < 0) {
            digitalWrite(LED_PIN,HIGH);
            delay(100);
            Serial.println("FAILED — retry in 2s");
            digitalWrite(LED_PIN,LOW);
            delay(2000);
            continue;
        }
        Serial.println("OK");

        delay(200);  // let auto-started session executor settle before declaring

        z_owned_closure_sample_t closure;
        z_closure_sample(&closure, on_cmd_vel, NULL, NULL);
        z_view_keyexpr_t ke;
        z_view_keyexpr_from_str_unchecked(&ke, CMD_KEYEXPR);
        if (z_declare_subscriber(z_session_loan(&s), &sub, z_view_keyexpr_loan(&ke),
                                 z_closure_sample_move(&closure), NULL) < 0) {
            Serial.println("Unable to declare subscriber — retry in 2s");
            delay(1000);
            ESP.restart();
        }
        // Fan subscriber
        {
            z_owned_closure_sample_t fan_closure;
            z_closure_sample(&fan_closure, on_fan_cmd, NULL, NULL);
            z_view_keyexpr_t fan_ke;
            z_view_keyexpr_from_str_unchecked(&fan_ke, FAN_KEYEXPR);
            z_declare_subscriber(z_session_loan(&s), &fan_sub,
                                 z_view_keyexpr_loan(&fan_ke),
                                 z_closure_sample_move(&fan_closure), NULL);
        }

        // Battery publisher
        {
            z_view_keyexpr_t bat_ke;
            z_view_keyexpr_from_str_unchecked(&bat_ke, BAT_KEYEXPR);
            z_declare_publisher(z_session_loan(&s), &bat_pub,
                                z_view_keyexpr_loan(&bat_ke), NULL);
        }

        xTaskCreate(bat_publish_task, "bat_pub", 4096, NULL, 1, &g_bat_task);
        break;
    }
    Serial.print("Subscribed to: ");
    Serial.println(CMD_KEYEXPR);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    if (!g_loop_task) {
        g_loop_task = xTaskGetCurrentTaskHandle();
        esp_task_wdt_add(NULL);  // register loop task with hardware watchdog
    }
    esp_task_wdt_reset();  // feed watchdog — if loop() blocks >10s, WDT resets chip

    static uint32_t last_zenoh_ms       = 0;
    static uint32_t last_hz_ms          = 0;
    static uint32_t last_hz_count       = 0;
    static uint32_t last_bat_ms         = 0;
    static float    recv_hz             = 0.0f;
    static bool     btn_prev            = HIGH;
    static uint32_t btn_debounce_ms     = 0;
    static bool     g_fan_on            = false;
    uint32_t now = millis();

    // Boot button — falling edge → toggle fan 50% / off
    bool btn_now = digitalRead(BOOT_BTN);
    if (btn_prev == HIGH && btn_now == LOW && (now - btn_debounce_ms) > 200) {
        btn_debounce_ms = now;
        g_fan_on = !g_fan_on;
        int target_us = g_fan_on ? (ESC_MIN_US + (ESC_MAX_US - ESC_MIN_US) / 2) : ESC_MIN_US;
        esc_write_us(target_us);
        Serial.printf("fan %s (%d us)\n", g_fan_on ? "ON 50%" : "OFF", target_us);
    }
    btn_prev = btn_now;

    // Update Hz every 1 s
    if (now - last_hz_ms >= 1000) {
        uint32_t cnt = g_recv_count;
        recv_hz = (cnt - last_hz_count) * 1000.0f / (float)(now - last_hz_ms);
        last_hz_count = cnt;
        last_hz_ms    = now;
    }

    // Apply motor command from Zenoh callback — sole task driving motors
    if (g_cmd_fresh) {
        digitalWrite(LED_PIN,HIGH);
        g_cmd_fresh = false;
        float lin = g_cmd_linear;
        float ang = g_cmd_angular;
        int64_t recv_ms = g_last_recv_ms;

        Serial.printf(">> cmd_vel  lin.x=%.3f  ang.z=%.3f  lag=%ldms  %.1fHz\n",
                      lin, ang, (long)(now - recv_ms), recv_hz);

        last_zenoh_ms = now;
        drive(lin, ang);
        digitalWrite(LED_PIN,LOW);
    } else {
        if (now - last_zenoh_ms > 500) {
            drive(0, 0);
            if (!g_zenoh_connected) {
                // router offline / reconnecting: 100ms on, 2000ms off, not work yet!
                digitalWrite(LED_PIN, (now % 2100) < 100 ? HIGH : LOW);
            } else {
                // slow blink = no comms
                digitalWrite(LED_PIN, (now % 2000) < 1000 ? HIGH : LOW);
            }
        }
    }

    // Last-resort watchdog: client mode auto-reconnect retries every 1s indefinitely,
    // so only reboot if it has been stuck for an unreasonably long time (5 min).
    if (last_zenoh_ms > 0 && (now - last_zenoh_ms) > 300000) {
        Serial.println("Zenoh silent >5min — rebooting");
        delay(100);
        ESP.restart();
    }

    // Battery voltage — wake publish task every 2 s (task owns zenoh call, can't block loop)
    if (now - last_bat_ms >= 2000) {
        last_bat_ms = now;
        if (g_bat_task) xTaskNotifyGive(g_bat_task);

        // WiFi watchdog — if connection dropped, reboot to reconnect cleanly
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi lost — rebooting");
            drive(0, 0);
            delay(100);
            ESP.restart();
        }
    }

    // Block until callback wakes us or safety timeout fires
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
}