/**
 * @file    smart_cabinet.ino
 * @brief   Smart Cabinet v4.1 – ESP8266 + Blynk + HC-SR04 + PIR + Servo + DFPlayer
 *
 * Fixes applied vs v3.2:
 *  [F1] Loại bỏ delay() blocking trong rotateMg996() → dùng non-blocking state machine
 *  [F2] Gộp đo khoảng cách về một nơi duy nhất, tránh 2 lần kích TRIG đồng thời
 *  [F3] resetAllZones() nay reset cả waitingShowB, tránh SHOW_B phát sau nguy hiểm
 *  [F4] muteClose luôn set closeSoundPlayed = true để tránh phát liên tục
 *  [F5] warnPlaying reset đúng chỗ, tránh phát lặp WARN trong cùng lần tiếp cận
 *  [F6] audioPriority reset về 0 khi danger track kết thúc trước khi SHOW_B chạy
 *
 * Updates v4.1 (bổ sung từ bảng logic spec):
 *  [U1] <10cm + PIR=false → VẪN đóng tủ (an toàn ưu tiên cao nhất)
 *  [U2] <10cm + PIR=true  → Đóng tủ + phát TRACK_DANGER (không đổi)
 *  [U3] 10-15cm / 15-20cm + PIR=false → Không làm gì (đã đúng từ v4.0)
 */

/* ─── Blynk credentials ─────────────────────────────────────────────────── */
#define BLYNK_TEMPLATE_ID    "TMPL6p2hqp0nI"
#define BLYNK_TEMPLATE_NAME  "Smart Cabinet"
#define BLYNK_AUTH_TOKEN     "BrJbNre1-W0G1ILOSXr39dZoItG0PgEh"
#define BLYNK_PRINT          Serial

/* ─── Libraries ─────────────────────────────────────────────────────────── */
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

/* ─── WiFi ──────────────────────────────────────────────────────────────── */
static const char *WIFI_SSID = "Tiny Cafe - Revolution";
static const char *WIFI_PASS = "12345678";

/* ─── GPIO pin mapping ──────────────────────────────────────────────────── */
#define PIN_TRIG        D1
#define PIN_ECHO        D2
#define PIN_PIR         D3
#define PIN_DF_RX       D4
#define PIN_MG995       D5
#define PIN_MG996       D6
#define PIN_BUTTON      D7
#define PIN_DF_TX       D8

/* ─── Blynk virtual pins ────────────────────────────────────────────────── */
#define VP_DISTANCE     V0
#define VP_STATUS       V1
#define VP_OPEN         V2
#define VP_STAFF        V3
#define VP_MUTE         V4
#define VP_CLOSE        V5

/* ─── MG996R continuous servo ───────────────────────────────────────────── */
#define MG996_STOP      1500    /* µs – hold still            */
#define MG996_CW        1300    /* µs – open direction        */
#define MG996_CCW       1700    /* µs – close direction       */
#define MG996_TURN_MS   3500UL  /* run time for one full turn */

/* ─── MG995 sweep parameters ────────────────────────────────────────────── */
#define MG995_MIN_US    817
#define MG995_MAX_US    2294
#define MG995_STEP_US   22      /* µs per tick                */
#define MG995_STEP_MS   15UL    /* ms between ticks           */

/* ─── HC-SR04 distance thresholds (cm) ─────────────────────────────────── */
#define DIST_DANGER     10.0f
#define DIST_WARN       15.0f
#define DIST_SHOW       20.0f
#define DIST_TOL         5.0f   /* zone stability tolerance   */

/* ─── Radar zone tracking ───────────────────────────────────────────────── */
#define ZONE_COUNT       14
#define ZONE_SIZE_DEG    10
#define ZONE_START_DEG   30
#define ZONE_CONFIRM_MS  10000UL

/* ─── DFPlayer track index & length ────────────────────────────────────── */
#define TRACK_DANGER     1
#define TRACK_WARN       2
#define TRACK_SHOW_A     3
#define TRACK_SHOW_B     4

#define LEN_DANGER_MS    5000UL
#define LEN_WARN_MS      4000UL
#define LEN_SHOW_A_MS   10000UL
#define LEN_SHOW_B_MS   10000UL

/* ─── PIR timeout ───────────────────────────────────────────────────────── */
#define PIR_TIMEOUT_MS  10000UL

/* ─── Sensor polling interval ───────────────────────────────────────────── */
#define SENSOR_PERIOD_MS  200UL

/* ══════════════════════════════════════════════════════════════════════════
 *  DATA TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    bool          active;
    float         dist;
    unsigned long first_seen_ms;
    unsigned long last_seen_ms;
} zone_t;

/** Non-blocking MG996R state machine */
typedef enum {
    MG996_IDLE = 0,
    MG996_RUNNING,
    MG996_STOPPING
} mg996_state_t;

/* ══════════════════════════════════════════════════════════════════════════
 *  GLOBAL STATE
 * ══════════════════════════════════════════════════════════════════════════ */

/* Hardware objects */
static Servo              g_mg995;
static Servo              g_mg996;
static SoftwareSerial     g_df_serial(PIN_DF_RX, PIN_DF_TX);
static DFRobotDFPlayerMini g_player;
static BlynkTimer         g_blynk_timer;

/* Cached distance (shared between sensor handler & Blynk timer) [F2] */
static float g_last_dist = -1.0f;

/* MG995 sweep */
static int  g_mg995_us       = MG995_MIN_US;
static bool g_mg995_going_up = true;

/* MG996R non-blocking state machine [F1] */
static mg996_state_t  g_mg996_state      = MG996_IDLE;
static bool           g_mg996_closed     = false;
static unsigned long  g_mg996_start_ms   = 0;
static int            g_mg996_target_us  = MG996_STOP;
static bool           g_mg996_closed_val = false;

/* PIR */
static bool           g_pir_active    = false;
static unsigned long  g_pir_last_ms   = 0;

/* Audio priority */
static int            g_audio_priority = 0;
static unsigned long  g_audio_end_ms   = 0;

/* Show-sequence state */
static bool           g_show_triggered  = false;
static bool           g_waiting_show_b  = false;
static unsigned long  g_show_b_time_ms  = 0;

/* Suppress-duplicate flags */
static bool g_close_sound_played = false;
static bool g_mute_close         = false;
static bool g_warn_playing       = false;

/* Physical button */
static bool           g_last_btn_state = HIGH;
static unsigned long  g_btn_debounce   = 0;
static int            g_btn_track      = 0;

/* Staff virtual button counter */
static int g_staff_track = 0;

/* Timing */
static unsigned long g_last_mg995_ms  = 0;
static unsigned long g_last_sensor_ms = 0;

/* Radar zones */
static zone_t g_zones[ZONE_COUNT];

/* ══════════════════════════════════════════════════════════════════════════
 *  FORWARD DECLARATIONS
 * ══════════════════════════════════════════════════════════════════════════ */
static void     audio_play(int track, int priority, unsigned long len_ms);
static bool     audio_is_playing(void);
static void     audio_check_end(void);
static void     mg996_start(int dir_us, bool closed_val);
static void     mg996_update(void);
static void     mg995_step(void);
static float    distance_measure(void);
static void     pir_update(void);
static void     zone_update(int idx, float dist);
static int      zone_check_confirmed(void);
static void     zones_reset_all(void);
static void     sensor_handle(void);
static void     button_handle(void);
static void     blynk_send_distance(void);
static bool     wifi_connected(void);

/* ══════════════════════════════════════════════════════════════════════════
 *  HELPERS – inline
 * ══════════════════════════════════════════════════════════════════════════ */

/** Convert current µs pulse width to logical angle (30°–170°). */
static inline int servo_us_to_angle(int us)
{
    return (int)(30.0f + (us - 817.0f) / (2294.0f - 817.0f) * 140.0f);
}

/** Map angle to radar zone index (clamped). */
static inline int angle_to_zone(int angle)
{
    int idx = (angle - ZONE_START_DEG) / ZONE_SIZE_DEG;
    if (idx < 0)           idx = 0;
    if (idx >= ZONE_COUNT) idx = ZONE_COUNT - 1;
    return idx;
}

static inline bool wifi_connected(void)
{
    return (WiFi.status() == WL_CONNECTED);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  AUDIO
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Play a DFPlayer track if its priority >= current audio priority.
 *
 * Priority levels:
 *   0 = show (auto-triggered background)
 *   1 = button / manual
 *   2 = staff override
 *   3 = danger / warning sensor
 */
static void audio_play(int track, int priority, unsigned long len_ms)
{
    if (priority < g_audio_priority) {
        Serial.printf("[DF] Skip track %d (cur_pri=%d)\n", track, g_audio_priority);
        return;
    }
    g_audio_priority = priority;
    g_audio_end_ms   = millis() + len_ms;
    g_player.play(track);
    Serial.printf("[DF] >>> Track %d (pri=%d, len=%lums)\n", track, priority, len_ms);
}

static bool audio_is_playing(void)
{
    return (millis() < g_audio_end_ms);
}

/** Called every loop to decay priority after playback ends. */
static void audio_check_end(void)
{
    if (g_audio_priority > 0 && !audio_is_playing()) {
        Serial.printf("[DF] Done (was pri=%d)\n", g_audio_priority);
        g_audio_priority = 0;
        g_warn_playing   = false;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  MG996R – non-blocking state machine  [F1]
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Request a non-blocking MG996R rotation.
 *        Returns immediately; call mg996_update() every loop.
 */
static void mg996_start(int dir_us, bool closed_val)
{
    if (g_mg996_state != MG996_IDLE) {
        Serial.println("[MG996] Busy, request ignored");
        return;
    }
    g_mg996_target_us  = dir_us;
    g_mg996_closed_val = closed_val;
    g_mg996_start_ms   = millis();
    g_mg996_state      = MG996_RUNNING;
    g_mg996.writeMicroseconds(dir_us);
    Serial.printf("[MG996] Start dir=%d, will run %lums\n", dir_us, MG996_TURN_MS);
}

/** Must be called every loop iteration. */
static void mg996_update(void)
{
    if (g_mg996_state != MG996_RUNNING) return;

    if (millis() - g_mg996_start_ms >= MG996_TURN_MS) {
        g_mg996.writeMicroseconds(MG996_STOP);
        g_mg996_closed = g_mg996_closed_val;
        g_mg996_state  = MG996_IDLE;
        Serial.printf("[MG996] Done, closed=%d\n", g_mg996_closed);
    }
}

static inline bool mg996_is_busy(void)
{
    return (g_mg996_state != MG996_IDLE);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  MG995 SWEEP
 * ══════════════════════════════════════════════════════════════════════════ */

static void mg995_step(void)
{
    g_mg995.writeMicroseconds(g_mg995_us);

    if (g_mg995_going_up) {
        g_mg995_us += MG995_STEP_US;
        if (g_mg995_us >= MG995_MAX_US) {
            g_mg995_us       = MG995_MAX_US;
            g_mg995_going_up = false;
        }
    } else {
        g_mg995_us -= MG995_STEP_US;
        if (g_mg995_us <= MG995_MIN_US) {
            g_mg995_us       = MG995_MIN_US;
            g_mg995_going_up = true;
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  HC-SR04 DISTANCE  [F2] – single measurement point
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Trigger HC-SR04 and return distance in cm, or -1 on timeout.
 *        Do NOT call from two places; use g_last_dist for secondary consumers.
 */
static float distance_measure(void)
{
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    long dur = pulseIn(PIN_ECHO, HIGH, 60000UL);
    if (dur == 0) return -1.0f;
    return dur * 0.0343f / 2.0f;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  PIR
 * ══════════════════════════════════════════════════════════════════════════ */

static void pir_update(void)
{
    if (digitalRead(PIN_PIR) == HIGH) {
        g_pir_active  = true;
        g_pir_last_ms = millis();
    } else if (millis() - g_pir_last_ms > PIR_TIMEOUT_MS) {
        if (g_pir_active) Serial.println("[PIR] Motion ended");
        g_pir_active = false;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  RADAR ZONE TRACKING
 * ══════════════════════════════════════════════════════════════════════════ */

static void zone_update(int idx, float dist)
{
    unsigned long now = millis();
    zone_t       *z   = &g_zones[idx];

    bool in_range = (dist > 0.0f) && (dist >= DIST_WARN) && (dist < DIST_SHOW);

    if (in_range) {
        if (!z->active) {
            z->active        = true;
            z->dist          = dist;
            z->first_seen_ms = now;
            z->last_seen_ms  = now;
            Serial.printf("[ZONE %d°] Object appeared: %.1fcm\n",
                          ZONE_START_DEG + idx * ZONE_SIZE_DEG, dist);
        } else {
            if (fabsf(dist - z->dist) <= DIST_TOL) {
                z->last_seen_ms = now;           /* stable – refresh timer */
            } else {
                z->dist          = dist;         /* moved – restart confirm */
                z->first_seen_ms = now;
                z->last_seen_ms  = now;
            }
        }
    } else {
        if (z->active) {
            Serial.printf("[ZONE %d°] Object left\n",
                          ZONE_START_DEG + idx * ZONE_SIZE_DEG);
        }
        z->active = false;
    }
}

/** @return Zone index confirmed for ZONE_CONFIRM_MS, or -1 if none. */
static int zone_check_confirmed(void)
{
    unsigned long now = millis();
    for (int i = 0; i < ZONE_COUNT; i++) {
        if (g_zones[i].active
            && (now - g_zones[i].first_seen_ms >= ZONE_CONFIRM_MS)
            && (now - g_zones[i].last_seen_ms  <  2000UL)) {
            return i;
        }
    }
    return -1;
}

/** Reset all zones and show-sequence state together.  [F3] */
static void zones_reset_all(void)
{
    for (int i = 0; i < ZONE_COUNT; i++) {
        g_zones[i].active = false;
    }
    g_show_triggered = false;
    g_waiting_show_b = false;   /* [F3] was missing in v3.2 */
}

/* ══════════════════════════════════════════════════════════════════════════
 *  MAIN SENSOR HANDLER
 * ══════════════════════════════════════════════════════════════════════════ */

static void sensor_handle(void)
{
    /* [F2] One measurement per cycle; result cached for Blynk timer too */
    float         dist = distance_measure();
    unsigned long now  = millis();
    g_last_dist        = dist;

    int angle    = servo_us_to_angle(g_mg995_us);
    int zone_idx = angle_to_zone(angle);

    Serial.printf("Dist=%.1fcm Angle=%d° Zone=%d PIR=%s\n",
                  dist, angle, zone_idx, g_pir_active ? "YES" : "NO");

    /* ── Object absent or too far ─────────────────────────────────── */
    if (dist < 0.0f || dist > DIST_SHOW) {
        g_close_sound_played = false;
        g_warn_playing       = false;       /* [F5] reset only when clear */
        zone_update(zone_idx, -1.0f);

        /* If no zone remains active, drop show state */
        bool any_active = false;
        for (int i = 0; i < ZONE_COUNT; i++) {
            if (g_zones[i].active) { any_active = true; break; }
        }
        if (!any_active) {
            g_show_triggered = false;
            g_waiting_show_b = false;
        }
        return;
    }

    /* ── DANGER zone: dist < DIST_DANGER ─────────────────────────── */
    if (dist < DIST_DANGER) {
        zones_reset_all();                  /* [F3] clears waitingShowB  */
        g_warn_playing = false;

        /*
         * An toàn là ưu tiên cao nhất:
         * Đóng tủ bất kể PIR có active hay không (vật thể lạ / lỗi PIR).
         * Spec: "<10cm + PIR=false → Vẫn đóng tủ"
         */
        if (!g_mg996_closed) {
            if (g_pir_active) {
                Serial.println("[SENSOR] <10cm + PIR=true : AUTO LOCK (intrusion)");
                if (wifi_connected()) {
                    Blynk.virtualWrite(VP_STATUS, "DONG TU BAO VE!");
                    Blynk.logEvent("intrusion_alert", "Tiep can nguy hiem!");
                }
            } else {
                Serial.println("[SENSOR] <10cm + PIR=false: AUTO LOCK (safety)");
                if (wifi_connected()) {
                    Blynk.virtualWrite(VP_STATUS, "DONG TU AN TOAN");
                }
            }
            mg996_start(MG996_CCW, true);
        }

        /*
         * Phát âm thanh TRACK_DANGER chỉ khi PIR=true (có người).
         * Khi PIR=false → vật thể tĩnh / gõ nhẹ → không phát âm.
         * [F4] Luôn set closeSoundPlayed kể cả khi mute để chặn vòng lặp.
         */
        if (g_pir_active && !g_close_sound_played) {
            g_close_sound_played = true;
            if (!g_mute_close) {
                audio_play(TRACK_DANGER, 3, LEN_DANGER_MS);
                Serial.println("[SENSOR] Playing TRACK_DANGER");
            }
        }
        return;
    }

    /* ── WARN zone: DIST_DANGER ≤ dist < DIST_WARN ───────────────── */
    if (dist < DIST_WARN) {
        g_close_sound_played = false;
        zone_update(zone_idx, -1.0f);       /* not in radar range */

        if (g_pir_active && !g_warn_playing && !audio_is_playing()) {
            g_warn_playing = true;
            audio_play(TRACK_WARN, 3, LEN_WARN_MS);
            Serial.println("[SENSOR] Playing TRACK_WARN");
            if (wifi_connected()) {
                Blynk.virtualWrite(VP_STATUS, "Canh bao: nguoi den gan!");
            }
        }
        return;
    }

    /* ── SHOW zone: DIST_WARN ≤ dist < DIST_SHOW ─────────────────── */
    if (g_pir_active) {
        g_close_sound_played = false;
        g_warn_playing       = false;

        zone_update(zone_idx, dist);

        if (!g_show_triggered && g_audio_priority < 2) {
            int confirmed = zone_check_confirmed();
            if (confirmed >= 0) {
                g_show_triggered = true;
                g_waiting_show_b = true;
                g_show_b_time_ms = now + LEN_SHOW_A_MS;
                audio_play(TRACK_SHOW_A, 0, LEN_SHOW_A_MS);
                Serial.printf("[RADAR] Zone %d° confirmed: SHOW_A → SHOW_B\n",
                              ZONE_START_DEG + confirmed * ZONE_SIZE_DEG);
                if (wifi_connected()) {
                    Blynk.virtualWrite(VP_STATUS, "Dang trinh bay...");
                }
            }
        }
    } else {
        zone_update(zone_idx, -1.0f);
        Serial.println("[SENSOR] Object present but PIR=false, skip");
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  PHYSICAL BUTTON
 * ══════════════════════════════════════════════════════════════════════════ */

static void button_handle(void)
{
    bool          state = digitalRead(PIN_BUTTON);
    unsigned long now   = millis();

    if (state == LOW && g_last_btn_state == HIGH
        && (now - g_btn_debounce) > 50UL) {

        g_btn_debounce    = now;
        g_last_btn_state  = LOW;

        if (g_audio_priority >= 2) {
            Serial.println("[BTN] Sensor active, ignored");
            return;
        }

        int  track = (g_btn_track % 2 == 0) ? TRACK_SHOW_A : TRACK_SHOW_B;
        unsigned long len = (track == TRACK_SHOW_A) ? LEN_SHOW_A_MS : LEN_SHOW_B_MS;
        audio_play(track, 1, len);
        Serial.printf("[BTN] Track %d\n", track);
        g_btn_track++;
    }

    if (state == HIGH) g_last_btn_state = HIGH;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  BLYNK TIMER CALLBACK  [F2] – reuses cached distance
 * ══════════════════════════════════════════════════════════════════════════ */

static void blynk_send_distance(void)
{
    if (g_last_dist > 0.0f) {
        Blynk.virtualWrite(VP_DISTANCE, g_last_dist);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  BLYNK VIRTUAL PIN HANDLERS
 * ══════════════════════════════════════════════════════════════════════════ */

BLYNK_WRITE(VP_OPEN)
{
    if (param.asInt() != 1) return;
    if (g_mg996_closed == false) { Blynk.virtualWrite(VP_STATUS, "Da mo roi");    return; }
    if (mg996_is_busy())         { Blynk.virtualWrite(VP_STATUS, "Dang quay..."); return; }
    Serial.println("[V2] Open cabinet");
    Blynk.virtualWrite(VP_STATUS, "Dang mo...");
    mg996_start(MG996_CW, false);
}

BLYNK_WRITE(VP_CLOSE)
{
    if (param.asInt() != 1) return;
    if (g_mg996_closed)  { Blynk.virtualWrite(VP_STATUS, "Da dong roi");   return; }
    if (mg996_is_busy()) { Blynk.virtualWrite(VP_STATUS, "Dang quay..."); return; }
    Serial.println("[V5] Close cabinet");
    Blynk.virtualWrite(VP_STATUS, "Dang dong...");
    mg996_start(MG996_CCW, true);
}

BLYNK_WRITE(VP_STAFF)
{
    if (param.asInt() != 1) return;
    if (g_audio_priority == 3) { Serial.println("[V3] Sensor active, ignored"); return; }

    int  track = (g_staff_track % 2 == 0) ? TRACK_SHOW_A : TRACK_SHOW_B;
    unsigned long len = (track == TRACK_SHOW_A) ? LEN_SHOW_A_MS : LEN_SHOW_B_MS;
    audio_play(track, 2, len);
    Serial.printf("[V3] Staff track %d\n", track);
    g_staff_track++;
}

BLYNK_WRITE(VP_MUTE)
{
    g_mute_close = (param.asInt() == 1);

    if (g_mute_close && g_audio_priority == 3) {
        g_player.stop();
        g_audio_priority     = 0;
        g_close_sound_played = false; /* allow re-arm after unmute */
    }

    Blynk.virtualWrite(VP_STATUS, g_mute_close ? "Am 0001: TAT" : "Am 0001: BAT");
    Serial.printf("[V4] Mute danger: %s\n", g_mute_close ? "ON" : "OFF");
}

/* ══════════════════════════════════════════════════════════════════════════
 *  SETUP
 * ══════════════════════════════════════════════════════════════════════════ */

void setup(void)
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== Smart Cabinet v4.0 ===");

    /* GPIO */
    pinMode(PIN_TRIG,   OUTPUT);
    pinMode(PIN_ECHO,   INPUT);
    pinMode(PIN_PIR,    INPUT);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    digitalWrite(PIN_TRIG, LOW);

    /* Zone array */
    for (int i = 0; i < ZONE_COUNT; i++) {
        g_zones[i] = (zone_t){ false, 0.0f, 0UL, 0UL };
    }

    /* Servos */
    g_mg996.attach(PIN_MG996);
    g_mg995.attach(PIN_MG995, 500, 2400);
    g_mg996.writeMicroseconds(MG996_STOP);
    g_mg995.writeMicroseconds(MG995_MIN_US);

    /* DFPlayer */
    g_df_serial.begin(9600);
    delay(1000);
    if (!g_player.begin(g_df_serial)) {
        Serial.println("[DF] Init FAILED");
    } else {
        Serial.println("[DF] Init OK");
        g_player.volume(10);
    }

    /* WiFi + Blynk */
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000UL) {
        delay(500);
        Serial.print('.');
    }
    Serial.println();

    if (wifi_connected()) {
        Serial.println("[WiFi] Connected");
        Blynk.config(BLYNK_AUTH_TOKEN);
        Blynk.connect();
        /* [F2] Blynk timer now sends cached value – no extra TRIG pulse */
        g_blynk_timer.setInterval(300L, blynk_send_distance);
    } else {
        Serial.println("[WiFi] Offline mode");
    }

    Serial.println("=== READY ===\n");
}

/* ══════════════════════════════════════════════════════════════════════════
 *  LOOP
 * ══════════════════════════════════════════════════════════════════════════ */

void loop(void)
{
    unsigned long now = millis();

    if (wifi_connected()) {
        Blynk.run();
        g_blynk_timer.run();
    }

    audio_check_end();
    mg996_update();         /* [F1] non-blocking servo tick */
    pir_update();

    /* MG995 sweep tick */
    if (now - g_last_mg995_ms >= MG995_STEP_MS) {
        g_last_mg995_ms = now;
        mg995_step();
    }

    /* Main sensor handler */
    if (now - g_last_sensor_ms >= SENSOR_PERIOD_MS) {
        g_last_sensor_ms = now;
        sensor_handle();
    }

    /* Deferred SHOW_B playback */
    if (g_waiting_show_b && now >= g_show_b_time_ms && !audio_is_playing()) {
        /* [F6] If a danger event raised priority to 3 during SHOW_A,
         *      skip SHOW_B entirely and let the operator re-trigger. */
        if (g_audio_priority < 2) {
            g_waiting_show_b = false;
            audio_play(TRACK_SHOW_B, 0, LEN_SHOW_B_MS);
            Serial.println("[SHOW] Playing SHOW_B");
        } else {
            /* Danger/staff track is still active – abort show sequence */
            g_waiting_show_b = false;
            g_show_triggered = false;
            Serial.println("[SHOW] SHOW_B aborted (priority override)");
        }
    }

    button_handle();
}
