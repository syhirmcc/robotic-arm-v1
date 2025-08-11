// === ESP‑NOW ARM RECEIVER ===
// Receives one flex value and drives one servo on IO16 with smoothing + timeout failsafe.

#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

struct Payload { uint16_t flex; uint8_t id; };

volatile uint16_t latestFlex = 0;
volatile unsigned long lastPktMs = 0;

// --- SERVO PIN ---
const int SERVO_PIN = 16;

// --- MAP (tune these to your straight/bent) ---
const int RAW_MIN = 1200;   // flex value when finger straight
const int RAW_MAX = 2800;   // flex value when finger bent

const int ANG_MIN = 20;     // safe servo min
const int ANG_MAX = 160;    // safe servo max
const float FAILSAFE_SEC = 0.7f;  // go to center if no packets

// --- SMOOTHING ---
const float MAX_DEG_PER_SEC = 180.0f;
const float DEAD_BAND_DEG   = 0.7f;
const int   LOOP_HZ         = 100;

Servo s;
float posDeg = (ANG_MIN + ANG_MAX) * 0.5f;
unsigned long lastUs = 0;
const unsigned long LOOP_US = 1000000UL / LOOP_HZ;

static inline float clampf(float v,float a,float b){ return v<a?a:(v>b?b:v); }
static inline float easeFactor(float errDeg){
  float e=fabsf(errDeg), kmin=0.35f, span=1.0f-kmin;
  return (e>=30.0f)?1.0f:(kmin + (e/30.0f)*span);
}

// New callback signature (ESP32 core v3+ / IDF v5+)
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len){
  if (len == (int)sizeof(Payload)){
    Payload p; memcpy(&p, data, sizeof(p));
    latestFlex = p.flex;
    lastPktMs  = millis();
  }
}

void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ESP‑NOW ARM RECEIVER ===");
  WiFi.mode(WIFI_STA);
  Serial.print("ARM MAC: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK){ Serial.println("ESP‑NOW init FAILED"); while(1){} }
  esp_now_register_recv_cb(onRecv);

  s.setPeriodHertz(50);
  s.attach(SERVO_PIN, 500, 2400);
  s.write(posDeg);

  lastUs = micros();
  lastPktMs = millis();
  Serial.println("ARM RECEIVER ready. Waiting for packets...");
}

void loop(){
  // fixed‑rate loop
  unsigned long now = micros();
  if (now - lastUs < LOOP_US) return;
  float dt = (now - lastUs)/1e6f;
  lastUs = now;

  // Packet timeout → gently return to center
  bool stale = (millis() - lastPktMs) > (unsigned long)(FAILSAFE_SEC*1000.0f);

  float target;
  if (!stale){
    // map flex → angle
    int rmin = min(RAW_MIN, RAW_MAX), rmax = max(RAW_MIN, RAW_MAX);
    float t = (rmax==rmin) ? 0.0f : ((float)latestFlex - rmin)/(float)(rmax-rmin);
    t = clampf(t, 0.0f, 1.0f);
    target = ANG_MIN + t*(ANG_MAX - ANG_MIN);
  } else {
    target = (ANG_MIN + ANG_MAX) * 0.5f; // center on fail
  }

  // slew‑limit + easing
  float err = target - posDeg;
  if (fabsf(err) > DEAD_BAND_DEG){
    float maxStep = MAX_DEG_PER_SEC * dt * easeFactor(err);
    posDeg += (err>0 ? fminf(err, maxStep) : fmaxf(err, -maxStep));
    posDeg = clampf(posDeg, ANG_MIN, ANG_MAX);
    s.write(posDeg);
  }

  static unsigned long lastPrint=0;
  if (millis()-lastPrint > 250){
    lastPrint = millis();
    Serial.printf("flex:%4u  tgt:%5.1f  pos:%5.1f  %s\n",
                  latestFlex, target, posDeg, stale?"(failsafe)":"");
  }
}
