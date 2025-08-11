// === ARM RECEIVER: 1 flex + IMU angles -> 4 servos ===
// Base<-roll, Shoulder<-pitch, Elbow<-pitch (scaled), Claw<-flex

#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

struct Payload { uint16_t flex; int16_t pitch10; int16_t roll10; uint8_t id; };

volatile Payload latest{};
volatile unsigned long lastPktMs = 0;

// ---- SERVO PINS (edit if needed) ----
const int SERVO_PINS[4] = {16, 2, 17, 4}; // base, shoulder, elbow, claw

// ---- JOINT SAFE RANGES (deg) ----
int JOINT_RANGE[4][2] = {
  { 10, 170 },  // base
  { 20, 155 },  // shoulder
  { 25, 165 },  // elbow
  { 30, 150 }   // claw
};

// ---- MAPS ----
// Flex raw range (put your straight/bent here from earlier)
const int FLEX_MIN = 1200;
const int FLEX_MAX = 2800;

// Pitch/roll range in degrees around neutral wrist pose
const float PITCH_MIN = -45, PITCH_MAX = +45; // wrist down/up
const float ROLL_MIN  = -60, ROLL_MAX  = +60; // twist left/right

// Per-joint scaling (0..1), and invert by swapping min/max in JOINT_RANGE
float SHOULDER_SCALE = 1.0f;  // how much pitch drives shoulder
float ELBOW_SCALE    = 0.7f;  // how much pitch drives elbow
float BASE_SCALE     = 1.0f;  // how much roll drives base

// ---- SMOOTHING ----
const float MAX_DEG_PER_SEC = 180.0f;
const float DEAD_BAND_DEG   = 0.7f;
const int   LOOP_HZ         = 100;
const float FAILSAFE_SEC    = 0.7f;

Servo servos[4];
float posDeg[4];
unsigned long lastUs = 0;
const unsigned long LOOP_US = 1000000UL / LOOP_HZ;

static inline float clampf(float v,float a,float b){ return v<a?a:(v>b?b:v); }
static inline float mapf(float x,float inMin,float inMax,float outMin,float outMax){
  if (inMax==inMin) return outMin;
  float t = (x-inMin)/(inMax-inMin); if(t<0)t=0; if(t>1)t=1;
  return outMin + t*(outMax-outMin);
}
static inline float easeFactor(float errDeg){
  float e=fabsf(errDeg), kmin=0.35f, span=1.0f-kmin;
  return (e>=30.0f)?1.0f:(kmin + (e/30.0f)*span);
}

// new ESP‑NOW signature
void onRecv(const esp_now_recv_info_t*, const uint8_t* data, int len){
  if (len == (int)sizeof(Payload)){
    memcpy((void*)&latest, data, sizeof(Payload));
    lastPktMs = millis();
  }
}

void setup(){
  Serial.begin(115200);
  delay(200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init()!=ESP_OK){ while(1){} }
  esp_now_register_recv_cb(onRecv);

  for(int i=0;i<4;i++){
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2400);
    posDeg[i] = (JOINT_RANGE[i][0] + JOINT_RANGE[i][1]) * 0.5f;
    servos[i].write(posDeg[i]);
  }
  lastUs = micros(); lastPktMs = millis();
  Serial.println("ARM receiver ready (flex+IMU).");
}

void loop(){
  // fixed‑rate
  unsigned long now = micros();
  if (now - lastUs < LOOP_US) return;
  float dt = (now - lastUs)/1e6f; lastUs = now;

  bool stale = (millis() - lastPktMs) > (unsigned long)(FAILSAFE_SEC*1000.0f);

  // unpack (make local copies)
  uint16_t flex = latest.flex;
  float pitch = latest.pitch10 / 10.0f;
  float roll  = latest.roll10  / 10.0f;

  // targets
  float target[4];
  if (!stale){
    // Base from roll
    float baseT = mapf(roll * BASE_SCALE,  ROLL_MIN,  ROLL_MAX,
                       JOINT_RANGE[0][0], JOINT_RANGE[0][1]);
    // Shoulder from pitch
    float shT   = mapf(pitch * SHOULDER_SCALE, PITCH_MIN, PITCH_MAX,
                       JOINT_RANGE[1][0], JOINT_RANGE[1][1]);
    // Elbow from pitch (scaled)
    float elT   = mapf(pitch * ELBOW_SCALE,    PITCH_MIN, PITCH_MAX,
                       JOINT_RANGE[2][0], JOINT_RANGE[2][1]);
    // Claw from flex
    int fmin = min(FLEX_MIN, FLEX_MAX), fmax = max(FLEX_MIN, FLEX_MAX);
    float clawT = mapf((float)flex, fmin, fmax,
                       JOINT_RANGE[3][0], JOINT_RANGE[3][1]);

    target[0]=baseT; target[1]=shT; target[2]=elT; target[3]=clawT;
  } else {
    // stale → center
    for(int i=0;i<4;i++) target[i] = 0.5f*(JOINT_RANGE[i][0]+JOINT_RANGE[i][1]);
  }

  // slew + write
  for(int i=0;i<4;i++){
    float jmin=JOINT_RANGE[i][0], jmax=JOINT_RANGE[i][1];
    float err = target[i] - posDeg[i];
    if (fabsf(err) > DEAD_BAND_DEG){
      float maxStep = MAX_DEG_PER_SEC * dt * easeFactor(err);
      posDeg[i] += (err>0 ? fminf(err, maxStep) : fmaxf(err, -maxStep));
      posDeg[i]  = clampf(posDeg[i], fmin(jmin,jmax), fmax(jmin,jmax));
      servos[i].write(posDeg[i]);
    }
  }

  // debug (every 300ms)
  static unsigned long last=0;
  if (millis()-last>300){
    last=millis();
    Serial.printf("[RECV] flex=%4u  pitch=%.1f  roll=%.1f  %s\n",
                  flex, pitch, roll, stale?"(failsafe)":"");
  }
}
