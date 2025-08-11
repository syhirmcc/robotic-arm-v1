// === GLOVE SENDER: Flex + MPU6050 → ESP‑NOW ===
// Sends: flex (0..4095), pitch_deg, roll_deg to the ARM board.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const int FLEX_PIN = 32;          // flex junction ADC1
const float FLEX_ALPHA = 0.20f;   // flex low-pass
float flexFilt = 0;

// ---- IMU ----
Adafruit_MPU6050 mpu;
bool imuReady = false;
float pitch = 0, roll = 0;        // filtered degrees
unsigned long lastIMUms = 0;
const float CF_ALPHA = 0.98f;     // complementary filter gyro weight

struct Payload {
  uint16_t flex;      // 0..4095
  int16_t  pitch10;   // pitch * 10 (deg*10)
  int16_t  roll10;    // roll  * 10 (deg*10)
  uint8_t  id;        // reserved
} tx;

uint8_t ARM_MAC[6];
// PASTE YOUR ARM MAC (STA) HERE:
const char* ARM_MAC_STR = "F8:B3:B7:7F:1E:D0";

bool parseMac(const char* s, uint8_t out[6]){
  return 6==sscanf(s,"%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                   &out[0],&out[1],&out[2],&out[3],&out[4],&out[5]);
}

void onSend(const wifi_tx_info_t*, esp_now_send_status_t){}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== GLOVE SENDER: Flex + MPU6050 ===");

  // Flex ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(FLEX_PIN, ADC_11db);
  flexFilt = analogRead(FLEX_PIN);

  // IMU init (I2C on 21/22)
  Wire.begin(21,22);
  imuReady = mpu.begin();
  if (!imuReady) Serial.println("MPU6050 not found! Check wiring.");
  else {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    // seed angle with accel tilt
    sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t);
    float ax=a.acceleration.x, ay=a.acceleration.y, az=a.acceleration.z;
    float rollAcc  = atan2f(ay, az) * 180.0f/M_PI;
    float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f/M_PI;
    roll = rollAcc; pitch = pitchAcc;
    lastIMUms = millis();
  }

  // ESP‑NOW
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  // (Optional) lock RF channel if you also forced it on the receiver:
  // esp_wifi_set_promiscuous(true);
  // esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  // esp_wifi_set_promiscuous(false);

  if (!parseMac(ARM_MAC_STR, ARM_MAC)) { Serial.println("BAD ARM MAC"); while(1){} }
  if (esp_now_init()!=ESP_OK){ Serial.println("ESP‑NOW init FAILED"); while(1){} }
  esp_now_register_send_cb(onSend);

  esp_now_peer_info_t peer{}; memcpy(peer.peer_addr, ARM_MAC, 6);
  peer.channel = 0; peer.encrypt = false;
  if (esp_now_add_peer(&peer)!=ESP_OK){ Serial.println("Add peer FAILED"); while(1){} }

  Serial.println("GLOVE SENDER ready.");
}

void loop() {
  // Flex
  int raw = analogRead(FLEX_PIN);
  flexFilt += FLEX_ALPHA * (raw - flexFilt);

  // IMU
  if (imuReady){
    sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t);
    float ax=a.acceleration.x, ay=a.acceleration.y, az=a.acceleration.z;
    float gx=g.gyro.x * 180.0f/M_PI; // deg/s
    float gy=g.gyro.y * 180.0f/M_PI; // deg/s

    unsigned long now = millis();
    float dt = (now - lastIMUms) * 0.001f; if (dt < 0) dt = 0; lastIMUms = now;

    // integrate gyro
    pitch += gx * dt;
    roll  += gy * dt;

    // accel tilt
    float rollAcc  = atan2f(ay, az) * 180.0f/M_PI;
    float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f/M_PI;

    // complementary filter
    pitch = CF_ALPHA * pitch + (1.0f - CF_ALPHA) * pitchAcc;
    roll  = CF_ALPHA * roll  + (1.0f - CF_ALPHA) * rollAcc;
  }

  // pack + send
  tx.flex    = (uint16_t)constrain((int)lroundf(flexFilt), 0, 4095);
  tx.pitch10 = (int16_t)lroundf(pitch * 10.0f);
  tx.roll10  = (int16_t)lroundf(roll  * 10.0f);
  tx.id      = 0;

  esp_now_send(ARM_MAC, (uint8_t*)&tx, sizeof(tx));

  // debug
  static unsigned long last=0;
  if (millis()-last > 250) {
    last = millis();
    Serial.printf("[SEND] flex=%4u  pitch=%.1f  roll=%.1f\n", tx.flex, tx.pitch10/10.0f, tx.roll10/10.0f);
  }

  delay(15); // ~66 Hz
}
