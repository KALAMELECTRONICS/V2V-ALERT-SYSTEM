
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <math.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define TOUCH_PIN 27
#define MPU_ADDR 0x68

#define ALERT_DURATION_MS 10000UL
#define SEND_INTERVAL_MS 500UL
#define SENSOR_INTERVAL_MS 20UL
#define ROTATION_THRESHOLD_DPS 150.0f
#define ROTATION_REARM_DPS 80.0f
#define REARM_TIME_MS 500UL

uint8_t receiverMAC[] = {0xB4, 0xBF, 0xE9, 0x0D, 0xFF, 0x04};

typedef struct {
  char message[32];
  int hazardType;
  unsigned long counter;
  float acceleration;
} HazardData;

HazardData txData;

bool espNowReady = false;
bool alertActive = false;
bool previousTouchState = false;
bool rotationArmed = true;

int activeHazardType = 0;

unsigned long alertStartTime = 0;
unsigned long lastSendTime = 0;
unsigned long lastSensorTime = 0;
unsigned long belowThresholdSince = 0;
unsigned long messageCounter = 0;

float latestAcceleration = 0.0f;
float rotationMagnitude = 0.0f;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) return false;

  int count = Wire.requestFrom((int)MPU_ADDR, (int)len);

  if (count != len) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    data[i] = Wire.read();
  }

  return true;
}

int16_t makeInt16(uint8_t highByte, uint8_t lowByte) {
  return (int16_t)(((uint16_t)highByte << 8) | lowByte);
}

bool initializeMPU6050() {
  uint8_t id = 0;
  uint8_t data[14];

  if (!readRegisters(0x75, &id, 1)) {
    Serial.println("[ERROR] Cannot read sensor identity.");
    return false;
  }

  Serial.print("[INFO] WHO_AM_I = 0x");
  Serial.println(id, HEX);

  if (!writeRegister(0x6B, 0x00)) return false;
  delay(100);

  if (!writeRegister(0x1C, 0x10)) return false;
  if (!writeRegister(0x1B, 0x08)) return false;
  if (!writeRegister(0x1A, 0x03)) return false;

  delay(100);

  if (!readRegisters(0x3B, data, 14)) return false;

  Serial.println("[OK] Sensor initialized.");
  return true;
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ?
                 "[ESP-NOW] SEND SUCCESS" :
                 "[ESP-NOW] SEND FAILED");
}

void sendHazardMessage(const char *message, int type) {
  memset(&txData, 0, sizeof(txData));

  snprintf(txData.message, sizeof(txData.message), "%s", message);

  txData.hazardType = type;
  txData.counter = ++messageCounter;
  txData.acceleration = latestAcceleration;

  esp_err_t result = esp_now_send(
    receiverMAC,
    (uint8_t *)&txData,
    sizeof(txData)
  );

  Serial.print("[TX] ");
  Serial.print(message);
  Serial.print(" | Result: ");
  Serial.println(result == ESP_OK ? "QUEUED" : "ERROR");
}

void startAlert(const char *message, int type) {
  alertActive = true;
  activeHazardType = type;
  alertStartTime = millis();
  lastSendTime = millis();

  Serial.print("[HAZARD] ");
  Serial.println(message);

  sendHazardMessage(message, type);
}

bool readMPUData(float &gyroDps) {
  uint8_t data[14];

  if (!readRegisters(0x3B, data, 14)) return false;

  int16_t ax = makeInt16(data[0], data[1]);
  int16_t ay = makeInt16(data[2], data[3]);
  int16_t az = makeInt16(data[4], data[5]);

  int16_t gx = makeInt16(data[8], data[9]);
  int16_t gy = makeInt16(data[10], data[11]);
  int16_t gz = makeInt16(data[12], data[13]);

  float ax_g = ax / 4096.0f;
  float ay_g = ay / 4096.0f;
  float az_g = az / 4096.0f;

  latestAcceleration = sqrtf(
    ax_g * ax_g + ay_g * ay_g + az_g * az_g
  ) * 9.80665f;

  float gxDps = gx / 65.5f;
  float gyDps = gy / 65.5f;
  float gzDps = gz / 65.5f;

  gyroDps = sqrtf(
    gxDps * gxDps +
    gyDps * gyDps +
    gzDps * gzDps
  );

  return true;
}

void checkRotation() {
  if (millis() - lastSensorTime < SENSOR_INTERVAL_MS) return;

  lastSensorTime = millis();

  if (!readMPUData(rotationMagnitude)) {
    Serial.println("[ERROR] Sensor read failed.");
    return;
  }

  if (!rotationArmed) {
    if (rotationMagnitude < ROTATION_REARM_DPS) {
      if (belowThresholdSince == 0) {
        belowThresholdSince = millis();
      }

      if (millis() - belowThresholdSince >= REARM_TIME_MS) {
        rotationArmed = true;
        belowThresholdSince = 0;
      }
    } else {
      belowThresholdSince = 0;
    }
  }

  if (rotationArmed &&
      rotationMagnitude >= ROTATION_THRESHOLD_DPS) {

    rotationArmed = false;
    belowThresholdSince = 0;

    Serial.print("[ROTATION] ");
    Serial.println(rotationMagnitude);

    if (!alertActive) {
      startAlert("ROTATION ALERT", 2);
    }
  }
}

void checkTouchSensor() {
  bool touchNow = digitalRead(TOUCH_PIN) == HIGH;

  if (touchNow && !previousTouchState) {
    startAlert("EMERGENCY ALERT", 1);
  }

  previousTouchState = touchNow;
}

void handleAlertTimer() {
  if (!alertActive) return;

  if (millis() - alertStartTime >= ALERT_DURATION_MS) {
    alertActive = false;
    activeHazardType = 0;

    Serial.println("[SYSTEM] NORMAL");
    return;
  }

  if (millis() - lastSendTime >= SEND_INTERVAL_MS) {
    if (activeHazardType == 1) {
      sendHazardMessage("EMERGENCY ALERT", 1);
    } else if (activeHazardType == 2) {
      sendHazardMessage("ROTATION ALERT", 2);
    }

    lastSendTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(TOUCH_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("V2V HAZARD SYSTEM - CAR A");

  if (!initializeMPU6050()) {
    Serial.println("[ERROR] Sensor initialization failed.");

    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("[INFO] Car A MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW initialization failed.");

    while (true) delay(1000);
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ERROR] Failed to add Car B.");

    while (true) delay(1000);
  }

  espNowReady = true;

  Serial.println("[OK] ESP-NOW ready.");
  Serial.println("[OK] Touch sensor ready.");
  Serial.println("[OK] Rotation detection ready.");
  Serial.println("[SYSTEM] NORMAL");
}

void loop() {
  if (!espNowReady) {
    delay(100);
    return;
  }

  checkTouchSensor();
  checkRotation();
  handleAlertTimer();

  delay(2);
}
