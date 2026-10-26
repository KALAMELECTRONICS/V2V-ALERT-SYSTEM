
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <string.h>

#define TRIG_PIN 32
#define ECHO_PIN 18
#define BUZZER_PIN 33

#define SDA_PIN 21
#define SCL_PIN 22

#define WARNING_DISTANCE_CM 300.0f
#define CRITICAL_DISTANCE_CM 150.0f

#define ALERT_TIMEOUT_MS 11500UL
#define DISTANCE_INTERVAL_MS 250UL

LiquidCrystal_I2C lcd(0x27, 16, 2);

uint8_t carAMAC[] = {
  0x30, 0x76, 0xF5, 0xE7, 0x47, 0x50
};

typedef struct {
  char message[32];
  int hazardType;
  unsigned long counter;
  float acceleration;
} HazardData;

HazardData receivedData;
HazardData latestMessage;

portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool newMessageAvailable = false;

bool hazardActive = false;
int currentHazardType = 0;

unsigned long lastMessageTime = 0;
unsigned long lastDistanceTime = 0;

float distanceCm = -1.0f;

char previousLine1[17] = "";
char previousLine2[17] = "";

void onReceive(const uint8_t *mac,
               const uint8_t *incomingData,
               int len) {
  if (len != sizeof(HazardData)) return;
  if (memcmp(mac, carAMAC, 6) != 0) return;

  HazardData tempData;
  memcpy(&tempData, incomingData, sizeof(tempData));

  portENTER_CRITICAL(&dataMux);
  receivedData = tempData;
  newMessageAvailable = true;
  portEXIT_CRITICAL(&dataMux);
}

void printLCD(const char *line1, const char *line2) {
  char newLine1[17] = {0};
  char newLine2[17] = {0};

  strncpy(newLine1, line1, 16);
  strncpy(newLine2, line2, 16);

  if (strncmp(previousLine1, newLine1, 16) != 0) {
    lcd.setCursor(0, 0);
    lcd.print(newLine1);

    int length = strlen(newLine1);
    for (int i = length; i < 16; i++) lcd.print(' ');

    memcpy(previousLine1, newLine1, 17);
  }

  if (strncmp(previousLine2, newLine2, 16) != 0) {
    lcd.setCursor(0, 1);
    lcd.print(newLine2);

    int length = strlen(newLine2);
    for (int i = length; i < 16; i++) lcd.print(' ');

    memcpy(previousLine2, newLine2, 17);
  }
}

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);

  if (duration == 0) return -1.0f;

  return duration * 0.0343f / 2.0f;
}

void updateDistance() {
  if (millis() - lastDistanceTime < DISTANCE_INTERVAL_MS) return;

  lastDistanceTime = millis();
  distanceCm = measureDistance();
}

void processReceivedMessage() {
  bool messageReady = false;

  portENTER_CRITICAL(&dataMux);

  if (newMessageAvailable) {
    latestMessage = receivedData;
    newMessageAvailable = false;
    messageReady = true;
  }

  portEXIT_CRITICAL(&dataMux);

  if (!messageReady) return;

  if (latestMessage.hazardType != 1 &&
      latestMessage.hazardType != 2) return;

  currentHazardType = latestMessage.hazardType;
  hazardActive = true;
  lastMessageTime = millis();

  Serial.print("[RX] ");
  Serial.println(latestMessage.message);
}

void handleAlertTimeout() {
  if (!hazardActive) return;

  if (millis() - lastMessageTime >= ALERT_TIMEOUT_MS) {
    hazardActive = false;
    currentHazardType = 0;

    Serial.println("[SYSTEM] Alert cleared.");
  }
}

void updateLCD() {
  char line1[17];
  char line2[17];

  if (hazardActive && currentHazardType == 1) {
    printLCD("EMERGENCY HELP", "Vehicle Alert!");
    return;
  }

  if (distanceCm < 0) {
    snprintf(line2, sizeof(line2), "Dist: NO ECHO");
  } else {
    int displayedDistance = (int)roundf(distanceCm);

    if (distanceCm <= CRITICAL_DISTANCE_CM) {
      snprintf(line2, sizeof(line2), "CRITICAL %dcm",
               displayedDistance);
    } else if (distanceCm <= WARNING_DISTANCE_CM) {
      snprintf(line2, sizeof(line2), "MEDIUM %dcm",
               displayedDistance);
    } else {
      snprintf(line2, sizeof(line2), "NORMAL %dcm",
               displayedDistance);
    }
  }

  if (hazardActive && currentHazardType == 2) {
    snprintf(line1, sizeof(line1), "HAZARD ALERT");
  } else {
    snprintf(line1, sizeof(line1), "V2V SYSTEM");
  }

  printLCD(line1, line2);
}

void updateBuzzer() {
  bool critical =
    hazardActive &&
    distanceCm >= 0 &&
    distanceCm <= CRITICAL_DISTANCE_CM;

  digitalWrite(BUZZER_PIN, critical ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);

  lcd.init();
  lcd.backlight();

  printLCD("V2V HAZARD", "WARNING SYSTEM");
  delay(2000);

  printLCD("Connecting...", "ESP-NOW");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println("V2V HAZARD WARNING SYSTEM - CAR B");

  Serial.print("Car B MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    printLCD("ESP-NOW ERROR", "Restart device");

    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onReceive);

  printLCD("V2V SYSTEM", "Waiting for Car A");

  Serial.println("[OK] ESP-NOW receiver ready.");
}

void loop() {
  updateDistance();
  processReceivedMessage();
  handleAlertTimeout();
  updateLCD();
  updateBuzzer();

  delay(10);
}
