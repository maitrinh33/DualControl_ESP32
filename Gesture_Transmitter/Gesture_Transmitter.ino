#include <esp_now.h>
#include <WiFi.h>
#include <WiFiManager.h> 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

QueueHandle_t logQueue;
TaskHandle_t firebaseLoggerTaskHandle;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

const char* FIREBASE_HOST = "https://dualcontrol-9a925-default-rtdb.asia-southeast1.firebasedatabase.app/";
const char* FIREBASE_AUTH = ""; // Leave blank if database rules allow public write for testing

MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

float offsetX = 0, offsetY = 0, offsetZ = 0;
unsigned long buttonPressStartTime = 0;
bool buttonHeldForCalibration = false;
const unsigned long calibrationHoldTime = 2000; 

const int controlButtonPin = 0;  // BOOT button
bool controlEnabled = true;
bool lastButtonState = HIGH;

uint8_t receiverMacAddress[] = {0xA0, 0xB7, 0x65, 0x05, 0x75, 0x9C};

struct PacketData {
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;
  byte command;
  byte source;
};
PacketData data;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional: add LED blink 
}

void uploadToFirebase(const PacketData& packet) {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  String url = String(FIREBASE_HOST) + "/gesture_logs.json";

  time_t now = time(nullptr);
  struct tm* timeinfo = gmtime(&now);

  char timestamp[30];
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", timeinfo);

  String json = "{";
  json += "\"xAxisValue\":" + String(packet.xAxisValue) + ",";
  json += "\"yAxisValue\":" + String(packet.yAxisValue) + ",";
  json += "\"zAxisValue\":" + String(packet.zAxisValue) + ",";
  json += "\"command\":" + String(packet.command) + ",";
  json += "\"source\":" + String(packet.source) + ",";
  json += "\"timestamp\":\"" + String(timestamp) + "\"";
  json += "}";

  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");

  int code = http.POST(json);
  Serial.print("📤 Firebase upload status: ");
  Serial.println(code);

  http.end();
}

void firebaseLoggerTask(void* param) {
  PacketData packet;

  while (true) {
    if (xQueueReceive(logQueue, &packet, portMAX_DELAY)) {
      uploadToFirebase(packet); 
    }
  }
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFiManager wm;

  wm.setConfigPortalTimeout(180);
  // wm.resetSettings();
  
  Serial.println("📶 Starting WiFiManager portal...");
  Serial.println("📲 Connect to 'GestureSetup' AP and enter Wi-Fi credentials");

  if (!wm.autoConnect("GestureSetup")) {
    Serial.println("❌ Failed to connect and timeout expired");
    ESP.restart();
    delay(1000);
  }

  Serial.println("✅ WiFi connected via WiFiManager");
}


void setupMPU() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #endif

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    dmpReady = true;
  }
}

void calibrateOffsets() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    offsetX = ypr[0] * 180 / M_PI;
    offsetY = ypr[1] * 180 / M_PI;
    offsetZ = ypr[2] * 180 / M_PI;

    Serial.println("✅ Calibrated to current neutral position:");
    Serial.print("Offset X: "); Serial.println(offsetX);
    Serial.print("Offset Y: "); Serial.println(offsetY);
    Serial.print("Offset Z: "); Serial.println(offsetZ);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.disconnect();

  setupWiFi();

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("⏳ Waiting for NTP time");
  while (time(nullptr) < 100000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Time synced");

  logQueue = xQueueCreate(10, sizeof(PacketData)); 
  if (logQueue == NULL) {
    Serial.println("❌ Failed to create log queue");
  }

  xTaskCreatePinnedToCore(
    firebaseLoggerTask,    
    "FirebaseLogger",      
    4096,                  
    NULL,                  
    1,                     
    &firebaseLoggerTaskHandle, 
    0                      // Core 0 (Core 1 is used by loop())
  );


  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("Success: Initialized ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = WiFi.channel();  
  peerInfo.encrypt = false;


  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  } else {
    Serial.println("Success: Added peer");
  }

  setupMPU();
  pinMode(2, OUTPUT);
  pinMode(controlButtonPin, INPUT_PULLUP);

}

void loop() {
  bool buttonState = digitalRead(controlButtonPin);

  if (buttonState == LOW) {
    if (!buttonHeldForCalibration) {
      buttonPressStartTime = millis();
      buttonHeldForCalibration = true;
    } else if (millis() - buttonPressStartTime > calibrationHoldTime) {
      calibrateOffsets(); 
      buttonHeldForCalibration = false;
    }
  } else {
    buttonHeldForCalibration = false;
  }

  // Toggle control with button press (short press)
  if (buttonState == LOW && lastButtonState == HIGH && millis() - buttonPressStartTime < 1000) {
    delay(50);  // debounce
    controlEnabled = !controlEnabled;
    Serial.println(controlEnabled ? "▶ Điều khiển BẬT" : "⏸ Điều khiển TẮT");
  }
  lastButtonState = buttonState;

  if (!dmpReady || !controlEnabled) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float rawX = ypr[0] * 180 / M_PI;
    float rawY = ypr[1] * 180 / M_PI;
    float rawZ = ypr[2] * 180 / M_PI;

    float correctedX = rawX - offsetX;
    float correctedY = rawY - offsetY;
    float correctedZ = rawZ - offsetZ;

    static float smoothX = correctedX;
    static float smoothY = correctedY;
    static float smoothZ = correctedZ;

    const float alpha = 0.85;
    smoothX = alpha * smoothX + (1 - alpha) * correctedX;
    smoothY = alpha * smoothY + (1 - alpha) * correctedY;
    smoothZ = alpha * smoothZ + (1 - alpha) * correctedZ;

    if (abs(smoothX) < 5) smoothX = 0;
    if (abs(smoothY) < 5) smoothY = 0;
    if (abs(smoothZ) < 5) smoothZ = 0;

    data.xAxisValue = map(constrain(smoothX, -90, 90), -90, 90, 0, 254);
    data.yAxisValue = map(constrain(smoothY, -90, 90), -90, 90, 0, 254);
    data.zAxisValue = map(constrain(smoothZ, -90, 90), -90, 90, 0, 254);
    data.command = 0;
    data.source = 11;

    digitalWrite(2, HIGH);
    esp_now_send(receiverMacAddress, (uint8_t *) &data, sizeof(data));
    xQueueSend(logQueue, &data, 0);
    digitalWrite(2, LOW);

    Serial.print("Sent: ");
    Serial.print((int)smoothX); Serial.print(" | ");
    Serial.print((int)smoothY); Serial.print(" | ");
    Serial.println((int)smoothZ);

    delay(10);
  }
}

