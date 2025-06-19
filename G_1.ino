#include <esp_now.h>
#include <WiFi.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

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
const unsigned long calibrationHoldTime = 2000; // 2 seconds

const int controlButtonPin = 0;  // Nút BOOT
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
  // Optional: add LED blink here too
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
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("Success: Initialized ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 1;
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

  // Calibration check
  if (buttonState == LOW) {
    if (!buttonHeldForCalibration) {
      buttonPressStartTime = millis();
      buttonHeldForCalibration = true;
    } else if (millis() - buttonPressStartTime > calibrationHoldTime) {
      calibrateOffsets(); // perform calibration
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

    // Apply calibration offset
    float correctedX = rawX - offsetX;
    float correctedY = rawY - offsetY;
    float correctedZ = rawZ - offsetZ;

    // Smoothing
    static float smoothX = correctedX;
    static float smoothY = correctedY;
    static float smoothZ = correctedZ;

    const float alpha = 0.85;
    smoothX = alpha * smoothX + (1 - alpha) * correctedX;
    smoothY = alpha * smoothY + (1 - alpha) * correctedY;
    smoothZ = alpha * smoothZ + (1 - alpha) * correctedZ;

    // Deadzone
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
    digitalWrite(2, LOW);

    Serial.print("Sent: ");
    Serial.print((int)smoothX); Serial.print(" | ");
    Serial.print((int)smoothY); Serial.print(" | ");
    Serial.println((int)smoothZ);

    delay(10);
  }
}

