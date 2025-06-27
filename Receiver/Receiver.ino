#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <WiFiManager.h>

#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define FORWARD_LEFT 5
#define FORWARD_RIGHT 6
#define BACKWARD_LEFT 7
#define BACKWARD_RIGHT 8
#define TURN_LEFT 9
#define TURN_RIGHT 10
#define STOP 0

#define BACK_RIGHT_MOTOR 0
#define BACK_LEFT_MOTOR 1
#define FRONT_RIGHT_MOTOR 2
#define FRONT_LEFT_MOTOR 3

#define MAX_MOTOR_SPEED 200
const int PWMFreq = 1000;
const int PWMResolution = 8;

#define SIGNAL_TIMEOUT 1000
#define GESTURE_OVERRIDE_TIMEOUT 2000

const int LED_PIN = 2; 

struct MOTOR_PINS {
  int pinIN1;
  int pinIN2;
  int pinEn;
  int pwmSpeedChannel;
};

std::vector<MOTOR_PINS> motorPins = {
  {16, 17, 22, 4},  // BACK_RIGHT_MOTOR
  {18, 19, 23, 5},  // BACK_LEFT_MOTOR
  {26, 27, 14, 6},  // FRONT_RIGHT_MOTOR
  {33, 25, 32, 7},  // FRONT_LEFT_MOTOR
};

unsigned long lastRecvTime = 0;
unsigned long lastGestureTime = 0;
bool signalLost = false;

struct PacketData {
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;
  byte command; 
  byte source;  // 11 = gesture, 12 = remote
};

PacketData receiverData;
int lastCommandType = -1;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.print("Data: source=");
  Serial.print(receiverData.source);
  Serial.print(", command=");
  Serial.println(receiverData.command);


  if (len != sizeof(receiverData)) {
    Serial.println("Packet size mismatch - discarded");
    return;
  }

  memcpy(&receiverData, incomingData, sizeof(receiverData));
  lastRecvTime = millis();
  signalLost = false;
  digitalWrite(LED_PIN, HIGH);

  if (receiverData.source == 11 && (lastCommandType != 12 || millis() - lastGestureTime > GESTURE_OVERRIDE_TIMEOUT))
  {
      lastCommandType = 11;
      lastGestureTime = millis();
      processGesture(receiverData);
    } else if (receiverData.source == 12) {
      lastCommandType = 12;
      processRemote(receiverData);
    }
  }

void processGesture(const PacketData &data) {
  const int DEADZONE_LOW = 100;
  const int DEADZONE_HIGH = 150;

  bool inXDeadZone = data.xAxisValue > DEADZONE_LOW && data.xAxisValue < DEADZONE_HIGH;
  bool inYDeadZone = data.yAxisValue > DEADZONE_LOW && data.yAxisValue < DEADZONE_HIGH;
  bool inZDeadZone = data.zAxisValue > DEADZONE_LOW && data.zAxisValue < DEADZONE_HIGH;

  // Priority: Forward/Backward → Left/Right → Turn → Stop
  if (!inYDeadZone) {
    if (data.yAxisValue < DEADZONE_LOW) {
      processCarMovement(FORWARD);
    } else if (data.yAxisValue > DEADZONE_HIGH) {
      processCarMovement(BACKWARD);
    }
  }
  else if (!inXDeadZone) {
    if (data.xAxisValue < DEADZONE_LOW) {
      processCarMovement(LEFT);
    } else if (data.xAxisValue > DEADZONE_HIGH) {
      processCarMovement(RIGHT);
    }
  }
  else if (!inZDeadZone) {
    if (data.zAxisValue < DEADZONE_LOW) {
      processCarMovement(TURN_LEFT);
    } else if (data.zAxisValue > DEADZONE_HIGH) {
      processCarMovement(TURN_RIGHT);
    }
  }
  else {
    processCarMovement(STOP);
  }
}

void processRemote(const PacketData &data) {
  Serial.print("📥 Remote Command Received: ");
  Serial.println(data.command);

  switch (data.command) {
    case 0:  processCarMovementFromRemote(STOP);             break;
    case 1:  processCarMovementFromRemote(FORWARD);          break;
    case 2:  processCarMovementFromRemote(BACKWARD);         break;
    case 3:  processCarMovementFromRemote(LEFT);             break;
    case 4:  processCarMovementFromRemote(RIGHT);            break;
    case 5:  processCarMovementFromRemote(FORWARD_LEFT);     break;
    case 6:  processCarMovementFromRemote(FORWARD_RIGHT);    break;
    case 7:  processCarMovementFromRemote(BACKWARD_LEFT);    break;
    case 8: processCarMovementFromRemote(BACKWARD_RIGHT);    break;
    case 9:  processCarMovementFromRemote(TURN_LEFT);        break;
    case 10:  processCarMovementFromRemote(TURN_RIGHT);      break;
    default:
      Serial.println("Invalid remote command value");
      processCarMovement(STOP);
      break;
  }
}

void stopMotor(int motorNumber) {
  digitalWrite(motorPins[motorNumber].pinIN1, LOW);
  digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  ledcWrite(motorPins[motorNumber].pwmSpeedChannel, 0);
}

void stopAllMotors() {
  for (int i = 0; i < motorPins.size(); i++) {
    stopMotor(i);
  }
}

//Car movement processing from Remote
void processCarMovementFromRemote(int command) {
  Serial.print("Executing Movement from Remote: ");
  switch (command) {
    case FORWARD:
      Serial.println("FORWARD");
      rotateMotorWithSpeed(FRONT_RIGHT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_RIGHT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(FRONT_LEFT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_LEFT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      break;

    case BACKWARD:
      Serial.println("BACKWARD");
      rotateMotorWithSpeed(FRONT_RIGHT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_RIGHT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(FRONT_LEFT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_LEFT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      break;

    case LEFT:
      Serial.println("LEFT");
      rotateMotorWithSpeed(FRONT_RIGHT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_RIGHT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(FRONT_LEFT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_LEFT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      break;

    case RIGHT:
      Serial.println("RIGHT");
      rotateMotorWithSpeed(FRONT_RIGHT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_RIGHT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(FRONT_LEFT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_LEFT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      break;

    case FORWARD_LEFT:
      Serial.println("FORWARD_LEFT");
      rotateMotorWithSpeed(FRONT_RIGHT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_RIGHT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(FRONT_LEFT_MOTOR, FORWARD, MAX_MOTOR_SPEED / 2);
      rotateMotorWithSpeed(BACK_LEFT_MOTOR, FORWARD, MAX_MOTOR_SPEED / 2);
      break;

    case FORWARD_RIGHT:
      Serial.println("FORWARD_RIGHT");
      rotateMotorWithSpeed(FRONT_LEFT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_LEFT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(FRONT_RIGHT_MOTOR, FORWARD, MAX_MOTOR_SPEED / 2);
      rotateMotorWithSpeed(BACK_RIGHT_MOTOR, FORWARD, MAX_MOTOR_SPEED / 2);
      break;

    case BACKWARD_LEFT:
      Serial.println("BACKWARD_LEFT");
      rotateMotorWithSpeed(FRONT_LEFT_MOTOR, BACKWARD, MAX_MOTOR_SPEED / 2);
      rotateMotorWithSpeed(BACK_LEFT_MOTOR, BACKWARD, MAX_MOTOR_SPEED / 2);
      rotateMotorWithSpeed(FRONT_RIGHT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_RIGHT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      break;

    case BACKWARD_RIGHT:
      Serial.println("BACKWARD_RIGHT");
      rotateMotorWithSpeed(FRONT_LEFT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_LEFT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(FRONT_RIGHT_MOTOR, BACKWARD, MAX_MOTOR_SPEED / 2);
      rotateMotorWithSpeed(BACK_RIGHT_MOTOR, BACKWARD, MAX_MOTOR_SPEED / 2);
      break;
      
    case TURN_LEFT:
      Serial.println("TURN_LEFT (In Place)");
      rotateMotorWithSpeed(FRONT_RIGHT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_RIGHT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(FRONT_LEFT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_LEFT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      break;

    case TURN_RIGHT:
      Serial.println("TURN_RIGHT (In Place)");
      rotateMotorWithSpeed(FRONT_RIGHT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_RIGHT_MOTOR, BACKWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(FRONT_LEFT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      rotateMotorWithSpeed(BACK_LEFT_MOTOR, FORWARD, MAX_MOTOR_SPEED);
      break;
    case STOP:
    default:
      Serial.println("STOP");
      stopAllMotors();
      break;
  }
}

void rotateMotorWithSpeed(int motorNumber, int direction, int speed) {
  if (direction == FORWARD) {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  } else if (direction == BACKWARD) {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);
  } else {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
    speed = 0;
  }
  ledcWrite(motorPins[motorNumber].pwmSpeedChannel, abs(speed));
}

//Car movement processing from Remote
void processCarMovement(int inputValue) {
  Serial.print("🚗 Executing Movement from Gesture: ");
  switch (inputValue) {
    case FORWARD:
      Serial.println("FORWARD");
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);
      break;
    case BACKWARD:
      Serial.println("BACKWARD");
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      break;
    case LEFT:
      Serial.println("LEFT");
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);
      break;
    case RIGHT:
      Serial.println("RIGHT");
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      break;
    case FORWARD_LEFT:
      Serial.println("FORWARD_LEFT (Curve)");
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);
      break;
    case FORWARD_RIGHT:
      Serial.println("FORWARD_RIGHT (Curve)");
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, STOP);
      break;
    case BACKWARD_LEFT:
      Serial.println("BACKWARD_LEFT (Curve)");
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, STOP);
      break;
    case BACKWARD_RIGHT:
      Serial.println("BACKWARD_RIGHT (Curve)");
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      break;
    case TURN_LEFT:
      Serial.println("TURN_LEFT (In Place)");
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      break;
    case TURN_RIGHT:
      Serial.println("TURN_RIGHT (In Place)");
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);
      break;
    case STOP:
    default:
      Serial.println("STOP");
      stopAllMotors();
      break;
  }
}

void rotateMotor(int motorNumber, int motorSpeed) {
  if (motorSpeed == 0) {
    stopMotor(motorNumber);
    return;
  }

  digitalWrite(motorPins[motorNumber].pinIN1, motorSpeed > 0 ? HIGH : LOW);
  digitalWrite(motorPins[motorNumber].pinIN2, motorSpeed > 0 ? LOW : HIGH);
  ledcWrite(motorPins[motorNumber].pwmSpeedChannel, abs(motorSpeed));
}

void setUpPinModes() {
  for (int i = 0; i < motorPins.size(); i++) {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);
    ledcSetup(motorPins[i].pwmSpeedChannel, PWMFreq, PWMResolution);
    ledcAttachPin(motorPins[i].pinEn, motorPins[i].pwmSpeedChannel);
    rotateMotor(i, STOP);
  }
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void setup() {
  setUpPinModes();
  Serial.begin(115200);

  // Setup WiFiManager
  WiFi.mode(WIFI_STA);  
  WiFiManager wm;
  // wm.resetSettings();

  if (!wm.autoConnect("ESP-Receiver-Setup")) {
    Serial.println("Failed to connect or setup portal");
    ESP.restart();
  }
  Serial.println("Connected to WiFi");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("✅ ESP-NOW Receiver Ready");
}

void loop() {
  unsigned long now = millis();

  if (!signalLost && now - lastRecvTime > SIGNAL_TIMEOUT) {
    Serial.println("No signal - STOP");
    processCarMovement(STOP);
    signalLost = true;
    digitalWrite(LED_PIN, LOW);
  }

  // Allow remote to resume after gesture override timeout
  if (lastCommandType == 11 && now - lastGestureTime > GESTURE_OVERRIDE_TIMEOUT) {
    lastCommandType = -1;
  }

  // Allow new source
  if (lastCommandType != -1 && now - lastRecvTime > SIGNAL_TIMEOUT) {
    lastCommandType = -1; 
  }

}
