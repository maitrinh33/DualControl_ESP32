#include "esp_camera.h"
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>
#include <iostream>
#include <sstream>
#include <esp_timer.h>
#include <esp_now.h>

#define LIGHT_PIN 4

const int PWMFreq = 1000; 
const int PWMResolution = 8;
const int PWMLightChannel = 3;

//Camera related constants
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

enum command { 
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4,
  FORWARD_LEFT = 5,
  FORWARD_RIGHT = 6,
  BACKWARD_LEFT = 7,
  BACKWARD_RIGHT = 8,
  TURN_LEFT = 9,
  TURN_RIGHT = 10
};

AsyncWebServer server(80);
AsyncWebSocket wsCamera("/Camera");
AsyncWebSocket wsCarInput("/CarInput");

uint32_t cameraClientId = 0;
uint8_t receiverMacAddress[] = { 0xA0, 0xB7, 0x65, 0x05, 0x75, 0x9C };

DNSServer dns;
AsyncWiFiManager wifiManager(&server, &dns); 

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    wifiManager.setConfigPortalTimeout(180); 
  // wifiManager.resetSettings();

  if (!wifiManager.autoConnect("CameraSetup", "12345678")) {
    Serial.println("Failed to connect. Rebooting...");
    delay(2000);
    ESP.restart();
  }

  Serial.println("✅ WiFi connected");
  Serial.print("📶 IP Address: ");
  Serial.println(WiFi.localIP());
}

typedef struct {
  uint8_t xAxisValue;   
  uint8_t yAxisValue;   
  uint8_t zAxisValue;   
  uint8_t command;      
  uint8_t source;       // 11 = gesture , 12 = remote
} PacketData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void initEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = WiFi.channel();  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  Serial.println("ESP-NOW Initialized and peer added");
}


const char *htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html>
  <head>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
    <style>
    .arrows {
      font-size:40px;
      color:pink;
    }
    .circularArrows {
      font-size:50px;
      color:blue;
    }
    td.button {
      background-color:black;
      border-radius:50%;
      box-shadow: 5px 5px #888888;
    }
    td.button:active {
      transform: translate(5px,5px);
      box-shadow: none; 
    }

    .noselect {
      -webkit-touch-callout: none; /* iOS Safari */
        -webkit-user-select: none; /* Safari */
         -khtml-user-select: none; /* Konqueror HTML */
           -moz-user-select: none; /* Firefox */
            -ms-user-select: none; /* Internet Explorer/Edge */
                user-select: none; /* Non-prefixed version, currently
                                      supported by Chrome and Opera */
    }

    .slidecontainer {
      width: 100%;
    }

    .slider {
      -webkit-appearance: none;
      width: 100%;
      height: 15px;
      border-radius: 5px;
      background: #d3d3d3;
      outline: none;
      opacity: 0.7;
      -webkit-transition: .2s;
      transition: opacity .2s;
    }

    .slider:hover {
      opacity: 1;
    }
  
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 25px;
      height: 25px;
      border-radius: 50%;
      background: red;
      cursor: pointer;
    }

    .slider::-moz-range-thumb {
      width: 25px;
      height: 25px;
      border-radius: 50%;
      background: red;
      cursor: pointer;
    }

    </style>
  
  </head>
  <body class="noselect" align="center" style="background-color:white">
     
    <!--h2 style="color: teal;text-align:center;">Wi-Fi Camera &#128663; Control</h2-->
    
    <table id="mainTable" style="width:400px;margin:auto;table-layout:fixed" CELLSPACING=10>
      <tr>
        <td colspan="3">
          <img id="cameraImage" src="" style="width:400px;height:250px">
        </td>
      </tr>
      <tr>
        <td class="button" ontouchstart='startMovement("5")' ontouchend='stopMovement()'><span class="arrows" >&#11017;</span></td>
        <td class="button" ontouchstart='startMovement("1")' ontouchend='stopMovement()'><span class="arrows" >&#8679;</span></td>
        <td class="button" ontouchstart='startMovement("6")' ontouchend='stopMovement()'><span class="arrows" >&#11016;</span></td>
      </tr>
      <tr>
        <td class="button" ontouchstart='startMovement("3")' ontouchend='stopMovement()'><span class="arrows" >&#8678;</span></td>
        <td class="button"></td>    
        <td class="button" ontouchstart='startMovement("4")' ontouchend='stopMovement()'><span class="arrows" >&#8680;</span></td>
      </tr>
      <tr>
        <td class="button" ontouchstart='startMovement("7")' ontouchend='stopMovement()'><span class="arrows" >&#11019;</span></td>
        <td class="button" ontouchstart='startMovement("2")' ontouchend='stopMovement()'><span class="arrows" >&#8681;</span></td>
        <td class="button" ontouchstart='startMovement("8")' ontouchend='stopMovement()'><span class="arrows" >&#11018;</span></td>
      </tr>
      <tr>
        <td class="button" ontouchstart='startMovement("9")' ontouchend='stopMovement()'><span class="circularArrows" >&#8634;</span></td>
        <td></td>
        <td class="button" ontouchstart='startMovement("10")' ontouchend='stopMovement()'><span class="circularArrows" >&#8635;</span></td>
      </tr>
      <tr><td colspan="3" style="height:10px"></td></tr>
      <tr>
        <td style="text-align:left"><b>Speed:</b></td>
        <td colspan=2>
         <div class="slidecontainer">
            <input type="range" min="0" max="255" value="150" class="slider" id="Speed" oninput='sendButtonInput("Speed",value)'>
          </div>
        </td>
      </tr>        
      <tr>
        <td style="text-align:left"><b>Light:</b></td>
        <td colspan=2>
          <div class="slidecontainer">
            <input type="range" min="0" max="255" value="0" class="slider" id="Light" oninput='sendButtonInput("Light",value)'>
          </div>
        </td>   
      </tr>
    </table>
  
    <script>
      var webSocketCameraUrl = "ws:\/\/" + window.location.hostname + "/Camera";
      var webSocketCarInputUrl = "ws:\/\/" + window.location.hostname + "/CarInput";      
      var websocketCamera;
      var websocketCarInput;
      
      function initCameraWebSocket() 
      {
        websocketCamera = new WebSocket(webSocketCameraUrl);
        websocketCamera.binaryType = 'blob';
        websocketCamera.onopen    = function(event){};
        websocketCamera.onclose   = function(event){setTimeout(initCameraWebSocket, 2000);};
        websocketCamera.onmessage = function(event)
        {
          var imageId = document.getElementById("cameraImage");
          imageId.src = URL.createObjectURL(event.data);
        };
      }
      
      function initCarInputWebSocket() 
      {
        websocketCarInput = new WebSocket(webSocketCarInputUrl);
        websocketCarInput.onopen    = function(event)
        {
          var speedButton = document.getElementById("Speed");
          sendButtonInput("Speed", speedButton.value);
          var lightButton = document.getElementById("Light");
          sendButtonInput("Light", lightButton.value);
        };
        websocketCarInput.onclose   = function(event){setTimeout(initCarInputWebSocket, 2000);};
        websocketCarInput.onmessage = function(event){};        
      }
      
      function initWebSocket() 
      {
        initCameraWebSocket ();
        initCarInputWebSocket();
      }

      function sendButtonInput(key, value) {
        if (websocketCarInput && websocketCarInput.readyState === WebSocket.OPEN) {
          var data = key + "," + value;
          websocketCarInput.send(data);
        }
      }

      let movementInterval = null;

      function startMovement(command) {
        if (movementInterval !== null) clearInterval(movementInterval);
        sendButtonInput("MoveCar", command);
        movementInterval = setInterval(() => {
          sendButtonInput("MoveCar", command);
        }, 100);
      }


      function stopMovement() {
        if (movementInterval !== null) {
          clearInterval(movementInterval);
          movementInterval = null;
          sendButtonInput("MoveCar", "0"); // stop command
        }
      }

      window.onload = function () {
        initWebSocket();
        document.getElementById("mainTable").addEventListener("touchend", function(event){
          event.preventDefault();
        });
      };
   
    </script>
  </body>    
</html>
)HTMLHOMEPAGE";

/*=========== SEND CAR'S COMMANDS ===========*/
void sendCarCommands(command cmd) {
  PacketData data;
  data.xAxisValue = 0;
  data.yAxisValue = 0;  
  data.zAxisValue = 0;
  data.command = static_cast<uint8_t>(cmd);
  data.source = 12;

  Serial.print("Remote Command Sent: ");
  Serial.println(data.command);

  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&data, sizeof(data));
  if (result == ESP_OK) {
    Serial.println("Sent ESP-NOW remote command successfully");
  } else {
    Serial.printf("Error sending ESP-NOW remote command: %d\n", result);
  }
}

void handleWebSocketCommand(String commandStr) {
  if (commandStr.startsWith("MoveCar,")) {
    int cmdInt = commandStr.substring(8).toInt();
    if (cmdInt >= STOP && cmdInt <= TURN_RIGHT) {
      sendCarCommands(static_cast<command>(cmdInt));
    } else {
      Serial.println("Unknown command");
    }
  } else if (commandStr.startsWith("Light,")) {
    int brightness = commandStr.substring(6).toInt();
    ledcWrite(PWMLightChannel, brightness);
  } else {
    Serial.println("Invalid command format");
  }
}

// Xử lý WebSocket sự kiện điều khiển xe
void onCarInputWebSocketEvent(AsyncWebSocket *server,
                              AsyncWebSocketClient *client,
                              AwsEventType type,
                              void *arg,
                              uint8_t *data,
                              size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      sendCarCommands(static_cast<command>(0));  
      ledcWrite(PWMLightChannel, 0);
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        String message = String((char *)data).substring(0, len);
        Serial.printf("Received WebSocket message: %s\n", message.c_str());
        handleWebSocketCommand(message);
      }
      break;
    }
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
    default:
      break;
  }
}

void handleRoot(AsyncWebServerRequest *request) {
  request->send_P(200, "text/html", htmlHomePage);
}

void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "File Not Found");
}

void onCameraWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                            AwsEventType type, void *arg,
                            uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("Camera WebSocket client connected: %u\n", client->id());
      cameraClientId = client->id();
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("Camera WebSocket client disconnected: %u\n", client->id());
      if (cameraClientId == client->id()) {
        cameraClientId = 0;
      }
      break;
    case WS_EVT_DATA:
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
    default:
      break;
  }
}

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  if (psramFound()) {
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }
  Serial.println("Camera initialized");
}

void sendCameraFrame() {
  static int cameraRetryCount = 0;
  const int maxRetries = 5;

  if (cameraClientId == 0) return;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");

    if (cameraRetryCount < maxRetries) {
      Serial.println("Reinitializing camera...");
      esp_camera_deinit();
      setupCamera();
      cameraRetryCount++;
    } else {
      Serial.println("Max camera retries reached. Skipping reinit.");
    }

    return;
  }

  cameraRetryCount = 0;  

  if (!wsCamera.availableForWrite(cameraClientId)) {
    esp_camera_fb_return(fb);
    return;
  }

  wsCamera.binary(cameraClientId, fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void setupCameraStreaming() {
  const int intervalMs = 100;  
  esp_timer_create_args_t timer_args = {
    .callback = [](void *) {
      sendCameraFrame();
    },
    .name = "frameTimer"
  };
  static esp_timer_handle_t frameTimer;
  esp_timer_create(&timer_args, &frameTimer);
  esp_timer_start_periodic(frameTimer, intervalMs * 1000);
}

void setUpPinModes() {
  ledcSetup(PWMLightChannel, PWMFreq, PWMResolution);
  pinMode(LIGHT_PIN, OUTPUT);
  ledcAttachPin(LIGHT_PIN, PWMLightChannel);
}

void setup() {
  Serial.begin(115200);  
  setUpPinModes();

  setupWiFi();

  setupCamera();
  initEspNow();
  sendCarCommands(static_cast<command>(0)); 

  // Setup WebSocket event handlers
  wsCarInput.onEvent(onCarInputWebSocketEvent);
  wsCamera.onEvent(onCameraWebSocketEvent);
  server.addHandler(&wsCarInput);
  server.addHandler(&wsCamera);

  // Routes
  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);

  // Start server
  server.begin();
  Serial.println("HTTP server started");
  setupCameraStreaming();
}

void loop() {
  wsCamera.cleanupClients();
  wsCarInput.cleanupClients();

  delay(10);  
}

