// IPCamera.ino
// ESP32-CAM IP Camera Example
// This sketch sets up the ESP32-CAM as an IP camera streaming MJPEG over HTTP

#include "esp_camera.h"

#include <WiFi.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager


// WiFi Access Point credentials
const char* ap_ssid = "ESP32CAM_AP";
const char* ap_password = "12345678";

bool wifi_connected = false;

// Camera configuration (AI-Thinker module)

// AI-Thinker ESP32-CAM pin definitions
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Flash LED pin (built-in LED on most ESP32-CAM modules)
#define FLASH_LED_PIN     4

// Flash LED state
bool flashState = true; // Start with flash on

WiFiServer server(80);

void startCameraServer();

// Flash control functions
void setFlashState(bool state) {
  flashState = state;
  digitalWrite(FLASH_LED_PIN, state ? HIGH : LOW);
  Serial.println("Flash LED " + String(state ? "ON" : "OFF"));
}

void setup() {
  Serial.begin(115200);
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
  config.jpeg_quality = 12;
  config.fb_count = 2;

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Initialize flash LED at full brightness
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW); // Turn off
  flashState = false;
  Serial.println("Flash LED initialized at off state");

  // WiFiManager: Try to connect to saved WiFi, else start AP config portal
  WiFiManager wifiManager;
  wifiManager.setTimeout(180); // 3 min timeout for config portal
  Serial.println("[WiFiManager] Attempting to connect to saved WiFi...");
  if (wifiManager.autoConnect("ESP32CAM_ConfigAP")) {
    wifi_connected = true;
    Serial.print("WiFi connected. IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Camera Stream Ready. Access via http://" + WiFi.localIP().toString() + "/stream");
  } else {
    wifi_connected = false;
    Serial.println("[WiFiManager] No WiFi credentials set. Starting fallback AP mode.");
    WiFi.softAP(ap_ssid, ap_password);
    delay(1000);
    Serial.print("Access Point started. IP address: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("Camera Stream Ready. Connect to WiFi '" + String(ap_ssid) + "' and access http://" + WiFi.softAPIP().toString() + "/stream");
  }

  // Start web server
  server.begin();
  startCameraServer();
}

void loop() {
  // Handle incoming HTTP requests for streaming
  handleStreamRequest();
}

// Camera streaming server implementation
void startCameraServer() {
  Serial.println("Starting camera server on port 80");
  // Server is already started with server.begin() in setup()
}

void handleStreamRequest() {
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    client.flush();
    
    // Handle flash control requests
    if (request.indexOf("/flash/on") != -1) {
      setFlashState(true);
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Access-Control-Allow-Origin: *");
      client.println();
      client.println("{\"status\":\"success\",\"flash\":\"on\"}");
    } else if (request.indexOf("/flash/off") != -1) {
      setFlashState(false);
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Access-Control-Allow-Origin: *");
      client.println();
      client.println("{\"status\":\"success\",\"flash\":\"off\"}");
    } else if (request.indexOf("/flash/toggle") != -1) {
      setFlashState(!flashState);
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Access-Control-Allow-Origin: *");
      client.println();
      client.println("{\"status\":\"success\",\"flash\":\"" + String(flashState ? "on" : "off") + "\"}");
    } else if (request.indexOf("/flash/status") != -1) {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Access-Control-Allow-Origin: *");
      client.println();
      client.println("{\"status\":\"success\",\"flash\":\"" + String(flashState ? "on" : "off") + "\"}");
    } else if (request.indexOf("/stream") != -1) {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
      client.println("Connection: close");
      client.println();
      
      while (client.connected()) {
        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) {
          Serial.println("Camera capture failed");
          break;
        }
        
        client.println("--frame");
        client.println("Content-Type: image/jpeg");
        client.print("Content-Length: ");
        client.println(fb->len);
        client.println();
        client.write(fb->buf, fb->len);
        client.println();
        
        esp_camera_fb_return(fb);
        delay(30); // ~30 FPS
      }
    } else if (request.indexOf("/") != -1) {
      // Send HTML page with flash controls
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/html");
      client.println();
      client.println("<!DOCTYPE html><html><head><title>ESP32-CAM Stream</title></head>");
      client.println("<body><h1>ESP32-CAM Live Stream</h1>");
      client.println("<img src='/stream' style='width:100%; max-width:800px;'><br>");
      client.println("<button onclick=\"fetch('/flash/on')\">Flash ON</button>");
      client.println("<button onclick=\"fetch('/flash/off')\">Flash OFF</button>");
      client.println("<button onclick=\"fetch('/flash/toggle')\">Toggle Flash</button>");
      client.println("</body></html>");
    }
    client.stop();
  }
}
