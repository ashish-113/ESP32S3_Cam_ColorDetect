// ESP32-S3-CAM: live stream + red/green/yellow color detection
// Tested on Hiwonder / ThinkRobotics ESP32-S3-CAM AI Vision Module
// Arduino IDE 2.3.7, esp32 core 2.0.17
// See README.md for required Tools-menu settings.

#include "esp_camera.h"
#include "img_converters.h"
#include "esp_log.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>

// ---------- Hotspot ----------
const char* AP_SSID = "ESP32S3-CAM";   // open network, no password
const IPAddress AP_IP (192,168,4,1);
const IPAddress AP_GW (192,168,4,1);
const IPAddress AP_SN (255,255,255,0);

// ---------- I2C: this board acts as I2C SLAVE for the bot MCU ----------
// Wire camera IIC.SDA -> ESP32 GPIO21, camera IIC.SCL -> ESP32 GPIO22.
// Common GND is required between the two boards.
// If your Hiwonder expansion board routes the IIC header to different
// GPIOs, only change these two pin defines.
#define I2C_SLAVE_ADDR  0x52
#define I2C_SDA_PIN     1
#define I2C_SCL_PIN     2
#define I2C_FREQ_HZ     100000

// ---------- Pin map: Hiwonder / NullLab ESP32-S3-CAM ----------
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5
#define Y9_GPIO_NUM     16
#define Y8_GPIO_NUM     17
#define Y7_GPIO_NUM     18
#define Y6_GPIO_NUM     12
#define Y5_GPIO_NUM     10
#define Y4_GPIO_NUM      8
#define Y3_GPIO_NUM      9
#define Y2_GPIO_NUM     11
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM   13

WebServer server(80);

struct Detection {
  float red = 0, green = 0, yellow = 0;
  const char* dominant = "none";
  float confidence = 0;
} det;

// ---------- I2C publish buffer ----------
// Five bytes that an external I2C master can read from register 0x00:
//   [0] dominant code (0=none, 1=red, 2=green, 3=yellow)
//   [1] red %      (0..100)
//   [2] green %    (0..100)
//   [3] yellow %   (0..100)
//   [4] confidence (0..100)
volatile uint8_t pubBuf[5] = {0, 0, 0, 0, 0};
volatile uint8_t i2cReg    = 0;

static inline uint8_t clamp100(float v) {
  int x = (int)roundf(v);
  if (x < 0)   x = 0;
  if (x > 100) x = 100;
  return (uint8_t)x;
}

static uint8_t dominantCode() {
  if (det.confidence < 1.0f)                return 0;
  if (strcmp(det.dominant, "red")    == 0)  return 1;
  if (strcmp(det.dominant, "green")  == 0)  return 2;
  if (strcmp(det.dominant, "yellow") == 0)  return 3;
  return 0;
}

static void publishDetection() {
  pubBuf[0] = dominantCode();
  pubBuf[1] = clamp100(det.red);
  pubBuf[2] = clamp100(det.green);
  pubBuf[3] = clamp100(det.yellow);
  pubBuf[4] = clamp100(det.confidence);
}

// I2C slave callbacks. Keep these tiny - they run in interrupt context.
void onI2CReceive(int n) {
  if (n >= 1) i2cReg = Wire.read();
  while (Wire.available()) Wire.read();
}

void onI2CRequest() {
  if (i2cReg == 0x00) {
    Wire.write((const uint8_t*)pubBuf, 5);
  } else {
    uint8_t z = 0xFF;
    Wire.write(&z, 1);
  }
}

// ---------- Color classification ----------
// Returns: 0=other, 1=red, 2=green, 3=yellow
static inline uint8_t classify(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t maxc = r > g ? (r > b ? r : b) : (g > b ? g : b);
  uint8_t minc = r < g ? (r < b ? r : b) : (g < b ? g : b);
  uint8_t v = maxc;
  uint8_t delta = maxc - minc;
  if (delta == 0) return 0;
  uint8_t s = (uint16_t)delta * 255 / maxc;
  // Need a saturated, decently bright pixel to be a "color".
  if (s < 85 || v < 70) return 0;
  float h;
  if (maxc == r)      h = 60.0f * ((float)(g - b) / delta);
  else if (maxc == g) h = 60.0f * (2.0f + (float)(b - r) / delta);
  else                h = 60.0f * (4.0f + (float)(r - g) / delta);
  if (h < 0) h += 360;
  if (h < 20  || h > 340)  return 1;             // red    (~ 0  deg)
  if (h > 40  && h < 70)   return 3;             // yellow (~ 60 deg)
  if (h > 85  && h < 170)  return 2;             // green  (~120 deg)
  return 0;
}

static void analyzeRGB565(const uint8_t* buf, int w, int h) {
  uint32_t red = 0, green = 0, yellow = 0, total = 0;
  for (int y = 0; y < h; y += 2) {
    for (int x = 0; x < w; x += 2) {
      int i = (y * w + x) * 2;
      uint16_t p = ((uint16_t)buf[i] << 8) | buf[i + 1];
      uint8_t r = ((p >> 11) & 0x1F) << 3;
      uint8_t g = ((p >> 5)  & 0x3F) << 2;
      uint8_t b = ( p        & 0x1F) << 3;
      uint8_t c = classify(r, g, b);
      total++;
      if      (c == 1) red++;
      else if (c == 2) green++;
      else if (c == 3) yellow++;
    }
  }
  if (!total) return;
  det.red    = 100.0f * red    / total;
  det.green  = 100.0f * green  / total;
  det.yellow = 100.0f * yellow / total;
  float best = det.red; det.dominant = "red";    det.confidence = det.red;
  if (det.green  > best) { best = det.green;  det.dominant = "green";  det.confidence = det.green;  }
  if (det.yellow > best) { best = det.yellow; det.dominant = "yellow"; det.confidence = det.yellow; }
  if (best < 2.0f) { det.dominant = "none"; det.confidence = 0; }
}

// ---------- HTTP handlers ----------

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<title>ESP32-S3-CAM</title>
<style>
  body{font-family:system-ui,sans-serif;background:#111;color:#eee;text-align:center;margin:0;padding:16px}
  img{max-width:95vw;border:2px solid #333;border-radius:8px;background:#000}
  .row{display:flex;gap:12px;justify-content:center;flex-wrap:wrap;margin-top:12px}
  .card{background:#1b1b1b;padding:10px 14px;border-radius:8px;min-width:130px}
  .big{font-size:28px;font-weight:700}
  .bar{height:8px;background:#333;border-radius:4px;margin-top:6px;overflow:hidden}
  .fill{height:100%;width:0;transition:width .2s}
  .red .fill{background:#ef4444}.green .fill{background:#22c55e}.yellow .fill{background:#facc15}
  .dom{margin-top:10px;font-size:18px}
</style></head><body>
<h2>ESP32-S3-CAM &bull; live + color detection</h2>
<img id="cam" src="/jpg">
<div class="dom">Dominant: <span id="dom">-</span> (<span id="conf">0</span>%)</div>
<div class="row">
  <div class="card red">Red <div class="big"><span id="r">0</span>%</div><div class="bar"><div class="fill" id="rb"></div></div></div>
  <div class="card green">Green <div class="big"><span id="g">0</span>%</div><div class="bar"><div class="fill" id="gb"></div></div></div>
  <div class="card yellow">Yellow <div class="big"><span id="y">0</span>%</div><div class="bar"><div class="fill" id="yb"></div></div></div>
</div>
<script>
  const img = document.getElementById('cam');
  function refreshImg(){
    const n = new Image();
    n.onload  = () => { img.src = n.src; setTimeout(refreshImg, 50); };
    n.onerror = () => setTimeout(refreshImg, 250);
    n.src = '/jpg?t=' + Date.now();
  }
  async function refreshStatus(){
    try{
      const r = await fetch('/status', {cache:'no-store'});
      const j = await r.json();
      document.getElementById('r').textContent = j.red.toFixed(1);
      document.getElementById('g').textContent = j.green.toFixed(1);
      document.getElementById('y').textContent = j.yellow.toFixed(1);
      document.getElementById('rb').style.width = Math.min(100,j.red)+'%';
      document.getElementById('gb').style.width = Math.min(100,j.green)+'%';
      document.getElementById('yb').style.width = Math.min(100,j.yellow)+'%';
      document.getElementById('dom').textContent = j.dominant;
      document.getElementById('conf').textContent = j.confidence.toFixed(1);
    }catch(e){}
    setTimeout(refreshStatus, 300);
  }
  refreshImg();
  refreshStatus();
</script></body></html>
)HTML";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleStatus() {
  char buf[192];
  snprintf(buf, sizeof(buf),
    "{\"red\":%.2f,\"green\":%.2f,\"yellow\":%.2f,\"dominant\":\"%s\",\"confidence\":%.2f}",
    det.red, det.green, det.yellow, det.dominant, det.confidence);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

void handleJpg() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(500, "text/plain", "capture failed"); return; }

  if (fb->format == PIXFORMAT_RGB565) {
    analyzeRGB565(fb->buf, fb->width, fb->height);
    publishDetection();
  }

  uint8_t* jpg_buf = nullptr;
  size_t   jpg_len = 0;
  bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
  esp_camera_fb_return(fb);

  if (!ok) { server.send(500, "text/plain", "encode failed"); return; }

  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Content-Disposition", "inline; filename=cam.jpg");
  server.send_P(200, "image/jpeg", (const char*)jpg_buf, jpg_len);
  free(jpg_buf);
}

// ---------- camera ----------

bool initCamera() {
  bool hasPsram = psramFound();

  camera_config_t c;
  memset(&c, 0, sizeof(c));
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk  = XCLK_GPIO_NUM;
  c.pin_pclk  = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href  = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn  = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;

  c.xclk_freq_hz = 10000000;
  c.pixel_format = PIXFORMAT_RGB565;
  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  if (hasPsram) {
    c.frame_size  = FRAMESIZE_QVGA;   // 320x240
    c.fb_count    = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    c.frame_size  = FRAMESIZE_QQVGA;  // 160x120
    c.fb_count    = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init FAILED 0x%x\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    Serial.printf("Sensor PID: 0x%02x (0x9b=GC0308, 0x21=GC2145, 0x26=OV2640)\n",
                  s->id.PID);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 3000) { delay(10); }
  delay(300);

  Serial.println("\n== ESP32-S3-CAM hotspot demo ==");
  Serial.flush();

  esp_log_level_set("cam_hal", ESP_LOG_NONE);
  esp_log_level_set("gdma",    ESP_LOG_NONE);

  Serial.printf("PSRAM: %s, %u bytes\n",
                psramFound() ? "YES" : "NO", (unsigned)ESP.getPsramSize());

  if (!initCamera()) {
    Serial.println("Halting.");
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_SN);
  WiFi.softAP(AP_SSID);
  Serial.printf("AP: %s  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  // Start as I2C slave on the IIC header (custom pins on ESP32-S3).
  Wire.begin((uint8_t)I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  Serial.printf("I2C slave at 0x%02X on SDA=%d SCL=%d\n",
                I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);

  server.on("/",       handleRoot);
  server.on("/jpg",    handleJpg);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("HTTP server started.");
  Serial.println("Join Wi-Fi 'ESP32S3-CAM', open http://192.168.4.1/");
}

void loop() {
  server.handleClient();

  static uint32_t lastLog = 0;
  if (millis() - lastLog > 1000) {
    lastLog = millis();
    Serial.printf("R=%.1f%% G=%.1f%% Y=%.1f%% dom=%s (%.1f%%)\n",
                  det.red, det.green, det.yellow, det.dominant, det.confidence);
  }
}
