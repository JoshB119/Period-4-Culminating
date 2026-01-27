#include "esp_camera.h"
#include "ESP32Servo.h"

// Camera Pins (OV3660 / WROVER)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   21
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27

#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     19
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM      5
#define Y2_GPIO_NUM      4
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

Servo panServo;
Servo tiltServo;
float tiltAngle = 90;
float panAngle = 90;

enum {
  minX,
  maxX,
  minY,
  maxY,
  pixelCount,
  b_size // not size in pixels, its the amount of integers being stored in the blob, in this case 5.
};

const int IMG_WIDTH  = 160;
const int IMG_HEIGHT = 120;
int blob[b_size];

// Global Frame Pointer
camera_fb_t *fb = nullptr;

// RGB -> HSV
void RGBtoHSV(uint8_t r, uint8_t g, uint8_t b, float &h, float &s, float &v) {
  float rd = r / 255.0;
  float gd = g / 255.0;
  float bd = b / 255.0;

  float maxVal = max(max(rd, gd), bd);
  float minVal = min(min(rd, gd), bd);
  float delta  = maxVal - minVal;

  v = maxVal * 255.0;
  s = (maxVal == 0) ? 0 : (delta / maxVal) * 255.0;

  if (delta == 0) h = 0;
  else if (maxVal == rd) h = 60 * fmod(((gd - bd) / delta), 6);
  else if (maxVal == gd) h = 60 * (((bd - rd) / delta) + 2);
  else h = 60 * (((rd - gd) / delta) + 4);

  if (h < 0) h += 360;
  h /= 2.0; // divided by 2 so it shows up in 180 range like standard hsv systems
}

// Capture frame
bool captureFrame() {
  if (fb) esp_camera_fb_return(fb);

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    return false;
  }
  return true;
}

// ===== Get HSV at a pixel (YOU WILL USE THIS A LOT) =====
bool getHSVAt(int x, int y, float &h, float &s, float &v) {
  if (!fb) return false;
  if (x < 0 || y < 0 || x >= fb->width || y >= fb->height) return false;

  // YUV422 format: 2 pixels per 4 bytes → [Y0 U Y1 V]
  int rowOffset = y * fb->width * 2;
  int pixelOffset = (x & ~1) * 2;
  int i = rowOffset + pixelOffset;

  uint8_t y_val = fb->buf[i + (x % 2) * 2];
  uint8_t u = fb->buf[i + 1];
  uint8_t v_ = fb->buf[i + 3];

  int r = y_val + 1.402 * (v_ - 128);
  int g = y_val - 0.344 * (u - 128) - 0.714 * (v_ - 128);
  int b = y_val + 1.772 * (u - 128);

  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);

  RGBtoHSV(r, g, b, h, s, v);
  return true;
}

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);

  panServo.attach(14, 500, 2400);
  tiltServo.attach(13, 500, 2400);

  camera_config_t config; // settings for the camera
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;

  config.pixel_format = PIXFORMAT_YUV422;
  config.frame_size   = FRAMESIZE_QQVGA;
  config.fb_count     = 1;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  Serial.println("Camera ready");

  // Warm up the camera for better quality
  for (int i = 0; i < 30; i++) {
    camera_fb_t *tmp = esp_camera_fb_get();
    if (tmp) esp_camera_fb_return(tmp);
  }
}

bool isRed(int x, int y) { // check if the mentioned pixel is red (using hsv values)
  float h, s, v;
  getHSVAt(x, y, h, s, v);
  if (int(h) <= 20 && int(s) >= 155) {
    return true; // if true it gets counted as part of the object
  } else {
    return false;
  }
}

bool isNoise(int x, int y) { // Don't want random single pixels (noise) messing up bounding box
  if (x + 1 < IMG_WIDTH && isRed(x + 1, y)) { // so we search every direction to see if a pixel is alone
    return false;
  }
  else if (x - 1 >= 0 && isRed(x - 1, y)) { 
    return false;
  } 
  else if (y + 1 < IMG_HEIGHT && isRed(x, y + 1)) { 
    return false;
  } 
  else if (y - 1 >= 0 && isRed(x, y - 1)) {
    return false;
  }
  else {
    return true; // if it is alone it doesnt get included in the final bounding box
  }
}

void moveServos(int goalX, int goalY) {
  const float servoMulti = 0.15; // adjust until seems right
  int errorX = goalX - 80;
  int errorY = goalY - 60;
  panAngle -= errorX * servoMulti;
  tiltAngle -= errorY * servoMulti;
  panAngle  = constrain(panAngle,  20, 160); // stops servos from going too far
  tiltAngle = constrain(tiltAngle, 20, 160);
  Serial.printf("Moving to: %d, %d\n", goalX, goalY);
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
}

void loop() {
  captureFrame();
  blob[pixelCount] = 0;

  blob[minX] = IMG_WIDTH;
  blob[minY] = IMG_HEIGHT;
  blob[maxX] = 0;
  blob[maxY] = 0;
  float h, s, v;
  for (int pixelY = 0; pixelY < IMG_HEIGHT; pixelY++) {
    for (int pixelX = 0; pixelX < IMG_WIDTH; pixelX++) {
      if (isRed(pixelX, pixelY) && !isNoise(pixelX, pixelY)) {
        blob[pixelCount]++;
        if (pixelX < blob[minX]) blob[minX] = pixelX; // sets the bounding box of the whole image which lets use figure out where the center of it is
        if (pixelX > blob[maxX]) blob[maxX] = pixelX;
        if (pixelY < blob[minY]) blob[minY] = pixelY;
        if (pixelY > blob[maxY]) blob[maxY] = pixelY;
      }
    }
  }
  if (blob[pixelCount] > 20) {
    int objectX = (blob[maxX] + blob[minX]) / 2;
    int objectY = (blob[maxY] + blob[minY]) / 2;
    Serial.printf(
      "Blob: pixels=%d box=[%d,%d]→[%d,%d], (%d, %d)\n",
      blob[pixelCount],
      blob[minX],
      blob[minY],
      blob[maxX],
      blob[maxY],
      objectX,
      objectY
    );
    moveServos(objectX, objectY);
  }
  delay(200);
}
