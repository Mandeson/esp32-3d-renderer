#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "../lib/cglm/include/cglm/cglm.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using software SPI (default case):
/*#define OLED_MOSI  23
#define OLED_CLK   18
#define OLED_DC    4
#define OLED_CS    5
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, -1, OLED_CS);*/

// Comment out above, uncomment this block to use hardware SPI
#define OLED_DC     4
#define OLED_CS     5
SPIClass vspi(VSPI);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  &vspi, OLED_DC, -1, OLED_CS);
MPU6050 mpudev; // I2C

const float vertices[12 * 2 * 3] = {
	-0.5, -0.5, -0.5,
	 0.5, -0.5, -0.5,

	 0.5, -0.5, -0.5,
	 0.5,  0.5, -0.5,

	 0.5,  0.5, -0.5,
	-0.5,  0.5, -0.5,

	-0.5,  0.5, -0.5,
	-0.5, -0.5, -0.5,

	-0.5, -0.5,  0.5,
	 0.5, -0.5,  0.5,

	 0.5, -0.5,  0.5,
	 0.5,  0.5,  0.5,

	 0.5,  0.5,  0.5,
	-0.5,  0.5,  0.5,

	-0.5,  0.5,  0.5,
	-0.5, -0.5,  0.5,

	-0.5, -0.5, -0.5,
	-0.5, -0.5,  0.5,

	 0.5, -0.5, -0.5,
	 0.5, -0.5,  0.5,

	 0.5,  0.5, -0.5,
	 0.5,  0.5,  0.5,

	-0.5,  0.5, -0.5,
	-0.5,  0.5,  0.5,
};

mat4 projection;
float angle = 20.0;
uint64_t systemTime = 0;

bool init_success = false;

uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[42];  // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;         // [w, x, y, z]         Quaternion container
VectorFloat gravity;  // [x, y, z]            Gravity vector
float ypr[3];  

void drawLine(float x, float y, float xd, float yd) {
  int32_t X0 = (int32_t)((x + 1.0f) / 2.0f * 127.0f);
	int32_t Y0 = (int32_t)((y * (-1.0f) + 1.0f) / 2.0f * 63.0f);
	int32_t X1 = (int32_t)((xd + 1.0f) / 2.0f * 127.0f);
	int32_t Y1 = (int32_t)((yd * (-1.0f) + 1.0f) / 2.0f * 63.0f);

  display.drawLine(X0, Y0, X1, Y1, SSD1306_WHITE);
}

void drawCube(float x, float y, float z, float rotx, float roty, float rotz) {
  vec3 pos = {x, y, z};
  vec3 rotAxisX = {1.0f, 0.0f, 0.0f};
  vec3 rotAxisY = {0.0f, 1.0f, 0.0f};
  vec3 rotAxisZ = {0.0f, 0.0f, 1.0f};

	mat4 transform;
  glm_mat4_identity(transform);
	glm_translate(transform, pos);
	glm_rotate(transform, rotx, rotAxisX);
  glm_rotate(transform, roty, rotAxisY);
  glm_rotate(transform, rotz, rotAxisZ);

	mat4 mvp;
  glm_mat4_mul(projection, transform, mvp);

	for (uint32_t i = 0; i < 12; i++) {
		uint32_t offset = i * 2 * 3;
		vec4 beg = {vertices[offset + 0], vertices[offset + 1], vertices[offset + 2], 1.0f};
	  vec4 end = {vertices[offset + 3], vertices[offset + 4], vertices[offset + 5], 1.0f};

		glm_mat4_mulv(mvp, beg, beg);
    glm_mat4_mulv(mvp, end, end);

		vec2 beg2D = {beg[0] / beg[3], beg[1] / beg[3]};
		vec2 end2D = {end[0] / end[3], end[1] / end[3]};

		drawLine(beg2D[0], beg2D[1], end2D[0], end2D[1]);
	}
}

void setup() {
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  Wire.begin(13, 15);

  mpudev.initialize();

  if (!mpudev.testConnection()) {
      Serial.println("Connection to mpu failed");
      return;
  }

  if (mpudev.dmpInitialize()) {
      Serial.print("mpu: DMP Initialization failed");
      return;
  }

  // Calibration on start?
  // mpudev.CalibrateAccel(10);
  // mpudev.CalibrateGyro(10);

  // Alternatively, we can set offsets obtained by running IMU_Zero
  // example from the library.
  // mpudev.setXAccelOffset(0);
  // mpudev.setYAccelOffset(0);
  // mpudev.setZAccelOffset(0);
  // mpudev.setXGyroOffset(0);
  // mpudev.setYGyroOffset(0);
  // mpudev.setZGyroOffset(0);

  mpudev.CalibrateAccel(10);
  mpudev.CalibrateGyro(10);
  mpudev.setDMPEnabled(true);
  packetSize = mpudev.dmpGetFIFOPacketSize();

  glm_perspective(glm_rad(25.0f), 128.0f / 64.0f, 0.1f, 100.0f, projection);

  init_success = true;
  Serial.println("Init suceeded");
}

void loop() {
  if (!init_success)
    return;

  if (mpudev.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpudev.dmpGetQuaternion(&q, FIFOBuffer);
    mpudev.dmpGetGravity(&gravity, &q);
    mpudev.dmpGetYawPitchRoll(ypr, &q, &gravity);

    uint64_t currentTime = esp_timer_get_time();
    uint32_t dTime = currentTime - systemTime;
    systemTime = currentTime;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print(ypr[0]);
    display.print("\n");
    display.print(ypr[1]);
    display.print("\n");
    display.println(ypr[2]);
    //display.print("FPS: ");
    //display.println(1000000 / dTime);

    drawCube(0.0f, -0.1, -4.2f, -ypr[1], -ypr[2], ypr[0]);
    /*drawCube(-3.8f, 0.0, -11.0f, angle + 30.0, 0.2f, 0.6f, 0.3f);
    drawCube(2.5f, 0.0, -8.0f, angle, 0.9f, 0.3f, 0.7f);*/

    display.display();
  }

  vTaskDelay(pdMS_TO_TICKS(10));
}
