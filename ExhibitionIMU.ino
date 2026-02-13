#define BLYNK_TEMPLATE_ID "TMPL6rf2MoXmA"
#define BLYNK_TEMPLATE_NAME "imu"
#define BLYNK_AUTH_TOKEN "axHlSwIS2vEvjp2a4uu4Pd7aU8J1_hQt"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_now.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

Adafruit_MPU6050 mpu;

// WiFi credentials
char ssid[] = "Nonet";
char pass[] = "123456789";

typedef struct struct_message {
  float distance;
} struct_message;

struct_message myData;

BlynkTimer timer;

// Define the virtual pin for notifications
#define NOTIFY_VPIN V5
#define EVENT_NAME "PostureAlert"  // Define a name for your event

// Callback function for receiving data
void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&myData, data, sizeof(myData));
  Serial.print("Received Distance: ");
  Serial.print(myData.distance);
  Serial.println(" cm");

  // Send the distance to Blynk app
  Serial.print("Sending Distance to Blynk: ");
  Serial.println(myData.distance);
  Blynk.virtualWrite(V5, myData.distance);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}

void loop() {
  Blynk.run();
  sensors_event_t accel, gyro;
  mpu.getAccelerometerSensor()->getEvent(&accel);
  mpu.getGyroSensor()->getEvent(&gyro);

  // Calculate the gyro X, Y, and Z angles
  float gyroXAngle = gyro.gyro.x * 57.2958;
  float gyroYAngle = gyro.gyro.y * 57.2958;  // Y-axis angle
  float gyroZAngle = gyro.gyro.z * 57.2958;  // Z-axis angle

  // Print gyro X, Y, Z values to Serial Monitor
  Serial.print("Gyro X: ");
  Serial.print(gyroXAngle);
  Serial.print(" °/s, Y: ");
  Serial.print(gyroYAngle);  // Print Y-axis value
  Serial.print(" °/s, Z: ");
  Serial.print(gyroZAngle);  // Print Z-axis value
  Serial.println(" °/s");

  // Send X-axis angle to Blynk app (V0)
  Blynk.virtualWrite(V0, gyroXAngle);

  // Send Y-axis angle to Blynk app (V1)
  Blynk.virtualWrite(V1, gyroYAngle);

  // Send Z-axis angle to Blynk app (V2)
  Blynk.virtualWrite(V2, gyroZAngle);

  // Check if the X-axis angle is more than 6 degrees
  if (gyroXAngle > 6) {
    Serial.println("BAD POSTURE DETECTED, triggering notification...");

    // Trigger notification by setting V5 to 1
    Blynk.virtualWrite(NOTIFY_VPIN, 1);

    // Log the event to Blynk server
    Blynk.logEvent("angle_x", "!!!HUMAN QUESTION MARK IN THE MAKING!!!");
  } else {
    // Reset V5 if the angle is within the limit
    Blynk.virtualWrite(NOTIFY_VPIN, 0);
  }

  delay(1000);
}
