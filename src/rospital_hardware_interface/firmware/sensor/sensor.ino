#include <Wire.h>
#include <MPU6050.h>

const int trigPins[] = {2, 4, 6, 8, 10, 12};
const int echoPins[] = {3, 5, 7, 9, 11, 13};
MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 6; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
  Wire.begin();
  mpu.initialize();
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2 / 100.0;
}

void loop() {
  float distances[6];
  for (int i = 0; i < 6; i++) {
    distances[i] = getDistance(trigPins[i], echoPins[i]);
    delay(50); // Delay để tránh nhiễu giữa các cảm biến
  }
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  Serial.print("u1,"); Serial.print(distances[0], 2);
  Serial.print(",u2,"); Serial.print(distances[1], 2);
  Serial.print(",u3,"); Serial.print(distances[2], 2);
  Serial.print(",u4,"); Serial.print(distances[3], 2);
  Serial.print(",u5,"); Serial.print(distances[4], 2);
  Serial.print(",u6,"); Serial.print(distances[5], 2);
  Serial.print(",imu,");
  Serial.print(ax / 16384.0, 2); Serial.print(",");
  Serial.print(ay / 16384.0, 2); Serial.print(",");
  Serial.print(az / 16384.0, 2); Serial.print(",");
  Serial.print(gx / 131.0, 2); Serial.print(",");
  Serial.print(gy / 131.0, 2); Serial.print(",");
  Serial.println(gz / 131.0, 2);

  delay(100);
}