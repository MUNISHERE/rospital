#include <Wire.h>
#include <MPU6050.h>

const int trigPins[] = {2, 4, 6};
const int echoPins[] = {3, 5, 7};
MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) {
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
  return pulseIn(echoPin, HIGH) * 0.034 / 2 / 100.0; // Mét
}

void loop() {
  float distances[3];
  for (int i = 0; i < 3; i++) {
    distances[i] = getDistance(trigPins[i], echoPins[i]);
  }
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  Serial.print("u1,"); Serial.print(distances[0], 2);
  Serial.print(",u2,"); Serial.print(distances[1], 2);
  Serial.print(",u3,"); Serial.print(distances[2], 2);
  Serial.print(",imu,");
  Serial.print(ax / 16384.0, 2); Serial.print(",");
  Serial.print(ay / 16384.0, 2); Serial.print(",");
  Serial.print(az / 16384.0, 2); Serial.print(",");
  Serial.print(gx / 131.0, 2); Serial.print(",");
  Serial.print(gy / 131.0, 2); Serial.print(",");
  Serial.println(gz / 131.0, 2);

  delay(100);
}