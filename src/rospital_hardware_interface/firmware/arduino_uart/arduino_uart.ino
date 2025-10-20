#include <ODriveArduino.h>

HardwareSerial& odrive_serial = Serial1;
ODriveArduino odrive(odrive_serial);
const int AXIS_ID_RIGHT = 0;
const int AXIS_ID_LEFT = 1;

float velocity_right = 0.0, velocity_left = 0.0;
float old_right_enc = 0.0, old_left_enc = 0.0;
double right_pos = 0.0, left_pos = 0.0;
const float ENCODER_RESOLUTION = 90.0;

void setup() {
  Serial.begin(115200);
  odrive_serial.begin(115200);
  delay(100);

  Serial.println("Arduino started, waiting for ROS...");
  odrive_serial.print("sr\r\n");  // Soft reset ODrive
  delay(2000);

  // Đặt trạng thái điều khiển vòng kín
  setClosedLoopControl();
}

// Đặt trạng thái AXIS_STATE_CLOSED_LOOP_CONTROL cho cả hai trục
void setClosedLoopControl() {
  odrive_serial.print("w axis0.requested_state 8\r\n");  // AXIS_STATE_CLOSED_LOOP_CONTROL
  odrive_serial.print("w axis1.requested_state 8\r\n");
  delay(1000);

  // Kiểm tra trạng thái ban đầu
  checkODriveStates();
}

// Kiểm tra trạng thái và lỗi của ODrive
void checkODriveStates() {
  Serial.println("Checking ODrive states...");
  odrive_serial.print("r axis0.current_state\r\n");
  String state0 = odrive_serial.readStringUntil('\n');
  odrive_serial.print("r axis1.current_state\r\n");
  String state1 = odrive_serial.readStringUntil('\n');
  Serial.print("Axis 0 state: "); Serial.println(state0);
  Serial.print("Axis 1 state: "); Serial.println(state1);

  // Kiểm tra lỗi nếu state không phải 8
  if (state0.toInt() != 8) {
    checkODriveErrors(AXIS_ID_RIGHT);
  }
  if (state1.toInt() != 8) {
    checkODriveErrors(AXIS_ID_LEFT);
  }
}

// Kiểm tra lỗi chi tiết của trục
void checkODriveErrors(int axis) {
  Serial.print("Checking errors for Axis "); Serial.println(axis);
  odrive_serial.print("r axis"); Serial.print(axis); odrive_serial.print(".error\r\n");
  String axis_error = odrive_serial.readStringUntil('\n');
  odrive_serial.print("r axis"); Serial.print(axis); odrive_serial.print(".motor.error\r\n");
  String motor_error = odrive_serial.readStringUntil('\n');
  odrive_serial.print("r axis"); Serial.print(axis); odrive_serial.print(".encoder.error\r\n");
  String encoder_error = odrive_serial.readStringUntil('\n');
  
  Serial.print("Axis "); Serial.print(axis); Serial.print(" error: "); Serial.println(axis_error);
  Serial.print("Motor "); Serial.print(axis); Serial.print(" error: "); Serial.println(motor_error);
  Serial.print("Encoder "); Serial.print(axis); Serial.print(" error: "); Serial.println(encoder_error);

  // Xóa lỗi và đặt lại trạng thái nếu có lỗi
  if (axis_error.toInt() != 0 || motor_error.toInt() != 0 || encoder_error.toInt() != 0) {
    Serial.println("Clearing errors and resetting state...");
    odrive_serial.print("w axis"); Serial.print(axis); odrive_serial.print(".error 0\r\n");
    odrive_serial.print("w axis"); Serial.print(axis); odrive_serial.print(".motor.error 0\r\n");
    odrive_serial.print("w axis"); Serial.print(axis); odrive_serial.print(".encoder.error 0\r\n");
    odrive_serial.print("w axis"); Serial.print(axis); odrive_serial.print(".requested_state 8\r\n");
  }
}

void processROSData() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    Serial.print("Received from ROS: "); Serial.println(message);
    int rIndex = message.indexOf("r");
    int lIndex = message.indexOf("l");
    int commaIndex = message.indexOf(",", rIndex);

    if (rIndex != -1 && lIndex != -1 && commaIndex != -1) {
      velocity_right = message.substring(rIndex + 1, commaIndex).toFloat();
      velocity_left = message.substring(lIndex + 1).toFloat();
      Serial.print("Parsed - Right: "); Serial.print(velocity_right);
      Serial.print(", Left: "); Serial.println(velocity_left);
    } else {
      Serial.println("Error: Invalid ROS message format");
    }
  }
}

void updateMotorPositions() {
  float right_enc = odrive.GetPosition(AXIS_ID_RIGHT);
  float left_enc = odrive.GetPosition(AXIS_ID_LEFT);

  if (!isnan(right_enc) && !isnan(left_enc)) {
    right_pos += ((right_enc - old_right_enc) / ENCODER_RESOLUTION) * 360;
    old_right_enc = right_enc;
    left_pos += ((left_enc - old_left_enc) / ENCODER_RESOLUTION) * 360;
    old_left_enc = left_enc;
  } else {
    Serial.println("Warning: Invalid encoder readings");
  }
}

void sendFeedbackToROS() {
  float vel_right = odrive.GetVelocity(AXIS_ID_RIGHT);
  float vel_left = odrive.GetVelocity(AXIS_ID_LEFT);

  if (!isnan(vel_right) && !isnan(vel_left)) {
    Serial.print("r,"); Serial.print(vel_right);
    Serial.print(",l,"); Serial.print(vel_left);
    Serial.print(",rp,"); Serial.print(right_pos);
    Serial.print(",lp,"); Serial.print(left_pos);
    Serial.println(",");
  } else {
    Serial.println("Warning: Invalid velocity readings from ODrive");
  }
}

void loop() {
  processROSData();
  
  // Kiểm tra trạng thái ODrive định kỳ (mỗi 1 giây)
  static unsigned long last_check = 0;
  if (millis() - last_check > 1000) {
    checkODriveStates();
    last_check = millis();
  }

  odrive.SetVelocity(AXIS_ID_RIGHT, velocity_right);
  odrive.SetVelocity(AXIS_ID_LEFT, velocity_left);
  updateMotorPositions();
  sendFeedbackToROS();
  delay(30);
} 