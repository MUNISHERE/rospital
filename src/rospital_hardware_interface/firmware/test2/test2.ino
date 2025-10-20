#include "ODriveArduino.h"

HardwareSerial& odrive_serial = Serial1;
ODriveArduino odrive(odrive_serial);

const int AXIS_ID_RIGHT = 0;
const int AXIS_ID_LEFT = 1;

float velocity_right = 0.0, velocity_left = 0.0;
float old_right_pos = 0.0, old_left_pos = 0.0;
double right_pos = 0.0, left_pos = 0.0;
const float ENCODER_RESOLUTION = 8192.0 / 360.0;
bool odrive_connected = true;

void setup() {
  Serial.begin(115200);
  odrive_serial.begin(115200);
  delay(100);

  resetODrive();
  setClosedLoopControl();
}

void resetODrive() {
  odrive_serial.print("sr\n");
  delay(2000);
  odrive_connected = true;
}

void setClosedLoopControl() {
  odrive.run_state(AXIS_ID_RIGHT, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false);
  odrive.run_state(AXIS_ID_LEFT, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false);
  delay(1000);
  checkODriveStates();
}

void checkODriveStates() {
  odrive_serial.print("r axis");
  odrive_serial.print(AXIS_ID_RIGHT);
  odrive_serial.print(".current_state\n");
  int state0 = odrive.readInt();

  odrive_serial.print("r axis");
  odrive_serial.print(AXIS_ID_LEFT);
  odrive_serial.print(".current_state\n");
  int state1 = odrive.readInt();

  if (state0 != ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    checkODriveErrors(AXIS_ID_RIGHT);
  }
  if (state1 != ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    checkODriveErrors(AXIS_ID_LEFT);
  }
  odrive_connected = (state0 != ODriveArduino::AXIS_STATE_UNDEFINED && state1 != ODriveArduino::AXIS_STATE_UNDEFINED);
}

void checkODriveErrors(int axis) {
  odrive_serial.print("r axis");
  odrive_serial.print(axis);
  odrive_serial.print(".error\n");
  int axis_error = odrive.readInt();

  odrive_serial.print("r axis");
  odrive_serial.print(axis);
  odrive_serial.print(".motor.error\n");
  int motor_error = odrive.readInt();

  odrive_serial.print("r axis");
  odrive_serial.print(axis);
  odrive_serial.print(".encoder.error\n");
  int encoder_error = odrive.readInt();

  if (axis_error != 0 || motor_error != 0 || encoder_error != 0) {
    odrive_serial.print("w axis");
    odrive_serial.print(axis);
    odrive_serial.print(".error 0\n");

    odrive_serial.print("w axis");
    odrive_serial.print(axis);
    odrive_serial.print(".motor.error 0\n");

    odrive_serial.print("w axis");
    odrive_serial.print(axis);
    odrive_serial.print(".encoder.error 0\n");

    odrive.run_state(axis, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false);

    if (motor_error == 8) {
      odrive.SetVelocity(AXIS_ID_RIGHT, 0.0f);
      odrive.SetVelocity(AXIS_ID_LEFT, 0.0f);
      delay(1000);
    }
  }
}

void processROSData() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    int rIndex = message.indexOf("r");
    int lIndex = message.indexOf("l");
    int commaIndex = message.indexOf(",", rIndex);

    if (rIndex != -1 && lIndex != -1 && commaIndex != -1) {
      velocity_right = message.substring(rIndex + 1, commaIndex).toFloat();
      velocity_left = message.substring(lIndex + 1).toFloat();
    }
  }
}

void updateMotorPositions() {
  if (!odrive_connected) return;

  float right_enc = odrive.GetPosition(AXIS_ID_RIGHT);
  float vel_right = odrive.GetVelocity(AXIS_ID_RIGHT);
  float left_enc = odrive.GetPosition(AXIS_ID_LEFT);
  float vel_left = odrive.GetVelocity(AXIS_ID_LEFT);

  if (!isnan(right_enc) && !isnan(left_enc)) {
    right_pos += ((right_enc - old_right_pos) / ENCODER_RESOLUTION) * 360;
    left_pos += ((left_enc - old_left_pos) / ENCODER_RESOLUTION) * 360;
    old_right_pos = right_enc;
    old_left_pos = left_enc;

    sendFeedbackToROS(vel_right, vel_left);
  }
}

void sendFeedbackToROS(float vel_right, float vel_left) {
  if (!odrive_connected) return;
  Serial.print("r,"); Serial.print(vel_right);
  Serial.print(",l,"); Serial.print(vel_left);
  Serial.print(",rp,"); Serial.print(right_pos);
  Serial.print(",lp,"); Serial.print(left_pos);
  Serial.println(",");
}

void loop() {
  processROSData();

  static unsigned long last_check = 0;
  if (millis() - last_check > 1000) {
    checkODriveStates();
    if (!odrive_connected) {
      resetODrive();
      setClosedLoopControl();
    }
    last_check = millis();
  }

  if (odrive_connected) {
    odrive.SetVelocity(AXIS_ID_RIGHT, velocity_right);
    odrive.SetVelocity(AXIS_ID_LEFT, velocity_left);
    updateMotorPositions();
  }
  delay(30);
}