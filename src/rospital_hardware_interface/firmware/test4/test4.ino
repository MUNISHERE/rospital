#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <ODriveMCPCAN.hpp>
#include <ODriveCAN.h>
#define PI 3.141592653589793

// CAN bus baudrate
#define CAN_BAUDRATE 500000

// ODrive node IDs
#define AXIS_ID_RIGHT 0
#define AXIS_ID_LEFT 1

// MCP2515 settings
#define MCP2515_CS 10 // Chip Select
#define MCP2515_CLK_HZ 8000000 // 8MHz crystal

// Khởi tạo CAN interface
MCP2515 can_intf(MCP2515_CS);

// Instantiate ODrive objects
ODriveCAN odrive_right(wrap_can_intf(can_intf), AXIS_ID_RIGHT);
ODriveCAN odrive_left(wrap_can_intf(can_intf), AXIS_ID_LEFT);
ODriveCAN* odrives[] = {&odrive_right, &odrive_left};

// User data for ODrive
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

ODriveUserData right_user_data;
ODriveUserData left_user_data;

// Global variables for ROS2
float velocity_right = 0.0, velocity_left = 0.0;
float old_right_enc = 0.0, old_left_enc = 0.0;
double right_pos = 0.0, left_pos = 0.0;

// CAN receive callback
void onCanMessage(const CanMsg& msg) {
  for (auto odrive : odrives) {
    onReceive(msg, *odrive);
  }
}

// Heartbeat callback
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Feedback callback
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Setup CAN
bool setupCan() {
  SPI.begin();
  can_intf.reset();
  can_intf.setBitrate(CAN_BAUDRATE, MCP_8MHZ);
  can_intf.setNormalMode();
  return true;
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  Serial.println("Starting ODriveCAN");

  // Register callbacks
  odrive_right.onFeedback(onFeedback, &right_user_data);
  odrive_right.onStatus(onHeartbeat, &right_user_data);
  odrive_left.onFeedback(onFeedback, &left_user_data);
  odrive_left.onStatus(onHeartbeat, &left_user_data);

  // Initialize CAN
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true);
  }

  // Wait for ODrives
  Serial.println("Waiting for ODrives...");
  while (!right_user_data.received_heartbeat || !left_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }
  Serial.println("Found ODrives");

  // Set closed-loop control
  setClosedLoopControl();
}

void setClosedLoopControl() {
  Serial.println("Enabling closed loop control...");
  // Axis 0
  while (right_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive_right.clearErrors();
    delay(1);
    odrive_right.setControllerMode(2, 1); // CONTROL_MODE_VELOCITY_CONTROL = 2, INPUT_MODE_PASSTHROUGH = 1
    odrive_right.setState(8); // AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }

  // Axis 1
  while (left_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive_left.clearErrors();
    delay(1);
    odrive_left.setControllerMode(2, 1);
    odrive_left.setState(8);
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }
  Serial.println("ODrives in closed loop control");
}

void checkODriveStates() {
  Serial.println("Checking ODrive states...");
  // Axis 0
  if (right_user_data.received_heartbeat) {
    Serial.println("Axis 0 responding");
    ODriveCAN::Get_Error_msg_t error0;
    if (odrive_right.getError(error0, 200) && error0.axis_error != 0) {
      checkODriveErrors(AXIS_ID_RIGHT);
    }
  } else {
    Serial.println("Error: No heartbeat from Axis 0");
  }

  // Axis 1
  if (left_user_data.received_heartbeat) {
    Serial.println("Axis 1 responding");
    ODriveCAN::Get_Error_msg_t error1;
    if (odrive_left.getError(error1, 200) && error1.axis_error != 0) {
      checkODriveErrors(AXIS_ID_LEFT);
    }
  } else {
    Serial.println("Error: No heartbeat from Axis 1");
  }
}

void checkODriveErrors(int axis) {
  Serial.print("Checking errors for Axis "); Serial.println(axis);
  ODriveCAN::Get_Error_msg_t error;
  ODriveCAN& odrive = (axis == AXIS_ID_RIGHT) ? odrive_right : odrive_left;
  if (odrive.getError(error, 200)) {
    Serial.print("Axis "); Serial.print(axis); Serial.print(" error: "); Serial.println(error.axis_error);
    if (error.axis_error != 0) {
      clearErrors(axis);
    }
  } else {
    Serial.print("Error: No error response from Axis "); Serial.println(axis);
  }
}

void clearErrors(int axis) {
  Serial.println("Clearing errors...");
  ODriveCAN& odrive = (axis == AXIS_ID_RIGHT) ? odrive_right : odrive_left;
  odrive.clearErrors();
  delay(10);
  odrive.setControllerMode(2, 1);
  odrive.setState(8);
}

void processROSData() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    Serial.print("Received from ROS: "); Serial.println(message);
    int rIndex = message.indexOf("r");
    int lIndex = message.indexOf("l");
    int commaIndex = message.indexOf(",", rIndex);

    if (rIndex != -1 && lIndex != -1 && commaIndex != -1) {
      velocity_right = message.substring(rIndex + 1, commaIndex).toFloat() / (2 * PI); // rad/s -> turns/s
      velocity_left = message.substring(lIndex + 1).toFloat() / (2 * PI); // rad/s -> turns/s
      velocity_right = constrain(velocity_right, -10.0, 10.0);
      velocity_left = constrain(velocity_left, -10.0, 10.0);
      Serial.print("Parsed - Right: "); Serial.print(velocity_right);
      Serial.print(", Left: "); Serial.println(velocity_left);
    } else {
      Serial.println("Error: Invalid ROS message format");
    }
  }
}

void updateMotorPositions() {
  // Axis 0
  if (right_user_data.received_feedback) {
    float right_enc = right_user_data.last_feedback.pos_estimate;
    if (!isnan(right_enc)) {
      right_pos += (right_enc - old_right_enc) * 2 * PI; // Tính radian
      old_right_enc = right_enc;
    }
    right_user_data.received_feedback = false;
  }

  // Axis 1
  if (left_user_data.received_feedback) {
    float left_enc = left_user_data.last_feedback.pos_estimate;
    if (!isnan(left_enc)) {
      left_pos += (left_enc - old_left_enc) * 2 * PI; // Tính radian
      old_left_enc = left_enc;
    }
    left_user_data.received_feedback = false;
  }
}

void sendFeedbackToROS() {
  float vel_right = 0.0, vel_left = 0.0;
  if (right_user_data.received_feedback) {
    vel_right = right_user_data.last_feedback.vel_estimate * 2 * PI; // turns/s -> rad/s
  }
  if (left_user_data.received_feedback) {
    vel_left = left_user_data.last_feedback.vel_estimate * 2 * PI; // turns/s -> rad/s
  }

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
  pumpEvents(can_intf); // Handle CAN messages
  processROSData();

  // Gửi lệnh tốc độ
  odrive_right.setVelocity(velocity_right, 0.0);
  odrive_left.setVelocity(velocity_left, 0.0);

  // Kiểm tra trạng thái định kỳ
  static unsigned long last_check = 0;
  if (millis() - last_check > 1000) {
    checkODriveStates();
    last_check = millis();
  }

  updateMotorPositions();
  sendFeedbackToROS();
  delay(2); // ~500Hz
}