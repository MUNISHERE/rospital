#include <SPI.h>
#include <ACAN2515.h>
#define PI 3.141592653589793

// Cấu hình chân MCP2515
static const byte MCP2515_CS = 10;  // Chip Select
static const byte MCP2515_INT = 2;   // Interrupt (phải dùng chân hỗ trợ interrupt)
ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);
const uint32_t QUARTZ_FREQUENCY = 8 * 1000 * 1000; // 8MHz thạch anh

const int AXIS_ID_RIGHT = 0;
const int AXIS_ID_LEFT = 1;
float velocity_right = 0.0, velocity_left = 0.0;
float old_right_enc = 0.0, old_left_enc = 0.0;
double right_pos = 0.0, left_pos = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Đợi Serial sẵn sàng
  SPI.begin();

  // Khởi tạo CAN
  ACAN2515Settings settings(QUARTZ_FREQUENCY, 500 * 1000); // 500kbps
  settings.mRequestedMode = ACAN2515Settings::NormalMode;
  settings.mReceiveBufferSize = 16; // Tăng buffer để xử lý tin nhắn ODrive
  settings.mTransmitBuffer0Size = 8;
  uint16_t errorCode = can.begin(settings, [] { can.isr(); });
  if (errorCode == 0) {
    Serial.println("CAN Initialized");
  } else {
    Serial.print("CAN Init Failed: 0x");
    Serial.println(errorCode, HEX);
    while (1); // Dừng nếu lỗi
  }

  // Đặt trạng thái điều khiển vòng kín
  setClosedLoopControl();
}

void setClosedLoopControl() {
  CANMessage msg;
  uint32_t state = 8; // AXIS_STATE_CLOSED_LOOP_CONTROL
  msg.id = (AXIS_ID_RIGHT << 5) | 0x007; // CMD_SET_STATE
  msg.len = 4;
  memcpy(msg.data, &state, 4);
  while (!can.tryToSend(msg)) delay(1); // Chờ đến khi gửi được

  msg.id = (AXIS_ID_LEFT << 5) | 0x007;
  while (!can.tryToSend(msg)) delay(1);

  delay(1000);
  checkODriveStates();
}

void checkODriveStates() {
  Serial.println("Checking ODrive states...");
  CANMessage msg;
  // Axis 0
  msg.id = (AXIS_ID_RIGHT << 5) | 0x009; // CMD_GET_STATE
  msg.len = 0;
  while (!can.tryToSend(msg)) delay(1);
  unsigned long start = millis();
  while (!can.available() && millis() - start < 200);
  if (can.available()) {
    can.receive(msg);
    if (msg.len == 4) {
      uint32_t state = *(uint32_t*)msg.data;
      Serial.print("Axis 0 state: "); Serial.println(state);
      if (state != 8) checkODriveErrors(AXIS_ID_RIGHT);
    } else {
      Serial.println("Error: Invalid response length from Axis 0");
    }
  } else {
    Serial.println("Error: No response from Axis 0");
  }

  // Axis 1
  msg.id = (AXIS_ID_LEFT << 5) | 0x009;
  while (!can.tryToSend(msg)) delay(1);
  start = millis();
  while (!can.available() && millis() - start < 200);
  if (can.available()) {
    can.receive(msg);
    if (msg.len == 4) {
      uint32_t state = *(uint32_t*)msg.data;
      Serial.print("Axis 1 state: "); Serial.println(state);
      if (state != 8) checkODriveErrors(AXIS_ID_LEFT);
    } else {
      Serial.println("Error: Invalid response length from Axis 1");
    }
  } else {
    Serial.println("Error: No response from Axis 1");
  }
}

void checkODriveErrors(int axis) {
  Serial.print("Checking errors for Axis "); Serial.println(axis);
  CANMessage msg;
  msg.id = (axis << 5) | 0x017; // CMD_GET_ERROR
  msg.len = 0;
  while (!can.tryToSend(msg)) delay(1);
  unsigned long start = millis();
  while (!can.available() && millis() - start < 200);
  if (can.available()) {
    can.receive(msg);
    if (msg.len == 4) {
      uint32_t error = *(uint32_t*)msg.data;
      Serial.print("Axis "); Serial.print(axis); Serial.print(" error: "); Serial.println(error);
      if (error != 0) {
        clearErrors(axis);
      }
    } else {
      Serial.println("Error: Invalid error response length");
    }
  } else {
    Serial.print("Error: No error response from Axis "); Serial.println(axis);
  }
}

void clearErrors(int axis) {
  Serial.println("Clearing errors...");
  CANMessage msg;
  msg.id = (axis << 5) | 0x018; // CMD_CLEAR_ERRORS
  msg.len = 0;
  while (!can.tryToSend(msg)) delay(1);
  delay(10);
  msg.id = (axis << 5) | 0x007;
  msg.len = 4;
  uint32_t state = 8;
  memcpy(msg.data, &state, 4);
  while (!can.tryToSend(msg)) delay(1);
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
      velocity_right = constrain(velocity_right, -10.0, 10.0); // Giới hạn an toàn
      velocity_left = constrain(velocity_left, -10.0, 10.0);
      Serial.print("Parsed - Right: "); Serial.print(velocity_right);
      Serial.print(", Left: "); Serial.println(velocity_left);
    } else {
      Serial.println("Error: Invalid ROS message format");
    }
  }
}

void updateMotorPositions() {
  CANMessage msg;
  // Đọc vị trí Axis 0
  msg.id = (AXIS_ID_RIGHT << 5) | 0x00C; // CMD_GET_ENCODER_POS
  msg.len = 0;
  while (!can.tryToSend(msg)) delay(1);
  unsigned long start = millis();
  while (!can.available() && millis() - start < 50);
  if (can.available()) {
    can.receive(msg);
    if (msg.len == 4) {
      float right_enc = *(float*)msg.data;
      if (!isnan(right_enc)) {
        right_pos += (right_enc - old_right_enc) * 2 * PI; // Tính radian
        old_right_enc = right_enc;
      }
    }
  }

  // Đọc vị trí Axis 1
  msg.id = (AXIS_ID_LEFT << 5) | 0x00C;
  while (!can.tryToSend(msg)) delay(1);
  start = millis();
  while (!can.available() && millis() - start < 50);
  if (can.available()) {
    can.receive(msg);
    if (msg.len == 4) {
      float left_enc = *(float*)msg.data;
      if (!isnan(left_enc)) {
        left_pos += (left_enc - old_left_enc) * 2 * PI; // Tính radian
        old_left_enc = left_enc;
      }
    }
  }
}

void sendFeedbackToROS() {
  CANMessage msg;
  // Đọc vận tốc Axis 0
  msg.id = (AXIS_ID_RIGHT << 5) | 0x00D; // CMD_GET_VEL
  msg.len = 0;
  while (!can.tryToSend(msg)) delay(1);
  unsigned long start = millis();
  while (!can.available() && millis() - start < 50);
  float vel_right = 0.0, vel_left = 0.0;
  if (can.available()) {
    can.receive(msg);
    if (msg.len == 4) {
      vel_right = *(float*)msg.data;
    }
  }

  // Đọc vận tốc Axis 1
  msg.id = (AXIS_ID_LEFT << 5) | 0x00D;
  while (!can.tryToSend(msg)) delay(1);
  start = millis();
  while (!can.available() && millis() - start < 50);
  if (can.available()) {
    can.receive(msg);
    if (msg.len == 4) {
      vel_left = *(float*)msg.data;
    }
  }

  // Chuyển đổi đơn vị
  vel_right *= 2 * PI; // turns/s -> rad/s
  vel_left *= 2 * PI;

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

  // Gửi lệnh tốc độ Axis 0
  CANMessage msg;
  msg.id = (AXIS_ID_RIGHT << 5) | 0x00B; // CMD_SET_INPUT_VEL
  msg.len = 8;
  memcpy(msg.data, &velocity_right, 4);
  memset(&msg.data[4], 0, 4);
  while (!can.tryToSend(msg)) delay(1);

  // Gửi lệnh tốc độ Axis 1
  msg.id = (AXIS_ID_LEFT << 5) | 0x00B;
  memcpy(msg.data, &velocity_left, 4);
  while (!can.tryToSend(msg)) delay(1);

  // Kiểm tra trạng thái định kỳ
  static unsigned long last_check = 0;
  if (millis() - last_check > 1000) {
    checkODriveStates();
    last_check = millis();
  }

  updateMotorPositions();
  sendFeedbackToROS();
  delay(2); // Tăng tần suất
}