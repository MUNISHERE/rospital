#include <df_can.h>
#include <SPI.h>

// Định nghĩa chân CS của MCP2515
const int SPI_CS_PIN = 10;
MCPCAN CAN(SPI_CS_PIN);

// Định nghĩa Node ID của ODrive
const int AXIS_ID_RIGHT = 0;
const int AXIS_ID_LEFT = 1;

// Biến lưu trữ
float velocity_right = 0.0, velocity_left = 0.0;
double right_pos = 0.0, left_pos = 0.0;

// Biến trạng thái của ODrive
uint8_t right_state = 0, left_state = 0;

// Định nghĩa các hằng số cần thiết
#define CAN_OK 0
#define CAN_FAIL 1
#define PI 3.1415926535897932384626433832795

void setup() {
  Serial.begin(115200);

  // Khởi tạo CAN với tốc độ 250kbps
  int count = 50;
  while (count--) {
    CAN.init();
    if (CAN.begin(CAN_250KBPS) == CAN_OK) break;
    delay(100);
    if (count == 0) while (true);
  }

  // Flush CAN RX buffer to remove old messages
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long id;
    byte len;
    byte buf[8];
    CAN.readMsgBufID(&id, &len, buf);
  }
  // Thiết lập chế độ điều khiển
  setControllerMode();
  delay(1000);

  // Xóa lỗi trước khi thiết lập Closed Loop Control
  clearErrors();
  delay(1000);

  // Kiểm tra lỗi ban đầu
  checkODriveErrors(AXIS_ID_RIGHT);
  checkODriveErrors(AXIS_ID_LEFT);
  delay(1000);

  // Đặt trạng thái điều khiển vòng kín
  setClosedLoopControl();
}

void setControllerMode() {
  byte controlModeData[8] = {0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
  sendCANMessage(AXIS_ID_RIGHT, 0x00B, 8, controlModeData);
  sendCANMessage(AXIS_ID_LEFT, 0x00B, 8, controlModeData);
}

void clearErrors() {
  byte clearErrorData[1] = {0x01};
  sendCANMessage(AXIS_ID_RIGHT, 0x018, 1, clearErrorData);
  sendCANMessage(AXIS_ID_LEFT, 0x018, 1, clearErrorData);
}

void setClosedLoopControl() {
  byte closedLoopData[8] = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendCANMessage(AXIS_ID_RIGHT, 0x007, 8, closedLoopData);
  sendCANMessage(AXIS_ID_LEFT, 0x007, 8, closedLoopData);

  waitForClosedLoop(AXIS_ID_RIGHT);
  waitForClosedLoop(AXIS_ID_LEFT);
}

void waitForClosedLoop(int axis) {
  unsigned long startTime = millis();
  bool stateChanged = false;
  while (millis() - startTime < 5000) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      unsigned long id;
      uint8_t len;
      byte data[8];
      CAN.readMsgBuf(&len, data);
      id = CAN.getCanId();
      if (id == (axis << 5 | 0x01) && len >= 5) {
        uint8_t state = data[4];
        if (axis == AXIS_ID_RIGHT) right_state = state;
        if (axis == AXIS_ID_LEFT) left_state = state;
        if (state == 8) {
          stateChanged = true;
          break;
        }
      }
    }
    delay(10);
  }
}

void checkODriveErrors(int axis) {
  requestCANMessage(axis, 0x003);
  delay(100);
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long id;
    uint8_t len;
    byte data[8];
    CAN.readMsgBuf(&len, data);
    id = CAN.getCanId();
    if (id == (axis << 5 | 0x03)) {
      for (int i = 0; i < len; i++) {
        Serial.print(data[i], HEX);
        Serial.print(" ");
      }
    }
  }
}

void sendCANMessage(uint8_t node_id, uint16_t command_id, byte len, const byte *data) {
  uint32_t can_id = (node_id << 5) | command_id;
  int retry = 3;
  while (retry--) {
    byte sendStatus = CAN.sendMsgBuf(can_id, 0, len, const_cast<byte*>(data));
    if (sendStatus == CAN_OK) {
      return;
    }
    delay(10);
  }
  Serial.println("DEBUG: Failed to send CAN message");
}
void requestCANMessage(uint8_t node_id, uint16_t command_id) {
  uint32_t can_id = (node_id << 5) | command_id;
  CAN.sendMsgBuf(can_id, 0, 0, 1, nullptr);
}

bool waitForCANMessage(uint8_t node_id, uint16_t command_id, unsigned long timeout_ms, byte* data_out, uint8_t* len_out) {
  unsigned long startTime = millis();
  unsigned long expected_id = (node_id << 5) | command_id;
  while (millis() - startTime < timeout_ms) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      unsigned long id;
      uint8_t len;
      byte data[8];
      CAN.readMsgBuf(&len, data);
      id = CAN.getCanId();
      if (len > 0 && id == expected_id) {
        *len_out = len;
        memcpy(data_out, data, len);
        return true;
      }
    }
  }
  return false;
}

void receiveCANMessage(unsigned long* id, byte* len, byte* data) {
  byte status = CAN.checkReceive();
  if (status == CAN_NOMSG || CAN.checkError() == CAN_CTRLERROR) {
    *len = 0;
    return;
  }
  CAN.readMsgBufID(id, len, data);
}

void processROSData() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    message.trim();

    int rIndex = message.indexOf("r");
    int lIndex = message.indexOf("l");
    if (rIndex != -1 && lIndex != -1 && lIndex > rIndex) {
      int commaIndex = message.indexOf(",", rIndex);
      int lCommaIndex = message.indexOf(",", lIndex);

      String rightStr = message.substring(rIndex + 1, commaIndex);
      String leftStr = message.substring(lIndex + 1, lCommaIndex != -1 ? lCommaIndex : message.length());

      float rightVel = rightStr.toFloat();
      float leftVel = leftStr.toFloat();

      if (!isnan(rightVel) && !isnan(leftVel)) {
        velocity_right = rightVel;
        velocity_left = leftVel;

        byte velRightData[8];
        *(float*)velRightData = velocity_right;
        velRightData[4] = velRightData[5] = velRightData[6] = velRightData[7] = 0;
        sendCANMessage(AXIS_ID_RIGHT, 0x00D, 8, velRightData);

        byte velLeftData[8];
        *(float*)velLeftData = velocity_left;
        velLeftData[4] = velLeftData[5] = velLeftData[6] = velLeftData[7] = 0;
        sendCANMessage(AXIS_ID_LEFT, 0x00D, 8, velLeftData);
      } else {
        velocity_right = 0.0;
        velocity_left = 0.0;
      }
    } else {
      velocity_right = 0.0;
      velocity_left = 0.0;
    }
  }
}

void updateMotorPositions() {
  static unsigned long last_update = 0;
  if (millis() - last_update < 100) return;

  requestCANMessage(AXIS_ID_RIGHT, 0x009);
  requestCANMessage(AXIS_ID_LEFT, 0x009);

  byte right_data[8];
  uint8_t right_len = 0;
  if (waitForCANMessage(AXIS_ID_RIGHT, 0x009, 100, right_data, &right_len)) {
    if (right_len >= 8) {
      float right_enc_turns = *(float*)right_data;    // Pos_Estimate (rev)
      float right_vel_turns_per_sec = *(float*)&right_data[4]; // Vel_Estimate (rev/s)
      if (!isnan(right_enc_turns) && !isnan(right_vel_turns_per_sec)) {
        right_pos = right_enc_turns * 2 * PI; // Vị trí (m)
        velocity_right = right_vel_turns_per_sec * 2 * PI; // Vận tốc (m/s)
      }
    }
  }

  byte left_data[8];
  uint8_t left_len = 0;
  if (waitForCANMessage(AXIS_ID_LEFT, 0x009, 100, left_data, &left_len)) {
    if (left_len >= 8) {
      float left_enc_turns = *(float*)left_data;      // Pos_Estimate (rev)
      float left_vel_turns_per_sec = *(float*)&left_data[4]; // Vel_Estimate (rev/s)
      if (!isnan(left_enc_turns) && !isnan(left_vel_turns_per_sec)) {
        left_pos = left_enc_turns * 2 * PI; // Vị trí (m)
        velocity_left = left_vel_turns_per_sec * 2 * PI; // Vận tốc (m/s)
      }
    }
  }

  last_update = millis();
}

void sendFeedbackToROS() {
  static unsigned long last_feedback = 0;
  if (millis() - last_feedback >= 100) {
    if (isnan(velocity_right)) velocity_right = 0.0;
    if (isnan(velocity_left)) velocity_left = 0.0;
    if (isnan(right_pos)) right_pos = 0.0;
    if (isnan(left_pos)) left_pos = 0.0;

    String message = "r" + String(velocity_right, 2) + ",l" + String(velocity_left, 2) +
                     ",rp" + String(right_pos, 2) + ",lp" + String(left_pos, 2) + ",";
    Serial.println(message);
    last_feedback = millis();
  }
}

void loop() {
  static unsigned long last_can_check = 0;
  static unsigned long last_cycle = 0;
  static bool initial_setup_done = false;

  // Kiểm tra trạng thái ODrive ban đầu
  if (!initial_setup_done && millis() - last_can_check >= 100) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      unsigned long id;
      uint8_t len;
      byte data[8];
      CAN.readMsgBuf(&len, data);
      id = CAN.getCanId();

      if (id == (AXIS_ID_RIGHT << 5 | 0x01)) {
        right_state = data[4];
      } else if (id == (AXIS_ID_LEFT << 5 | 0x01)) {
        left_state = data[4];
      }
    }
    if (right_state != 8 || left_state != 8) {
      Serial.println("DEBUG: Setting Closed Loop Control");
      setClosedLoopControl();
    } else {
      initial_setup_done = true;
    }
    last_can_check = millis();
  }

  // Chu kỳ chính
  if (millis() - last_cycle >= 20) {
    updateMotorPositions();
    sendFeedbackToROS();
    processROSData();
    last_cycle = millis();
  }
}