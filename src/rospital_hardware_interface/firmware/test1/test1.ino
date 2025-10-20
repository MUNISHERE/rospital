#include <SPI.h>
#include <mcp_can.h>

#define SPI_CS_PIN 9

MCP_CAN CAN(SPI_CS_PIN);

void setup() {
  Serial.begin(115200);
  
  while (CAN_OK != CAN.begin(CAN_250KBPS, MCP_16MHz)) {
    Serial.println("CAN init fail, retry...");
    delay(100);
  }

  Serial.println("CAN init ok");
  CAN.setMode(MODE_NORMAL);
}

void loop() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    byte len = 0;
    byte buf[8];
    unsigned long recvId;

    CAN.readMsgBuf(&len, buf);
    recvId = CAN.getCanId();

    Serial.print("ID: ");
    Serial.print(recvId, HEX);
    Serial.print(" Data: ");
    for (int i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}
