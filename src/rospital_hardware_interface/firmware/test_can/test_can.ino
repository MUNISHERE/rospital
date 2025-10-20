#include <df_can.h>
#include <SPI.h>

const int SPI_CS_PIN = 10;
MCPCAN CAN(SPI_CS_PIN); // Set CS pin

void setup() {
    Serial.begin(115200);
    int count = 50; // Maximum attempts for initializing CAN-BUS

    // Retry CAN initialization until successful or count reaches zero
    while (count--) {
        CAN.init(); // Initialize CAN interface
        if (CAN.begin(CAN_250KBPS) == CAN_OK) {
            Serial.println("DFROBOT's CAN BUS Shield initialized successfully!");
            break;
        } else {
            Serial.println("DFROBOT's CAN BUS Shield initialization failed.");
            Serial.println("Retrying CAN BUS Shield initialization...");
            delay(100);

            if (count == 0) {
                Serial.println("Initialization failed after maximum retries. Please check connections.");
                return;
            }
        }
    }

    // Set control parameters using hex values
    setPositionGain();
    delay(1000);
    setVelocityGain();
    delay(1000);

    // Set Control Mode to Speed Control
    byte controlModeData[8] = {0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
    sendCANMessage(0x00B, 8, controlModeData);
    delay(2000); // Short delay to allow the motor controller to process

    // Enable Closed-Loop Mode
    byte closedLoopData[8] = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendCANMessage(0x007, 8, closedLoopData);
    delay(1000); // Short delay to allow the motor controller to process

    // Set Target Speed (example speed set to 10.0 in IEEE-754 floating-point format)
    byte targetSpeedData[8] = {0x00, 0x00, 0x20, 0x41, 0x00, 0x00, 0x00, 0x00}; // 10.0 as IEEE-754
    sendCANMessage(0x00D, 8, targetSpeedData);
}

void setPositionGain() {
    // Set Position Gain (e.g., 20.0 in IEEE-754 format, encoded in hex)
    byte posGainData[8] = {0x00, 0x00, 0xA0, 0x41, 0x00, 0x00, 0x00, 0x00}; // 20.0 in IEEE-754
    sendCANMessage(0x01A, 8, posGainData);
}

void setVelocityGain() {
    // Set Velocity Gain (e.g., 0.16 in IEEE-754 format, encoded in hex)
    byte velGainData[8] = {0xCD, 0xCC, 0x2C, 0x3E, 0xCD, 0xCC, 0xA6, 0x3E}; // 0.16 in IEEE-754
    sendCANMessage(0x01B, 8, velGainData);
}


void sendCANMessage(long id, byte len, const byte *data) {
    byte sendStatus = CAN.sendMsgBuf(id, 0, len, data);

    if (sendStatus == CAN_OK) {
        Serial.print("Message Sent: ID ");
        Serial.println(id, HEX);
    } else {
        Serial.print("Error Sending Message: ID ");
        Serial.print(id, HEX);
        Serial.print(" - Error Code: ");
        Serial.println(sendStatus);
    }
}

void loop() {
    // Empty loop as there is no need for repeated execution.
    byte targetSpeedData[8] = {0x00, 0x00, 0x20, 0x41, 0x00, 0x00, 0x00, 0x00}; // 10.0 as IEEE-754
    delay(2000);
    sendCANMessage(0x00D, 8, 0);
    delay(2000);
    sendCANMessage(0x00D, 8, targetSpeedData);
}                                                                                  
