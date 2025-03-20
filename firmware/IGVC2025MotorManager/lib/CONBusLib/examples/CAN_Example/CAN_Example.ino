/*
 * CAN_Example
 * 
 * Implements CONBus over CANBus using the ACAN2515 CAN library.
 * 
 * created 2023-04-06
 * by Noah Zemlin
 */

#include <CONBus.h> 
#include <CANBusDriver.h>
#include <ACAN2515.h>

// Setup CANBus using ACAN2515
static const byte MCP2515_SCK = 10;   // SCK input of MCP2515
static const byte MCP2515_MOSI = 11;  // SDI input of MCP2515
static const byte MCP2515_MISO = 8;   // SDO output of MCP2515

static const byte MCP2515_CS = 9;   // CS input of MCP2515
static const byte MCP2515_INT = 7;  // INT output of MCP2515

ACAN2515 can(MCP2515_CS, SPI1, MCP2515_INT);
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;  // 8 MHz

// Setup CONBus variables
CONBus::CONBus conbus;
CONBus::CANBusDriver conbus_can(conbus, 42); // device id 42

bool useOven = false;
int numberOfOvenRacks = 0;
float ovenTemperatureSetpoint = 100;

// Static variables for use in loop/interrupts
CANMessage frame;
CANMessage outFrame;

void setup() {

  while (!Serial) {
    // Wait for a Serial connection...
  }

  Serial.begin(115200);

  // Setup SPI controller (using Raspberry Pi Pico)
  SPI1.setSCK(MCP2515_SCK);
  SPI1.setTX(MCP2515_MOSI);
  SPI1.setRX(MCP2515_MISO);
  SPI1.setCS(MCP2515_CS);
  SPI1.begin();

  Serial.println("Configure ACAN2515");
  ACAN2515Settings settings(QUARTZ_FREQUENCY, 100UL * 1000UL);  // CAN bit rate 100 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select Normal mode
  const uint16_t errorCode = can.begin(settings, onCanRecieve );

  // Create CONBus registers
  // The addresses can be anything from 0 to 255
  conbus.addRegister(0, &useOven);
  conbus.addRegister(4, &numberOfOvenRacks);
  conbus.addRegister(21, &ovenTemperatureSetpoint);
}

void loop() {
  if (conbus_can.isReplyReady()) {
    conbus_can.peekReply(outFrame.id, outFrame.len, outFrame.data);

    bool success = can.tryToSend(outFrame);

    // If we successfully send the message, remove the message from the queue
    if (success) {
      conbus_can.popReply();
    }
  }
}

void onCanRecieve() {
  can.isr();
  can.receive(frame);  

  conbus_can.readCanMessage(frame.id, frame.data);
}