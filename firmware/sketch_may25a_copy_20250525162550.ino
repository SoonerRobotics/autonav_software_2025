#include <SPI.h>
#include <ACAN2515.h>
#include <Adafruit_NeoPixel.h>

static const int BLINK_PERIOD_MS = 500;
static const int DEFAULT_BRIGHTNESS = 50;

static const int CLED_COUNT = 50;
static const int CLED_PIN = 11;
static const int ESTOP_PIN = 28;
static const int MosPin = 19;

static const byte MCP2515_SCK = 2;
static const byte MCP2515_MOSI = 3;
static const byte MCP2515_MISO = 4;
static const byte MCP2515_CS = 5;
static const byte MCP2515_INT = 0;

Adafruit_NeoPixel strip(CLED_COUNT, CLED_PIN, NEO_GRB);

bool is_estopped = false;
bool is_mobility_stopped = false;
bool is_autonomous = false;
bool is_eco = false;

int current_brightness = DEFAULT_BRIGHTNESS;
int color_mode = 0;
int current_blink_period = BLINK_PERIOD_MS;
int current_color = strip.Color(255, 255, 255);

ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);
static const uint32_t QUARTZ_FREQUENCY = 16UL * 1000UL * 1000UL;

typedef struct SafetyLightsMessage {
    uint8_t autonomous : 1;
    uint8_t eco : 1;
    uint8_t mode : 6;
    uint8_t brightness;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t blink_period;
} SafetyLightsMessage;

SafetyLightsMessage last_CAN_message;
CANMessage frame;

void setup() {
    pinMode(MosPin, OUTPUT);
    digitalWrite(MosPin, HIGH);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    SPI.setSCK(MCP2515_SCK);
    SPI.setTX(MCP2515_MOSI);
    SPI.setRX(MCP2515_MISO);
    SPI.setCS(MCP2515_CS);
    SPI.begin();

    strip.begin();
    strip.show();
    strip.setBrightness(current_brightness);

    Serial.println("Configure ACAN2515");
    ACAN2515Settings settings(QUARTZ_FREQUENCY, 100UL * 1000UL);
    settings.mRequestedMode = ACAN2515Settings::NormalMode;
    const uint16_t errorCode = can.begin(settings, [] { can.isr(); });

    if (errorCode == 0) {
        Serial.println("CAN initialized");
    } else {
        Serial.print("Configuration error 0x");
        Serial.println(errorCode, HEX);
    }
}

void loop() {
    is_estopped = !digitalRead(ESTOP_PIN);
    digitalWrite(LED_BUILTIN, is_estopped ? LOW : HIGH);

    // Serial input parsing: red,green,blue,autonomous,eco,mode,brightness,blink_period
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        int values[8];
        int idx = 0;
        char *token = strtok((char *)input.c_str(), ",");
        while (token && idx < 8) {
            values[idx++] = atoi(token);
            token = strtok(NULL, ",");
        }

        if (idx == 8) {
            int red = values[0];
            int green = values[1];
            int blue = values[2];
            is_autonomous = values[3];
            is_eco = values[4];
            color_mode = values[5];
            current_brightness = values[6];
            current_blink_period = values[7];
            current_color = strip.Color(green, red, blue);
        }
    }

    if (can.available()) {
        can.receive(frame);
        Serial.print("Received CAN ");
        Serial.println(frame.id);

        switch (frame.id) {
            case 1:
                is_mobility_stopped = true;
                break;
            case 9:
                is_mobility_stopped = false;
                break;
            case 13:
                last_CAN_message = *(SafetyLightsMessage *)frame.data;
                is_autonomous = last_CAN_message.autonomous;
                is_eco = last_CAN_message.eco;
                color_mode = last_CAN_message.mode;
                current_brightness = last_CAN_message.brightness;
                current_blink_period = 10 * last_CAN_message.blink_period;
                current_color = strip.Color(last_CAN_message.green, last_CAN_message.red, last_CAN_message.blue);
                break;
        }
    }

    strip.setBrightness(current_brightness);

    if (is_estopped) {
        colorSolid();  // stay solid if mobility is stopped
    } else {
        modeSelector();  // otherwise show selected effect
    }
}

void modeSelector() {
    switch (color_mode) {
        case 0: colorSolid(); break;
        case 1: colorFlash(); break;
        case 2: colorFade(); break;
        default: colorSolid(); break;
    }
}

void colorSolid() {
    strip.fill(current_color);
    strip.show();
}

void colorFlash() {
    if ((millis() / current_blink_period) % 2 == 0) {
        strip.fill(current_color);
    } else {
        strip.fill(0);
    }
    strip.show();
}

void colorFade() {
    float fade = abs(sin(millis() / (float)current_blink_period));
    strip.setBrightness((int)(fade * current_brightness));
    strip.fill(current_color);
    strip.show();
}