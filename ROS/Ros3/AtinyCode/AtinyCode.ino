#include <SoftwareSerial.h>
#include <U8g2lib.h>
#include <Wire.h>

// pin configuration
const uint8_t SERIAL_RX_PIN = PIN_PA1;
const uint8_t SERIAL_TX_PIN = PIN_PA0;
const uint8_t OLED_SCL_PIN = PIN_PA3;
const uint8_t OLED_SDA_PIN = PIN_PA2;

// constants
const uint32_t SERIAL_BAUD_RATE = 57600;
const uint8_t INPUT_BUFFER_SIZE = 64;
const uint8_t MESSAGE_QUEUE_CAPACITY = 9;
const uint16_t SCROLL_DELAY_MS = 50;
const int DISPLAY_Y_POSITION = 20;

SoftwareSerial softSerial(SERIAL_RX_PIN, SERIAL_TX_PIN);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// input buffer
char inputBuffer[INPUT_BUFFER_SIZE];
uint8_t bufferIndex = 0;

// scrolling message
String currentMessage = "";
int scrollX = 0;
int scrollLimit = 0;
unsigned long lastScrollTime = 0;
bool scrollFinished = true;

// message queue
String messageQueue[MESSAGE_QUEUE_CAPACITY];
uint8_t queueStart = 0;
uint8_t queueEnd = 0;

bool isQueueEmpty() {
  return queueStart == queueEnd;
}

bool isQueueFull() {
  return ((queueEnd + 1) % MESSAGE_QUEUE_CAPACITY) == queueStart;
}

void enqueueMessage(const String& message) {
  if (message.length() == 0 || isQueueFull()) return;
  messageQueue[queueEnd] = message;
  queueEnd = (queueEnd + 1) % MESSAGE_QUEUE_CAPACITY;
}

String dequeueMessage() {
  if (isQueueEmpty()) return "";
  String message = messageQueue[queueStart];
  queueStart = (queueStart + 1) % MESSAGE_QUEUE_CAPACITY;
  return message;
}

void drawScrollingMessage() {
  int displayWidth = display.getDisplayWidth();

  display.firstPage();
  do {
    display.setCursor(displayWidth - scrollX, DISPLAY_Y_POSITION);
    display.print(currentMessage);
  } while (display.nextPage());

  if (millis() - lastScrollTime >= SCROLL_DELAY_MS) {
    scrollX++;
    lastScrollTime = millis();

    // wait until the entire message (including last character) scrolls out of view
    if (scrollX > scrollLimit + display.getStrWidth(" ")) {
      scrollFinished = true;
    }
  }
}

void setup() {
  display.begin();
  display.setFont(u8g2_font_6x10_tf);
  softSerial.begin(SERIAL_BAUD_RATE);
}

void loop() {
  // read incoming serial data
  while (softSerial.available()) {
    char incomingChar = softSerial.read();
    if (incomingChar == '!') {
      inputBuffer[bufferIndex] = '\0';
      enqueueMessage(String(inputBuffer));
      bufferIndex = 0;
    } else if (bufferIndex < INPUT_BUFFER_SIZE - 1) {
      inputBuffer[bufferIndex++] = incomingChar;
    }
  }

  // load next message only after the previous one has finished scrolling
  if (scrollFinished && !isQueueEmpty()) {
    currentMessage = dequeueMessage();
    scrollX = 0;
    scrollLimit = display.getStrWidth(currentMessage.c_str()) + display.getDisplayWidth();
    scrollFinished = false;
  }

  if (!scrollFinished) {
    drawScrollingMessage();
  }
}
