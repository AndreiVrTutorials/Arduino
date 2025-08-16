#include <Wire.h> 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int percentage = 100;
bool shuttingDown = false;

int batteryWidth = 20;
int batteryHeight = 8;

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  display.clearDisplay();

  // Calculate battery level
  int batteryLevel = map(percentage, 0, 100, 0, batteryWidth);

  // Percentage text
  char text[6];
  sprintf(text, "%d%%", percentage);
  int textWidth = strlen(text) * 6;  // Approx. 6 px per character

  // Coordinates for top-right corner
  int padding = 4;
  int totalWidth = batteryWidth + 4 + textWidth;
  int startX = SCREEN_WIDTH - totalWidth - padding;
  int batteryX = startX;
  int batteryY = padding;
  int textX = batteryX + batteryWidth + 4;
  int textY = batteryY + (batteryHeight - 8) / 2;

  // Draw battery outline and level
  display.drawRect(batteryX, batteryY, batteryWidth, batteryHeight, SSD1306_WHITE);
  display.fillRect(batteryX + 1, batteryY + 1, batteryLevel - 2 > 0 ? batteryLevel - 2 : 0, batteryHeight - 2, SSD1306_WHITE);

  // Display percentage
  display.setCursor(textX, textY);
  display.print(text);

  // Show "Shutting down" if battery reaches 4%
  if (percentage <= 4 && !shuttingDown) {
    shuttingDown = true;
    display.clearDisplay();
    display.setCursor((SCREEN_WIDTH - 60) / 2, SCREEN_HEIGHT / 2);
    display.print("Shutting down...");
    display.display();
    delay(2000);

    percentage = 100;
    shuttingDown = false;
  } else {
    if (!shuttingDown) {
      percentage -= 2;
    }
  }

  display.display();
  delay(e);
}
