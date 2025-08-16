#include <U8g2lib.h>  
#include <Wire.h>

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

int percentage = 100; 
bool shuttingDown = false;
bool warningShown = false;

const int batteryWidth = 20;
const int batteryHeight = 8;

const int screenWidth = 128;
const int screenHeight = 32;


  // warning box size
  const int warningWidth = 100;
  const int warningHeight = 24;

  // animation speed
  const int animationDelay = 50;

  // relative offsets for the triangle inside the warning box
  const int triangleWarningOffsetX = 6;  
  const int triangleWarningOffsetY = 6;  

  // vertex A (bottom right)
  const int triangleWarningAxOffsetX = triangleWarningOffsetX;
  const int triangleWarningAxOffsetY = triangleWarningOffsetY + 8;

  // vertex B (top)
  const int triangleWarningBxOffsetX = triangleWarningOffsetX + 6;
  const int triangleWarningBxOffsetY = triangleWarningOffsetY;

  // vertex C (bottom left)
  const int triangleWarningCxOffsetX = triangleWarningOffsetX + 12;
  const int triangleWarningCxOffsetY = triangleWarningOffsetY + 8;

  // center of the warning box horizontally
  int warningX = (screenWidth - warningWidth) / 2; 

  // calculate absolute positions of the triangle 
  int triangleWarningAx = warningX + triangleWarningAxOffsetX;
  int triangleWarningAy = triangleWarningAxOffsetY;

  int triangleWarningBx = warningX + triangleWarningBxOffsetX;
  int triangleWarningBy = triangleWarningBxOffsetY;

  int triangleWarningCx = warningX + triangleWarningCxOffsetX;
  int triangleWarningCy = triangleWarningCxOffsetY;
  

void drawWarning(int y){
  const int rightMarginX = 20;
  const int rightMarginY = 14;

  const String textWarning = "LOW BATTERY";

  // draw the warning box
  u8g2.drawFrame(warningX, y, warningWidth, warningHeight);

  // draw the triangle in the warning box
  u8g2.drawTriangle(triangleWarningAx, y + triangleWarningAy, 
                    triangleWarningBx, y + triangleWarningBy, 
                    triangleWarningCx, y + triangleWarningCy);

  // write the warning text to the right of the triangle
  u8g2.setCursor(warningX + triangleWarningOffsetX + rightMarginX, y + rightMarginY);
  u8g2.print(textWarning);
}

void showLowBatteryWarning() {
  if(warningShown)
    return;
  // entrance animation loop (moving down)
  for (int y = -warningHeight; y <= 10; y += 2) {  // animate from above the screen
    u8g2.firstPage();
    do {
      drawWarning(y);
    } while (u8g2.nextPage());  // draw the page on the screen

    delay(animationDelay);  // animation delay between each step
  }

  delay(1000);  // keep the warning visible for 1 second

  // Exit animation loop (moving upwards)
  for (int y = 10; y >= -warningHeight; y -= 2) { // does the same thing as the other for loop but in reverse
    u8g2.firstPage();
    do {
      drawWarning(y);
    } while (u8g2.nextPage());  

    delay(animationDelay);  
  }

  delay(1000);  
  warningShown = true; // marks that the warning has been shown
}

void setup() {
  u8g2.begin(); // initialize the OLED screen
  u8g2.setFont(u8g2_font_6x12_tr); // set the font to use for text
}

void updateBatteryStatus() {
  int batteryLevel = map(percentage, 0, 100, 0, batteryWidth); // converts the percentage into a proportional value to represent the battery bar

  // formats the percentage text as "xx%"
  char text[6];
  sprintf(text, "%d%%", percentage);

  const int widthCharacterPx = 6; 
  const int batteryToTextSpacing = 4;

  int textWidth = strlen(text) * widthCharacterPx; // calculates the text width in pixels (each character is 6px wide) --> otra constante
  int paddingX = 4; // spacing between elements
  int totalWidth = batteryWidth + batteryToTextSpacing + textWidth; // calculates the total width of the battery and the text --> 4 margen
  int startX = screenWidth - totalWidth - paddingX; // horizontal position to align the content to the right 

  // coordinates for drawing the battery and the text
  int batteryX = startX;
  int batteryY = paddingX;
  int textX = batteryX + batteryWidth + batteryToTextSpacing;
  int textY = batteryY + 7;

  // starts the drawing process
  u8g2.firstPage();
  do {
    u8g2.drawFrame(batteryX, batteryY, batteryWidth, batteryHeight); // draws the battery outline
    u8g2.drawBox(batteryX + 1, batteryY + 1, max(batteryLevel - 2, 0), batteryHeight - 2); // draws the inner charge bar that decreases over time

    // draws the battery percentage text
    u8g2.setCursor(textX, textY);
    u8g2.print(text);

  } while (u8g2.nextPage());

  // displays the warning message if battery is 20% or below and it hasn't been shown yet
  if (percentage <= 20) {
    showLowBatteryWarning();
  }

  if (percentage <= 4) {
    turnOff();

    delay(2000); // time the "shutting down" message stays on screen

    turnOn();
  }

  delay(500); // time delay before reducing the battery percentage
  percentage -= 2; // reduce battery by 2%
}

void turnOn(){
  // resets battery to 100% and clears flags
    percentage = 100;
    shuttingDown = false;
    warningShown = false;
}

void turnOff(){
  if(shuttingDown)
    return;
  const String textTurnOff = "Shutting down...";
  shuttingDown = true; // marks that the system is shutting down
  // displays "shutting down" if battery is 4% or lower
    u8g2.firstPage();
    do {
     // calculating the width of the text
    int textPowerOff = u8g2.getStrWidth(textTurnOff.c_str()); 

    // Center the text
    u8g2.setCursor((screenWidth - textPowerOff) / 2, screenHeight);
    u8g2.print(textTurnOff);
    } while (u8g2.nextPage());
}

void loop() {
  updateBatteryStatus(); // calling the "updateBatteryStatus()" function
}
