#include <Wire.h>                    // I2C communication library
#include <Adafruit_GFX.h>           // Core graphics library
#include <Adafruit_SSD1306.h>       // OLED driver for SSD1306 displays

//OLED screen size
#define SCREEN_WIDTH 128           
#define SCREEN_HEIGHT 64 

// create an SSD1306 display object connected
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// small heart bitmap (24x21 pixels)
const unsigned char heart_small[] PROGMEM = {
  0b00000110, 0b00000011, 0b00000000, // each row is 24 pixels (3 bytes of 8 bits)
  0b00011111, 0b10011111, 0b00000000,
  0b00111111, 0b11011111, 0b10000000,
  0b01111111, 0b11111111, 0b11000000,
  0b01111111, 0b11111111, 0b11000000,
  0b01111111, 0b11111111, 0b11000000,
  0b00111111, 0b11111111, 0b10000000,
  0b00011111, 0b11111111, 0b00000000,
  0b00001111, 0b11111110, 0b00000000,
  0b00000111, 0b11111100, 0b00000000,
  0b00000011, 0b11111000, 0b00000000,
  0b00000001, 0b11110000, 0b00000000,
  0b00000000, 0b11100000, 0b00000000,
  0b00000000, 0b01000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, // Empty rows for padding
  0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000
};

// large heart bitmap (24x21 pixels)
const unsigned char heart_large[] PROGMEM = {
  0b00000110, 0b00000011, 0b00000000, // each row is 24 pixels (3 bytes of 8 bits)
  0b00011111, 0b10011111, 0b00000000,
  0b00111111, 0b11011111, 0b10000000,
  0b01111111, 0b11111111, 0b11000000,
  0b11111111, 0b11111111, 0b11100000, // more pixels here to make it look "larger"
  0b11111111, 0b11111111, 0b11100000,
  0b11111111, 0b11111111, 0b11100000,
  0b01111111, 0b11111111, 0b11000000,
  0b00111111, 0b11111111, 0b10000000,
  0b00011111, 0b11111111, 0b00000000,
  0b00001111, 0b11111110, 0b00000000,
  0b00000111, 0b11111100, 0b00000000,
  0b00000011, 0b11111000, 0b00000000,
  0b00000001, 0b11110000, 0b00000000,
  0b00000000, 0b11100000, 0b00000000,
  0b00000000, 0b01000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000
};

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize OLED 
  display.clearDisplay();// clear display buffer
}

void loop() {
  // draw large heart
  display.clearDisplay(); // clear screen
  display.drawBitmap(52, 20, heart_large, 24, 21, SSD1306_WHITE); // x=52, y=20: center position; 24x21 is bitmap size; draw heart
  display.display();      
  delay(400); // wait

  // draw small heart
  display.clearDisplay();
  display.drawBitmap(52, 20, heart_small, 24, 21, SSD1306_WHITE); // draw smaller version at same position
  display.display();
  delay(400); // repeat every 400 milliseconds (simulate "beating")
}
