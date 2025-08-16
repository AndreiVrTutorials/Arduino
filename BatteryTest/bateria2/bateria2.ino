#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int batteryWidth = 100;   // Ancho de la batería
int batteryHeight = 20;   // Alto de la batería
int batteryX = (SCREEN_WIDTH - batteryWidth) / 2;  // Posición X centrada
int batteryY = (SCREEN_HEIGHT - batteryHeight) / 2; // Posición Y centrada
int batteryLevel = batteryWidth;  // Inicia al 100%
bool apagando = false;

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  display.clearDisplay();

  // Dibujar el contorno de la batería
  display.drawRect(batteryX, batteryY, batteryWidth, batteryHeight, SSD1306_WHITE);  // Batería exterior

  // Mostrar el nivel de la batería como un rectángulo dentro
  display.fillRect(batteryX + 1, batteryY + 1, batteryLevel - 2, batteryHeight - 2, SSD1306_WHITE);  // Batería interior (nivel)

  // Si la batería llega a 0%, mostrar "Apagando..."
  if (batteryLevel <= 0 && !apagando) {
    apagando = true;
    display.clearDisplay();
    display.setCursor(SCREEN_WIDTH / 2 - 40, SCREEN_HEIGHT / 2);
    display.print("Apagando...");
    display.display();
    delay(2000);  // Pausa de 2 segundos

    // Reiniciar la batería a 100%
    batteryLevel = batteryWidth;
    apagando = false;
  } else {
    // Decrecer el nivel de la batería (restar 2 píxeles en cada ciclo)
    if (!apagando) {
      batteryLevel -= 2;
    }
  }

  display.display();  // Actualizar la pantalla
  delay(100);  // Controlar la velocidad de la "descarga"
}
