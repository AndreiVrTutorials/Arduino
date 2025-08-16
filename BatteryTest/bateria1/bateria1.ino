#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int porcentaje = 100;  // Inicia en 100%
int delayTime = 20;    // Velocidad del decremento
bool apagando = false;

int batteryWidth = 20;   // Ancho de la batería
int batteryHeight = 4;   // Alto de la batería
int batteryX = (SCREEN_WIDTH - batteryWidth) / 2;  // Posición X centrada
int batteryY = (SCREEN_HEIGHT - batteryHeight) / 2; // Posición Y centrada
int batteryLevel = batteryWidth;  // Inicia al 100%

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  display.clearDisplay();

  // Mostrar el mensaje en la esquina superior derecha
  display.setCursor(SCREEN_WIDTH - 70, 0);
  display.drawRect(batteryX, batteryY, batteryWidth, batteryHeight, SSD1306_WHITE);  // Batería exterior
  display.fillRect(batteryX + 1, batteryY + 1, batteryLevel - 2, batteryHeight - 2, SSD1306_WHITE);  // Batería interior (nivel)
  display.print(porcentaje);
  display.print("%");

  // Si el porcentaje llega a 4%, cambiar a "Apagando"
  if (porcentaje <= 4 && !apagando) {
    apagando = true;
    display.clearDisplay();
    display.setCursor(SCREEN_WIDTH / 2 - 20, SCREEN_HEIGHT / 2);
    display.print("Apagando");
    display.display();
    delay(2000);  // Pausa de 2 segundos

    // Reiniciar a 100% después de apagar
    porcentaje = 100;
    apagando = false;
  } else {
    // Decrementar porcentaje si no está apagando
    if (!apagando) {
      delay(100);  
      porcentaje -= 2;  // Restar 2 en cada ciclo
    }
  }

  // Refrescar la pantalla
  display.display();

  delay(delayTime);
}
