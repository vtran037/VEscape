#include <StandardCplusplus.h>
#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>   // Core graphics library
#include <RGBmatrixPanel.h> // Hardware-specific library
#include <stdlib.h>
#include <vector>

#define CLK 8  
#define LAT A3
#define OE  9
#define A   A0
#define B   A1
#define C   A2

RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, false);

SoftwareSerial atmega(10, 11); // RX, TX

#define PLAYER matrix.Color888(150, 140, 53) //0, 255, 255
#define DEATH matrix.Color888(0, 0, 0)
#define WIN matrix.Color888(0, 40, 0)
#define WALL matrix.Color888(0, 0, 40)


uint8_t receive_USART() {
  while(!atmega.available());
  return atmega.read();
}

void setup() {
  matrix.begin();
  Serial.begin(250000);
  while (!Serial);
  atmega.begin(2400); //Also set on the Atmega side
}

void loop() {
  uint8_t data_USART = receive_USART();
  if(data_USART == 0x00) { 
    matrix.fillScreen(0);
  } 
  else {
    uint16_t ROLE = (uint8_t)(data_USART >> 5);  
    if(ROLE == 1){
      ROLE = WIN;
    }
    else if(ROLE == 2) {
      ROLE = WALL;
    }
    else if(ROLE == 3) {
      ROLE = PLAYER;
    }
    
    int x = receive_USART();
    int y = receive_USART();
      
    if((uint8_t)(data_USART << 3) == 2 << 3) {
      uint16_t piece = (uint8_t)(data_USART >> 5);
      if(piece <= 3) {
        matrix.drawRect(x, y, 2, 2, ROLE);
      }
      else if(piece == 4) { //DEATH
        matrix.drawRect(x, y, 2, 2, WALL);
        matrix.drawRect(x+1, y, 1, 1, DEATH); 
      }
    
    }

  }

  delay(15);
}
