#include <Wire.h>
#include <SPI.h>
#include <Pixy.h>

Pixy pixy;

int area;
int i;
int blocks;
int toSend;
byte xPos;
byte yPos;
byte coords[2];


void setup() {
  Wire.begin(0x14);
  Wire.onRequest(sendCoords);
  Serial.begin(9600);
  Serial.print("Starting...\n");
  pixy.init();
}


void loop() { //Get Data to store


}

int getBlockData() {
  blocks = pixy.getBlocks();
  //int xPos[blocks];
  //int yPos[blocks];
  int area[blocks];
  for (int i = 0; i < blocks; i++) {
    area[i] = (pixy.blocks[i].height * pixy.blocks[i - 1].width);
    if (i == 0) {
      toSend = i;
    } else {
      if (area[i - 1] < area[i]) {
        toSend = i;
      }
    }
  }
  return toSend;
}
int modify = 0;


void sendCoords(){
  int select = getBlockData();
  uint8_t xPos = (map((pixy.blocks[select].x), 0, 320, -128, 127));
  uint8_t yPos = (map((pixy.blocks[select].y), 0, 200, -128, 32));
  byte coords[] = {xPos, yPos};
  
  Wire.write(coords, 2);
  
  Serial.print(coords[0]);
  Serial.print(" : ");
  Serial.println(coords[1]);
  
}
