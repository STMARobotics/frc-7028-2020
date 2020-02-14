#include <Wire.h>
#include <Pixy2.h>

Pixy2 pixy;

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

int index;
int age;

int getBlockData() {
  blocks = pixy.ccc.getBlocks();
  //int xPos[blocks];
  //int yPos[blocks];
  int area[blocks];
  for (int i = 0; i < blocks; i++) {
    area[i] = (pixy.ccc.blocks[i].m_height * pixy.ccc.blocks[i - 1].m_width);
    if(index == pixy.ccc.blocks[i].m_index){
      toSend = i;
      return toSend;
    } else if (i == 0) {
      toSend = i;
      index = i;
    } else {
      if (area[i - 1] < area[i]) {
        toSend = i;
        index = i;
      }
    }
  }
  return toSend;
}

int modify = 0;


void sendCoords(){
  int select = getBlockData();
  uint8_t xPos = (map((pixy.ccc.blocks[select].m_x), 0, 316, -128, 127));
  uint8_t yPos = (map((pixy.ccc.blocks[select].m_y), 0, 208, -128, 32));
  uint8_t width = (map((pixy.ccc.blocks[select].m_width), 0, 316, -128, 127));
  uint8_t height = (map((pixy.ccc.blocks[select].m_height), 0, 208, -128, 32));
  byte coords[] = {xPos, yPos, width, height};
  
  Wire.write(coords, 4);
  
//  Serial.print(coords[0]);
//  Serial.print(" : ");
//  Serial.println(coords[1]);
  
}
