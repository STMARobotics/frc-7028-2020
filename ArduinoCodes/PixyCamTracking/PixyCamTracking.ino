#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>

Pixy pixy;
Servo servo;
int homeAngle = 50;
int angle = 50;
int leftLim = 130;
int rightLim = 0;

void setup() {
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pinMode(50, OUTPUT);
  servo.attach(8);
  servo.write(angle);
  pixy.init();
}

int area;
int tracker;
int i;
void loop() {

//Functional Code
//  int blocks = pixy.getBlocks();
//  tracker = 0;
//  //Serial.print("Number of Objects: ");
//  //Serial.println(blocks);
//  if (blocks > 1){
//    int base = (pixy.blocks[0].width*pixy.blocks[0].height);
//    for(int i = 1; i<blocks; i++){
//      area = (pixy.blocks[i].width*pixy.blocks[i].height);
//      if (area>=base){
//        tracker = i;
//      } else {
//        base = (pixy.blocks[i].width*pixy.blocks[i].height);
//        tracker = i-1;
//      }  
//    }
//  }
//  //Serial.print("Tracking Object "); Serial.print(tracker); Serial.print(" Center @ ("); Serial.print(pixy.blocks[tracker].x); Serial.print(","); Serial.print(pixy.blocks[tracker].x ); Serial.println(")");
//  int x = pixy.blocks[tracker].x;
//  if (blocks > 0){
//      if (x<140 && angle<=leftLim){
//      digitalWrite(50, LOW);
//      angle += 1;
//      i+=1;
//      servo.write(angle);
//    } else if (x>180 && angle>=rightLim){
//      digitalWrite(50, LOW);
//      angle -= 1;
//      servo.write(angle);
//      i+=1;
//    } else {
//      i+=1;
//      if (i>1000){
//        digitalWrite(50, HIGH);
//        Serial.println("Centered");
//        i = 0;
//      }
//    }
//  }

//Experimental Faster tracking
  int blocks = pixy.getBlocks();
//    if(blocks == 0){
//      servo.write(homeAngle);
//    }else{
      int x = pixy.blocks[0].x;
      while (x<130 && angle<=leftLim){
      //digitalWrite(50, LOW);
      angle += 1;
      servo.write(angle);
      delay(10);
      pixy.getBlocks();
      x = pixy.blocks[0].x;} 
      while (x>190 && angle>=rightLim){
      //digitalWrite(50, LOW);
      angle -= 1;
      servo.write(angle);
      delay(10);
      pixy.getBlocks();
      x = pixy.blocks[0].x;
    }
}
//}
