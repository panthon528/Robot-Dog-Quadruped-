#include "SerialTransfer.h"
#include <math.h>


#define STEP_HEIGHT   20/2 
SerialTransfer myTransfer;

struct STRUCT {
  float x1;
  float y1;
  float z1;
  float alpha1;
  float beta1;

  float x2;
  float y2;
  float z2;
  float alpha2;
  float beta2;
  
} SendStruct;

float feetheight1,feetheight2;
int theta=0;
bool pair=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial6.begin(115200);
  myTransfer.begin(Serial6);

  SendStruct.x1 = 0;
  SendStruct.y1 = -5;
  SendStruct.z1 = 145;
  SendStruct.alpha1 = 0;
  SendStruct.beta1 = 0;
  
  SendStruct.x2 = 0;
  SendStruct.y2 = -5;
  SendStruct.z2 = 145;
  SendStruct.alpha2 = 0;
  SendStruct.beta2 = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  float radang = theta/180.00*PI;
  float steping = -STEP_HEIGHT*cos(radang)+(STEP_HEIGHT);
  
  if (pair){
    SendStruct.z1 = 150-steping;
  }
  else{
    SendStruct.z2 = 150-steping;
    //SendStruct.z1 = 145;
  }
  
  myTransfer.sendDatum(SendStruct);
  theta += 8;
  if (theta>360){
    theta = 0;  
    pair = !pair;
    Serial.println("switch");
    delay(200);
  }
  else{}
  delay(10);
}
