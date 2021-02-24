#include "SerialTransfer.h"


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


void setup()
{
  Serial.begin(115200);
  Serial6.begin(115200);
  myTransfer.begin(Serial6);

  SendStruct.x1 = 1.1415926;
  SendStruct.y1 = 2.1415926;
  SendStruct.z1 = 3.1415926;
  SendStruct.alpha1 = 4.1415926;
  SendStruct.beta1 = 5.1415926;
  
  SendStruct.x2 = -1.1415926;
  SendStruct.y2 = -2.1415926;
  SendStruct.z2 = -3.1415926;
  SendStruct.alpha2 = -4.1415926;
  SendStruct.beta2 = -5.1415926;
}


void loop()
{
  myTransfer.sendDatum(SendStruct);
  delay(500);
}
