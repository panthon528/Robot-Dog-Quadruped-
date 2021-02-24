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
  Serial2.begin(115200);
  myTransfer.begin(Serial2);
}


void loop()
{
  if(myTransfer.available())
  {
    myTransfer.rxObj(SendStruct);
    Serial.print(SendStruct.x1,7);
    Serial.print("  ");
    Serial.print(SendStruct.y1,7);
    Serial.print("  ");
    Serial.print(SendStruct.z1,7);
    Serial.print("  ");
    Serial.print(SendStruct.alpha1,7);
    Serial.print("  ");
    Serial.println(SendStruct.beta1,7);
    
    Serial.print(SendStruct.x2,7);
    Serial.print("  ");
    Serial.print(SendStruct.y2,7);
    Serial.print("  ");
    Serial.print(SendStruct.z2,7);
    Serial.print("  ");
    Serial.print(SendStruct.alpha2,7);
    Serial.print("  ");
    Serial.println(SendStruct.beta2,7);
    Serial.println("  ");
  }
}
