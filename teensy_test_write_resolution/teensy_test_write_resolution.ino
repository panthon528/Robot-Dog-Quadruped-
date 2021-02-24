#include "SerialTransfer.h"
#include <math.h>

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

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int theta=0;

void setup()
{
  Serial.begin(115200);
  Serial6.begin(115200);
  myTransfer.begin(Serial6);

  SendStruct.x1 = 0;
  SendStruct.y1 = 0;
  SendStruct.z1 = 130;
  SendStruct.alpha1 = 0;
  SendStruct.beta1 = 0;
  
  SendStruct.x2 = 0;
  SendStruct.y2 = 0;
  SendStruct.z2 = 130;
  SendStruct.alpha2 = 0;
  SendStruct.beta2 = 0;
}


void loop()
{
  float radang = theta/180.00*PI;
  float steping = (-cos(radang)+1)*50;

  float temp = mapfloat((float)steping, 0, 100, -30, 70);
  SendStruct.y1 = temp;
  SendStruct.y2 = temp;
  theta += 1;
  Serial.println(temp);
  if (theta>360){
    theta = 0;  
    Serial.println("switch");
  }
  myTransfer.sendDatum(SendStruct);
  delay(10);
}
