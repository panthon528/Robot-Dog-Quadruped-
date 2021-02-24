/*
 Example sketch for the Xbox 360 USB library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <XBOXUSB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
XBOXUSB Xbox(&Usb);

void send2OpenCM (int x1,int y1,int z1,int x2,int y2,int z2);

void setup() {
  Serial.begin(115200);
  Serial6.begin(115200);
  
#if !defined(__MIPSEL__)
  //while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));
}
void loop() {
  int l2=0,r2;
  int lefty;
  Usb.Task();
  if (Xbox.Xbox360Connected) {
    //if (Xbox.getButtonPress(L2) || Xbox.getButtonPress(R2)) {
      //Serial.print("L2: ");
      //Serial.print(Xbox.getButtonPress(L2));
      l2 = map(Xbox.getButtonPress(L2),0,255,-30,70);
      l2 = 0;
      r2 = map(Xbox.getButtonPress(R2),0,255,145,100);
      //Serial.print("\tR2: ");
      //Serial.println(Xbox.getButtonPress(R2));
      //Xbox.setRumbleOn(Xbox.getButtonPress(L2), Xbox.getButtonPress(R2));
    //} else
      //Xbox.setRumbleOn(0, 0);
    /*if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500 || Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(LeftHatY) < -7500 || Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500 || Xbox.getAnalogHat(RightHatY) > 7500 || Xbox.getAnalogHat(RightHatY) < -7500) {
      if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500) {
        Serial.print(F("LeftHatX: "));
        Serial.print(Xbox.getAnalogHat(LeftHatX));
        Serial.print("\t");
      }
      if (Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(LeftHatY) < -7500) {
    */    //Serial.print(F("LeftHatY: "));
        //leftY = map(Xbox.getAnalogHat(LeftHatY));
        /*Serial.print(leftY);
        Serial.print("\t");
        Serial.print(32768);
        Serial.print("\t");
        Serial.println(-32768);*/

        //UART
        /*uSerial.print("[");
        Serial.print("-1,3,5,-7,9,11,-13,-15");
        Serial.println("]");*/
        
      /*}
      if (Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500) {
        Serial.print(F("RightHatX: "));
        Serial.print(Xbox.getAnalogHat(RightHatX));
        Serial.print("\t");
      }
      if (Xbox.getAnalogHat(RightHatY) > 7500 || Xbox.getAnalogHat(RightHatY) < -7500) {
        Serial.print(F("RightHatY: "));
        Serial.print(Xbox.getAnalogHat(RightHatY));
      }
      Serial.println();
    }*/
    /*
    if (Xbox.getButtonClick(UP)) {
      Xbox.setLedOn(LED1);
      Serial.println(F("Up"));
    }
    if (Xbox.getButtonClick(DOWN)) {
      Xbox.setLedOn(LED4);
      Serial.println(F("Down"));
    }
    if (Xbox.getButtonClick(LEFT)) {
      Xbox.setLedOn(LED3);
      Serial.println(F("Left"));
    }
    if (Xbox.getButtonClick(RIGHT)) {
      Xbox.setLedOn(LED2);
      Serial.println(F("Right"));
    }

    if (Xbox.getButtonClick(START)) {
      Xbox.setLedMode(ALTERNATING);
      Serial.println(F("Start"));
    }
    if (Xbox.getButtonClick(BACK)) {
      Xbox.setLedBlink(ALL);
      Serial.println(F("Back"));
    }
    if (Xbox.getButtonClick(L3))
      Serial.println(F("L3"));
    if (Xbox.getButtonClick(R3))
      Serial.println(F("R3"));

    if (Xbox.getButtonClick(L1))
      Serial.println(F("L1"));
    if (Xbox.getButtonClick(R1))
      Serial.println(F("R1"));
    if (Xbox.getButtonClick(XBOX)) {
      Xbox.setLedMode(ROTATING);
      Serial.println(F("Xbox"));
    }

    if (Xbox.getButtonClick(A))
      Serial.println(F("A"));
    if (Xbox.getButtonClick(B))
      Serial.println(F("B"));
    if (Xbox.getButtonClick(X))
      Serial.println(F("X"));
    if (Xbox.getButtonClick(Y))
      Serial.println(F("Y"));
    */
  }
  send2OpenCM (0,l2,r2,0,l2,r2);
}
void send2OpenCM (int x1,int y1,int z1,int x2,int y2,int z2){
  Serial6.print("[");
  
  Serial6.print(x1);
  Serial6.print(",");
  Serial6.print(y1);
  Serial6.print(",");
  Serial6.print(z1);
  Serial6.print(",");
  Serial6.print("0");

  Serial6.print(",");

  Serial6.print(x2);
  Serial6.print(",");
  Serial6.print(y2);
  Serial6.print(",");
  Serial6.print(z2);
  Serial6.print(",");
  Serial6.print("0");
  
  Serial6.print("]");
}
