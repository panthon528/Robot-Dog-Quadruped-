#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "SerialTransfer.h"
#include <math.h>


#define STEP_HEIGHT   40/2 
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

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
  float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
  
  float lastgam = 0;
  float integral = 0;
  float feedback = 0; 
  int theta=0;
  bool state=0;

  float lastx1 = 0;
  float lasty1 = 0;

  float lastx2 = 0;
  float lasty2 = 0;
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial6.begin(115200);
  myTransfer.begin(Serial6);

  SendStruct.x1 = 0;
  SendStruct.y1 = 0;
  SendStruct.z1 = 145;
  SendStruct.alpha1 = 0;
  SendStruct.beta1 = 0;
  
  SendStruct.x2 = 0;
  SendStruct.y2 = 0;
  SendStruct.z2 = 105;
  SendStruct.alpha2 = 0;
  SendStruct.beta2 = 0;

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t event;
  bno.getEvent(&event);

  float alpha = (90-54.8112569)/180.00*PI;

  float gamma;
  if (state == 0){
    gamma = atan2(sin(alpha)*tan(-event.orientation.z/180.00*PI)+cos(alpha)*tan(event.orientation.y/180.00*PI),1);
  }
  else{
    gamma = atan2(sin(alpha)*tan(-event.orientation.z/180.00*PI)-cos(alpha)*tan(event.orientation.y/180.00*PI),1);
  }
  /*Serial.print(event.orientation.z);
  Serial.print("\t");
  Serial.print(event.orientation.y);
  Serial.print("\t");*/
  Serial.println(gamma/PI*180.00);
  
  integral += gamma;
  if (integral > 45.00/180.00*PI){
    integral = 45.00/180.00*PI;
  }
  else if(integral < -45.00/180.00*PI){
    integral = -45.00/180.00*PI;
  }
   
  feedback += 0.10*gamma+0.08*(gamma-lastgam)+0.00*integral;
  if (feedback > 45.00/180.00*PI){
    feedback = 45.00/180.00*PI;
  }
  else if(feedback < -45.00/180.00*PI){
    feedback = -45.00/180.00*PI;
  }
  
  float ortogonal = 145*tan(feedback);
  
  /*SendStruct.x1 = ortogonal*cos(alpha);
  SendStruct.y1 = ortogonal*sin(alpha);
  SendStruct.z1 = 145*cos(feedback);
  */
  float radang = theta/180.00*PI;
  float steping = -STEP_HEIGHT*cos(radang)+(STEP_HEIGHT);
  if (state == 0){
    //first pair
    SendStruct.x1 = ortogonal*cos(alpha);
    SendStruct.y1 = ortogonal*sin(alpha);
    SendStruct.z1 = 145;//*cos(feedback);

    lastx1 = SendStruct.x1;
    lasty1 = SendStruct.y1;
    
    //second pair
    if (theta < 180){
      SendStruct.x2 = mapfloat(steping, 0, 40, lastx2, 0);
      SendStruct.y2 = mapfloat(steping, 0, 40, lasty2, 0);
    }
    else{
      SendStruct.x2 = 0;
      SendStruct.y2 = 0;
    }
    SendStruct.z2 = 145-40;
  }
  else{
    //first pair
    if (theta < 180){
      SendStruct.x1 = mapfloat(steping, 0, 40, lastx1, 0);
      SendStruct.y1 = mapfloat(steping, 0, 40, lasty1, 0);
    }
    else{
      SendStruct.x1 = 0;
      SendStruct.y1 = 0;
    }
    SendStruct.z1 = 145-steping;
    //second pair
    SendStruct.x2 = -ortogonal*cos(alpha);
    SendStruct.y2 = ortogonal*sin(alpha);
    SendStruct.z2 = 145;//*cos(feedback);
    
    lastx2 = SendStruct.x2;
    lasty2 = SendStruct.y2;
  }
  lastgam = gamma;
  myTransfer.sendDatum(SendStruct);
  //theta+=10;
  if (theta>360){
    theta = 0;  
    state = !state;
    Serial.println("switch");
    //reset PID
    integral = 0;
    feedback = 0;
    lastgam = 0;
  }
  delay(10);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
