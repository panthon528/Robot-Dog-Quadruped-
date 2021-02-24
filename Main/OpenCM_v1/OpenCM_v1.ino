/*
 * Dynamixel : AX-series with Protocol 1.0
 * Controller : OpenCM9.04C + OpenCM 485 EXP
 * Power Source : SMPS 12V 5A
 * 
 * AX-Series are connected to Dynamixel BUS on OpenCM 485 EXP board or DXL TTL connectors on OpenCM9.04
 * http://emanual.robotis.com/docs/en/parts/controller/opencm485exp/#layout
 * 
 * This example will test only one Dynamixel at a time.
*/
#include "SerialTransfer.h"
#include <DynamixelSDK.h>
#include <math.h>

// AX-series Control table address
#define ADDR_AX_TORQUE_ENABLE           24                 // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "1"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
                                                            //DEVICENAME "2" -> Serial2
                                                            //DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      800                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     500                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define RIGHT_ANGLE 1.5707963           //right angle in radian
#define NO_MOTOR 12

//object
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

//function
float pythagoras(float x, float y);
float coslaw1(float a,float b, float l);
void invKinematic2D(float x,float z);
void invKinematic3D(float x,float y,float z);
void rad2raw_set(int leg);
void pair(float x, float y, float z, float theta, int a);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  int16_t dxl_present_position[13] = {0};               // Present position

  int motor_raw[3];//0 raw servo value of front motor,1 raw servo value of back motor,2 raw servo value of hip motor
  int motor_rawALL[13];//raw servo value of every servo counting from 1
  float angleRad[3];//0 radian angle of front motor, 1 radian angle of back motor, 2 radian angle of hip motor


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200);
  myTransfer.begin(Serial2);
  
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  // Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }
  //read cold&dark position
  Serial.print("read cold and dark position\n");
  for (int i = 1;i <= NO_MOTOR;i++){
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, i, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position[i], &dxl_error);
    //check comm
    if (dxl_comm_result != COMM_SUCCESS){
        packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0){
        packetHandler->getRxPacketError(dxl_error);
    }
    Serial.print(" Servo");
    Serial.print(i);
    Serial.print(" ");
    Serial.print(dxl_present_position[i]);
  }
  //enable torque
  for (int i = 1;i <= NO_MOTOR;i++){
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0){
      packetHandler->getRxPacketError(dxl_error);
    }
    else{
      Serial.print("Dynamixel ");Serial.print(i);Serial.print(" is online \n");
    }
  }
  //write goal position
  while(1){
    
    if(myTransfer.available()){
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
      
      /*Serial.print(SendStruct.x2,7);
      Serial.print("  ");
      Serial.print(SendStruct.y2,7);
      Serial.print("  ");
      Serial.print(SendStruct.z2,7);
      Serial.print("  ");
      Serial.print(SendStruct.alpha2,7);
      Serial.print("  ");
      Serial.println(SendStruct.beta2,7);
      Serial.println("  ");
      */
      //end of reading, start inv kinematic
      pair(SendStruct.x1, SendStruct.y1, SendStruct.z1,SendStruct.alpha1, 0);
      pair(SendStruct.x2, SendStruct.y2, SendStruct.z2,SendStruct.alpha2, 1);     

      //write to all motors
      for (int i = 1;i <= 12;i++){
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, i, ADDR_AX_GOAL_POSITION, motor_rawALL[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0){
          packetHandler->getRxPacketError(dxl_error);
        }
      }
      
    }//end of serial available
    else
      Serial.println("not received");
  }//end while 1 loop
  
  // Disable Dynamixel Torque
  for (int i = 1;i <= NO_MOTOR;i++){
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0){
      packetHandler->getRxPacketError(dxl_error);
    }
  }
  // Close port
  portHandler->closePort();
  
}

void loop() {

}

float pythagoras(float x, float y){
  return sqrt(x*x+y*y);
}
float coslaw1(float a,float b, float l){
  //input relative coordinate in mm from axle of motor2
  return acos((a+l*l)/(b*l));
}
void invKinematic2D(float x,float z){
  //find beta (angle of motor 2)
  float L1 = pythagoras(x,z); //length between motor 2 and end effector
  float betaPrime = coslaw1(-5280.0,92.0,L1) - RIGHT_ANGLE + atan2(z,x); //angle between l1 and motor 1 arm
  
  //find angle of lower link of motor 2
  float theta1 = acos((9512.0 - L1*L1)/7912.0)- RIGHT_ANGLE + betaPrime;
  
  //find coordinate of 2nd end effector
  float xPrime = 74.0*cos(theta1) - 46.0*sin(betaPrime);
  float zPrime = 74.0*sin(theta1) + 46.0*cos(betaPrime);
  
  //output
  float beta = betaPrime + RIGHT_ANGLE;
  float alpha = 2*RIGHT_ANGLE - coslaw1(-3360.0,92.0,pythagoras(32.0 - xPrime,zPrime)) - atan2(zPrime,32.0-xPrime);
  
  angleRad[0] = alpha;
  angleRad[1] = beta;
}
void invKinematic3D(float x,float y,float z){
  float theta = atan2(z,y);
  z = sqrt(z*z+y*y) - 45;

  invKinematic2D(x,z);
  angleRad[2] = theta;
}
void rad2raw_set1(void){
  //front left leg
  //convert radian to sevo input(+1.0471976 to accounted for initial 60 degrees start)
  motor_raw[0] = (angleRad[0]+1.0471976+RIGHT_ANGLE)/5.2359878*1023;
  //convert radian to sevo input(+1.0471976 to accounted for initial 60 degrees start)
  motor_raw[1] = (angleRad[1]+1.0471976-RIGHT_ANGLE)/5.2359878*1023;
  //hip angle +150 degrees and counter rotation
  motor_raw[2] = 1023 - (angleRad[2] + 2.6179938)/5.2359878*1023;
  /*for (int i = 0;i < NO_MOTOR;i++){
    Serial.println(angleRad[i]);
  }*/
}
void rad2raw_set(int leg){
  //front right leg
  switch(leg) {
    case 2:
      //front left leg
      //convert radian to sevo input(+1.0471976 to accounted for initial 60 degrees start)
      motor_raw[0] = (angleRad[0]+1.0471976+RIGHT_ANGLE)/5.2359878*1023;
      //convert radian to sevo input(+1.0471976 to accounted for initial 60 degrees start)
      motor_raw[1] = (angleRad[1]+1.0471976-RIGHT_ANGLE)/5.2359878*1023;
      //hip angle +150 degrees and counter rotation
      motor_raw[2] = 1023 - (angleRad[2] + 2.6179938)/5.2359878*1023;
      break;
    case 1:
      //front right leg
      //convert radian to sevo input(+1.0471976 to accounted for initial 60 degrees start)
      motor_raw[0] = 1023 - (angleRad[0]+1.0471976+RIGHT_ANGLE)/5.2359878*1023;
      //convert radian to sevo input(+1.0471976 to accounted for initial 60 degrees start)
      motor_raw[1] = 1023 - (angleRad[1]+1.0471976-RIGHT_ANGLE)/5.2359878*1023;
      //hip angle -30 degrees and counter rotation
      motor_raw[2] = 1023 - (angleRad[2] - 0.523598776)/5.2359878*1023;
      break;
    case 4:
      //back left
      //convert radian to sevo input(+1.0471976 to accounted for initial 60 degrees start)
      motor_raw[0] =(angleRad[0]+1.0471976+RIGHT_ANGLE)/5.2359878*1023;
      //convert radian to sevo input(+1.0471976 to accounted for initial 60 degrees start)
      motor_raw[1] =(angleRad[1]+1.0471976-RIGHT_ANGLE)/5.2359878*1023;
      //hip angle -30 degrees and counter rotation
      motor_raw[2] =(angleRad[2] + 2.6179938)/5.2359878*1023;
      break;
    case 3:
      //back right
      //convert radian to sevo input(+1.0471976 to accounted for initial 60 degrees start)
      motor_raw[0] = 1023 - (angleRad[0]+1.0471976+RIGHT_ANGLE)/5.2359878*1023;
      //convert radian to sevo input(+1.0471976 to accounted for initial 60 degrees start)
      motor_raw[1] = 1023 - (angleRad[1]+1.0471976-RIGHT_ANGLE)/5.2359878*1023;
      //hip angle -30 degrees and counter rotation
      motor_raw[2] = (angleRad[2] - 0.523598776)/5.2359878*1023;
      break;
  }
}
void pair(float x, float y, float z, float theta, int a){
  const float hypotLenght = 95.4410813; //half length between leg pair
  const float defAngle = 0.95663689896; //defult angle of leg pair (54.8112569 degree)
  const float localX = 55.00;//offset distance between local and global coordinate
  const float localY1 = 65.00;//upper offset
  const float localY2 = 92.00;//lower offset
  
  if (a == 0){
    //leg2 and leg3
    //leg2
    float xloc = hypotLenght*sin(defAngle-theta) - localY1 + y;
    float yloc = hypotLenght*cos(defAngle-theta) - localX - x;
    /*Serial.print("leg2: ");
    Serial.print(xloc);Serial.print(",");
    Serial.print(yloc);Serial.print(",");
    Serial.println(z);*/
    invKinematic3D(xloc,yloc,z);
    rad2raw_set(2);
    for (int i = 0;i<3;i++){
      motor_rawALL[1+i] = motor_raw[i];
    } 
    //leg3
    xloc = localY2 - hypotLenght*sin(defAngle-theta) + y;
    yloc = localX - hypotLenght*cos(defAngle-theta) - x;
    /*Serial.print("leg3: ");
    Serial.print(xloc);Serial.print(",");
    Serial.print(yloc);Serial.print(",");
    Serial.println(z);*/
    invKinematic3D(xloc,yloc,z);
    rad2raw_set(3);
    for (int i = 0;i<3;i++){
      motor_rawALL[10+i] = motor_raw[i];
    } 
  }
  else if (a==1){
    //leg1 and leg4
    //leg1
    float xloc = hypotLenght*sin(defAngle+theta) - localY1 + y;
    float yloc = localX - hypotLenght*cos(defAngle+theta) - x;
    /*Serial.print("leg1: ");
    Serial.print(xloc);Serial.print(",");
    Serial.print(yloc);Serial.print(",");
    Serial.println(z);*/
    invKinematic3D(xloc,yloc,z);
    rad2raw_set(1);
    for (int i = 0;i<3;i++){
      motor_rawALL[4+i] = motor_raw[i];
    } 
    //leg4
    xloc = localY2 - hypotLenght*sin(defAngle+theta) + y;
    yloc = hypotLenght*cos(defAngle+theta) - localX - x;
    /*Serial.print("leg4: ");
    Serial.print(xloc);Serial.print(",");
    Serial.print(yloc);Serial.print(",");
    Serial.println(z);*/
    invKinematic3D(xloc,yloc,z);
    rad2raw_set(4);
    for (int i = 0;i<3;i++){
      motor_rawALL[7+i] = motor_raw[i];
    } 
  }
  
}
