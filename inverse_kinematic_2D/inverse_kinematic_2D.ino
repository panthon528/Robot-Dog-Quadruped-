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

//function
float pythagoras(float x, float y);
float coslaw1(float a,float b, float l);
void invKinematic2D(float x,float z);
void invKinematic3D(float x,float y,float z);
void motorWrite(int motor, int rawValue);
int rad2rawFL(float rad);
int rad2rawBL(float rad);
int rad2rawFLC(float rad);
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  int16_t dxl_present_position = 0;               // Present position

  int motor1_raw = 0;
  int motor2_raw = 0;
  int motor3_raw = 0;
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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
  //enable torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 3, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  //write goal position
  while(1){
    Serial.print("type in z: ");
    while(Serial.available()==0);
    
    String str;
    str = Serial.readString();
    Serial.print(str);
    int a = str.length();
    int number = 0;
      switch(a) {
    case 4:
      number += (str[a-4]- 48)*100;
    case 3:
      number += (str[a-3]- 48)*10;
    case 2:
      number += (str[a-2]- 48);
      break;
    }
    Serial.println(-number);
    //invKinematic2D(-number,100);
    invKinematic3D(40,number,145);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, 1, ADDR_AX_GOAL_POSITION, motor1_raw, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, 2, ADDR_AX_GOAL_POSITION, motor2_raw, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, 3, ADDR_AX_GOAL_POSITION, motor3_raw, &dxl_error);
  }
  
  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 3, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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
  
  motor1_raw = rad2rawFL(alpha);
  motor2_raw = rad2rawBL(beta);
}
void invKinematic3D(float x,float y,float z){
  float theta = atan2(z,y);
  z = sqrt(z*z+y*y) - 45;

  invKinematic2D(x,z);
  motor3_raw = rad2rawFLC(theta);
  
}
void motorWrite(int motor, int rawValue){
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, motor, ADDR_AX_GOAL_POSITION, rawValue, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
}
int rad2rawFL(float rad){
  //convert radian to sevo input(+1.0471976 to accounted for initial 60 degrees start)
  return (rad+1.0471976+RIGHT_ANGLE)/5.2359878*1023;
}
int rad2rawBL(float rad){
  //convert radian to sevo input(+1.0471976 to accounted for initial 60 degrees start)
  return (rad+1.0471976-RIGHT_ANGLE)/5.2359878*1023;
}
int rad2rawFLC(float rad){
  return 1023 - (rad + 2.6179938)/5.2359878*1023;
}
