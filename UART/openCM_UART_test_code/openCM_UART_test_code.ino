
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
}

void loop() {
  char incomingByte;
        
  /*if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.print("USB received: ");
    Serial.println(incomingByte, DEC);
    Serial.print("USB received:");
    Serial.println("send from teensy");
      
               Serial6.print("USB received:");
                Serial6.println("send from teensy");
  }*/
  if (Serial2.available() > 0) {
    incomingByte = Serial2.read();
    //Serial.print("UART received: ");
    Serial.print(incomingByte);
                /*Serial2.print("UART received:");
                Serial2.println(incomingByte, DEC);*/
  }
}
