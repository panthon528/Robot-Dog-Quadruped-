
void setup() {
  Serial.begin(9600);
  Serial6.begin(9600);
}

void loop() {
  int incomingByte;
        
  //if (Serial.available() > 0) {
    //incomingByte = Serial.read();
    //Serial.print("USB received: ");
    //Serial.println(incomingByte, DEC);
    Serial.print("USB received:");
    Serial.println("send from teensy");
                
                Serial6.print("USB received:");
                Serial6.println("send from teensy");
  //}
  /*if (Serial6.available() > 0) {
    incomingByte = Serial6.read();
    Serial.print("UART received: ");
    Serial.println(incomingByte, DEC);
                Serial6.print("UART received:");
                Serial6.println(incomingByte, DEC);
  }*/
}
