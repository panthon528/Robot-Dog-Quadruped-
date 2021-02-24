
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
}

void loop() {
  String inString = "";
  int inInt = 0;
  
  if (Serial2.available() > 0 && Serial2.read() == '[') {
    int inChar;
    bool negative = false;
    do{
        if (Serial2.available() > 0) {
          inChar = Serial2.read();
          if (inChar == '-'){
            negative = true;
          }
          if (isDigit(inChar)) {
            // convert the incoming byte to a char and add it to the string:
            inString += (char)inChar;
          }
        }
    }while (inChar != ']');
    inInt = inString.toInt();
    if (negative)
      inInt *= -1;

    Serial.println(inInt);
  }
}
