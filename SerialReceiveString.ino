
String receivedString = "";

void setup() {
Serial.begin(9600);
Serial.print("started\n");
}

void loop() {
  while(Serial.available()>0){
    int CharReceived = Serial.read();
    //if(isDigit(CharReceived)){
    if(1){
      receivedString += (char)CharReceived;
    }
    if(CharReceived=='|'){
      Serial.print("received: ");
      Serial.println(receivedString);
      int receivedValue = receivedString.toInt();
      Serial.print("value: ");
      Serial.println(receivedValue);
      receivedString = "";
    }
  }
}

