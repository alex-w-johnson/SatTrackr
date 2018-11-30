
String receivedString = "";
static int receivedValue = 0;
static int receivedArray[6] = [];
static int checksum = 0;
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
    int idx = 0;
    while(idx < 6;){
      if(CharReceived=='|'){
        Serial.println("Starting to read row.");
        Serial.print("received: ");
        Serial.println(receivedString);
        receivedValue = receivedString.toInt();
        Serial.print("value: ");
        Serial.println(receivedValue);
        receivedString = "";
        receivedArray[idx] = receivedValue;
        checksum += receivedValue;
        idx++;
      }
      else if(CharReceived==','){
        Serial.print("received: ");
        Serial.println(receivedString);
        receivedValue = receivedString.toInt();
        Serial.print("value: ");
        Serial.println(receivedValue);
        receivedString = "";
        receivedArray[idx] = receivedValue;
        checksum += receivedValue;
      else if(CharReceived==';'){
        if(checksum != receivedValue){
          Serial.println("PANIC NOW CHKSUM IS DIFFERENT YA DINGUS!")
        }
        Serial.print("Row recieved:    ");
        for(int printidx = 0; printidx <= idx; printidx++){
          Serial.print(recievedArray[printidx]);
          Serial.print("    ");
        }
        Serial.println("Proceeding to next row!");
      else{
        Serial.println("SOMETHING ELSE WENT WRONG YA DINGUS!");
      }
        
          
      }
        
    }
  }
}

