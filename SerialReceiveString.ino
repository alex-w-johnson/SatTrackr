String receivedString = "";
const int rowLength = 6;
static int receivedValue = 0;
static int receivedArray[rowLength] = {};
static int checksum = 0;

void setup() {
  Serial.begin(9600);
  Serial.print("started\n");
}
void loop() {
  while (Serial.available() > 0) {

    int idx = 0;
    while (idx < 6) {
      //Serial.print("Column Index: "); debugging
      //Serial.println(idx);
      int CharReceived = Serial.read();
      //if(isDigit(CharReceived)){
      if (1) {
        receivedString += (char)CharReceived;
      }
      if (CharReceived == '|') {
        Serial.println("Starting to read row.");
        receivedString = "";
        delay(20);
      }
      else if (CharReceived == ',') {
        Serial.print("received: ");
        Serial.println(receivedString);
        receivedValue = receivedString.toInt();
        Serial.print("value: ");
        Serial.println(receivedValue);
        receivedString = "";
        receivedArray[idx] = receivedValue;
        checksum += receivedValue;
        idx++;
        delay(20);
      }
      else if (CharReceived == ';') {
        receivedValue = receivedString.toInt();
        Serial.print("value: ");
        Serial.println(receivedValue);
        receivedString = "";
        receivedArray[idx] = receivedValue;
        if (checksum != receivedValue) {
          Serial.println("PANIC NOW CHKSUM IS DIFFERENT YA DINGUS!");
        }
        Serial.print("Row recieved:    ");
        for (int printidx = 0; printidx < rowLength; printidx++) {
          Serial.print(receivedArray[printidx]);
          Serial.print("    ");
        }
        Serial.println("Proceeding to next row!");
        idx++;
        delay(20);
      }
      delay(50);
    }

  }
}
