#define HWSERIAL Serial1

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  // put your setup code here, to run once:
}

void loop() {
  int incomingByte;
  // put your main code here, to run repeatedly:
  if (Serial1.available() > 0) {
    incomingByte = Serial1.read();
    Serial.print("UART received: ");
    Serial.println(incomingByte, DEC);
    Serial1.print("UART received:");
    Serial1.println(incomingByte, DEC);
  }
}
