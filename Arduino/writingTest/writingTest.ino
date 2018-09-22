#include <EEPROM.h>
#include <SoftwareSerial.h>
  int addr=0;
void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);


}

void loop() {
  // put your main code here, to run repeatedly:

  int readNum = EEPROM.read(addr);
  Serial.print("Raw from address :");
  Serial.print(readNum, DEC);
  Serial.println();
  EEPROM.write(addr,(readNum+1));
  delay(2000);

}
