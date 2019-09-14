//Adress 0 will be slave address
//Address 1 will be master address

#include <EEPROM.h>

#define my_address 41

void setup() {
  Serial.begin(2000000);
  
  EEPROM.write(0, my_address);
  EEPROM.write(1, 1);
  
  Serial.print("Address 0: "); Serial.println(EEPROM.read(0));
  Serial.print("Address 1: "); Serial.println(EEPROM.read(1));
}

void loop() {
}