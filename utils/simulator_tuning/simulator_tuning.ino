//Connection order: 5V - LDR - A0 - Resistor - GND
//Connection order: 9 - Resistor - Led - GND

//Increase arduino Timer1 frequency
#include <avr/io.h>
const byte mask = B11111000;
int prescale = 1;

//Input and output pins
#define LDR_pin A0
#define led_pin 9

//LDR constants
#define measurements_LDR 5
#define measurements_per_step 300
#define m -0.6505
#define b 4.778

//Static gain calculation variables
float Go, Go_sum = 0;

//LDR measurement variables
float V_R, R_LDR, L_sum;

void setup() { 

  //Maximum frequency
  TCCR1B = (TCCR1B & mask) | prescale;

  //Init serial connection
  Serial.begin(2000000);

  //Calibrate LDR readings (obtain static gain)
  calibrate();

  //Send static gain through serial
  Serial.println(Go);

  //Turn off led
  analogWrite(led_pin, 0);
  delay(1000);

  //Positive Steps
  for (int i = 1; i <= 5; i++) {
    step_change(i);  
  } 
  
  //Negative Steps
  for (int i = 4; i >= 0; i--) {
    step_change(i);  
  } 
}

void loop() {
}

void calibrate() {
  
  //Turn on led with 5 different voltage levels
  for (int voltage = 1; voltage <= 5; voltage++) {
  
    //Change led brightness
    analogWrite(led_pin, voltage*255/5);
    delay(200);
  
    //Read LDR measurements and calculate static gain
    Go_sum += LDR_measurement()/voltage;
  }

  //Calculate average static gain
  Go = Go_sum/5;
}

float LDR_measurement() {

  //Reset variable
  L_sum = 0;

  //Measurements with average filter
  for (int i = 0; i < measurements_LDR; i++) {
    V_R = (analogRead(LDR_pin)*5)/1023.00;
    R_LDR = (50000-V_R*10000)/V_R;
    L_sum += pow(10,((log10(R_LDR)-b)/m));
    delay(1);
  }

  //Returns LDR illuminance
  return L_sum/measurements_LDR;
}

void step_change(int voltage) {
 
  //Put led at final state and record LDR voltage measurements
  analogWrite(led_pin, voltage*255/5);
  for (int i = 0; i < measurements_per_step; i ++) {
    Serial.println(micros());
    Serial.println((analogRead(LDR_pin)*5)/1023.00);
  }
}
