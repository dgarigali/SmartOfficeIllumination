//Libraries
#include <avr/io.h>
#include <Wire.h>
#include <EEPROM.h>
#include "lum_system.h"
#include "dist_controller.h"
#include "luminaire_controller.h"

//Illuminance thresholds
#define unoccupied_desk 20
#define occupied_desk 50

//I2C
#define i2c_speed 400000
const int my_address = EEPROM.read(0);

//Luminaire system (LDR)
#define LDR_pin A0
#define measurements_LDR 10
#define m -0.6505
#define b 4.778
#define R1 10e3
#define Vcc 5
#define max_ADC_resolution 1023.00

//Luminaire system (LED)
#define led_pin 9
#define max_DAC_resolution 1023

//Luminaire system (Calibration)
#define node_num EEPROM.read(1)
#define num_nodes 2

//Distributed controller
#define cost 1
#define rho 0.07
float des_illum = unoccupied_desk; //unoccupied by default

//Luminaire controller
#define sample_time 10 //ms
#define a5 -9.8875e-6
#define a4 0.0033
#define a3 -0.4397
#define a2 30.0663
#define a1 -1141.7
#define a0 27737
#define min_error 0.001 //value based on empirical results
#define kp 0.3 //tradeoff from both methods
#define ki 180 //tradeoff from both methods
float current_voltage;

//Switch
#define switch_pin 8
#define debounce_delay 100 //ms

//Comunication
#define serial_data_rate 100 //100*10ms = 1s
int counter = 0;
unsigned long prev_time = 0;
int serial_state = 0;

//Push button
boolean button_flag = true;
boolean desk_occupied = false;
boolean debounce_flag = true;
int button_current_state;
int button_past_state = HIGH;
unsigned long last_debounce_time = 0;

//Init luminaire system object
lum_system obj_lum_system = lum_system();

//Init distributed controller object
dist_controller obj_dist_controller = dist_controller(des_illum, cost, node_num, rho, num_nodes, Vcc);

//Init luminaire controller object
luminaire_controller lum_cont = luminaire_controller();

void setup() {

  //Init switch pin
  pinMode(switch_pin, INPUT);
  digitalWrite(switch_pin, HIGH); //pull-up mode

  //Init serial communication
  Serial.begin(2000000);

  //Change to PWM Phase-Correct 10-bit (improves quantization errors)
  TCCR1A = (TCCR1A & B11111100) | B00000011;

  //Set prescaler to 1 (maximum frequency of 8khz)
  TCCR1B = (TCCR1B & B11111000) | B00000001;

  //Config luminaire system
  obj_lum_system.set_LDR_parameters(R1, Vcc, m, b, max_ADC_resolution, measurements_LDR, LDR_pin);
  obj_lum_system.set_LED_parameters(led_pin, max_DAC_resolution);
  obj_lum_system.set_calibration_parameters(node_num, num_nodes);

  //Measure disturbance (external illuminance - led is off)
  obj_lum_system.calc_ext_dist();

  //Init I2C communication
  Wire.begin(my_address); //join as a master/slave
  Wire.setClock(i2c_speed); //define i2c clock frequency
  TWAR = (my_address << 1) | 1; //Enable receiving of broadcast messages
  Wire.onReceive(receiveEvent); //event handler

  //Calibration routine
  obj_lum_system.calibration_routine();

  //Configure distributed controller
  obj_dist_controller.configuration(obj_lum_system.get_gains(), obj_lum_system.get_ext_dist());
  
  //Send number of nodes
  Serial.write(num_nodes);

  //Send external illuminance
  int ext_illum_int = (int) ((obj_lum_system.get_ext_dist()+0.005)*100);  
  Serial.write(highByte(ext_illum_int));
  Serial.write(lowByte(ext_illum_int));

  //Send gains
  for (int i = 0; i < num_nodes; i++) {
    int gain_int = (int) ((obj_dist_controller.get_k(i)+0.005)*100);
    Serial.write(highByte(gain_int));
    Serial.write(lowByte(gain_int));
  }

  //Implement distributed controller
  obj_dist_controller.actuation();

  //Set feedforward voltage
  lum_cont.set_feedforward_voltage(obj_dist_controller.get_voltage());

  //Init and configure simulator  
  lum_cont.set_simulator_coefficients(a5, a4, a3, a2, a1, a0);
  lum_cont.configure_simulator(micros(), obj_lum_system.lux_to_volt(obj_dist_controller.get_illuminance()), obj_dist_controller.get_illuminance());
  lum_cont.set_initial_voltage(obj_lum_system.LDR_voltage());
    
  //Define discrete PI parameters
  lum_cont.set_PI_parameters(kp, ki, sample_time);
  lum_cont.set_feedback_min_error(min_error);
  lum_cont.set_anti_windup_limits(0, Vcc); 

  //Set timer 2 interrupts at 100Hz
  configure_timer2();
}

void configure_timer2() {
  cli();
  TCCR2A = 0; //clear register
  TCCR2B = 0; //clear register
  TCNT2  = 0; //reset counter
  OCR2A = 155; // (16*10^6/(100*1024))-1 (must be <256)
  TCCR2A = (TCCR2A & B11111101) | B00000010; //CTC mode
  TCCR2B=(1<<CS20)|(1<<CS21)|(1<<CS22); //Set 1024 prescaler
  TIMSK2 |= (1 << OCIE2A);  //Enable timer compare interrupt
  sei(); //allow interrupts
}

//Implement control actuation each sample time
ISR(TIMER2_COMPA_vect) {
  
  //Simulator
  lum_cont.simulator(micros());

  //Feedback controller
  current_voltage = obj_lum_system.LDR_voltage();
  lum_cont.feedback_controller(current_voltage);

  //Integrator anti-windup
  lum_cont.set_controller_voltage(lum_cont.get_feedforward_voltage() + lum_cont.get_feedback_voltage());
  lum_cont.anti_windup();

  //Actuate on the led
  analogWrite(led_pin, obj_lum_system.dac(lum_cont.get_controller_voltage()));

  //Update feedback controller variables
  lum_cont.update_feedback();
}

void receiveEvent(int value) {

  int i = 0, state = 0;
  byte ID, message_ID, payload;
 
  while(Wire.available()) {

    switch(state) {
      
      case 0:
        ID = Wire.read();
        if (ID == 0) { //Calibration
          state = 1;  
        } else if (ID == 1) { //Consensus
          state = 2;
        } else if (ID == 2) { //Logs
          state = 4; 
        }
        break;
        
      case 1: //Calibration
        message_ID = Wire.read();
        obj_lum_system.receive_byte(message_ID);
        state = 0;
        break;
        
      case 2: //Consensus
        message_ID = Wire.read();
        if (message_ID == 0 or message_ID == 2) { //Reference or cost message
          obj_dist_controller.receive_byte(message_ID, true);
          state = 0;
        } else if (message_ID == 1) { //Consensus message
          obj_dist_controller.receive_byte(message_ID, false);
          state = 3;
        } 
        break;
        
      case 3: 
        payload = Wire.read();
        i++;
        if (i == num_nodes*2 + 1) {
          obj_dist_controller.receive_byte(payload, true);
          i = 0;
          state = 0;
        } else {
          obj_dist_controller.receive_byte(payload, false);
        }
        break;

     case 4: //logs (ignored by arduino)
        payload = Wire.read();
        i++;
        if (i == 12) {
          i = 0;
          state = 0;
        }
        
    }   
  }
}

void loop() {

  //Handle serial communication (check if serial interface defined a new illuminance reference or new cost)
  read_serial_port();

  //Handle push button (check if it was pushed, with debounce)
  push_button_routine();

  //Check I2C messages
  if(obj_dist_controller.compute_message()) {
    lum_cont.configure_simulator(micros(), obj_lum_system.lux_to_volt(obj_dist_controller.get_illuminance()), obj_dist_controller.get_illuminance());
    lum_cont.set_feedforward_voltage(obj_dist_controller.get_voltage());
  }

  //Send data through serial
  unsigned long current_time = millis();
  if ((current_time - prev_time) > sample_time) {
    write_serial_port();
    prev_time = current_time;
  }
}

//Check if there is new data on serial port
void read_serial_port() {

  if(Serial.available()) {

    byte data = Serial.read();

    switch(serial_state) {
      
      case 0:
        if (data == 0) { //Reference
          serial_state = 1;
        } else if (data == 1) { //Cost
          serial_state = 2;
        } 
        break;
        
      case 1: //Reference
        des_illum = data;
        obj_dist_controller.set_des_illuminance(data);
        obj_dist_controller.reference_change();
        lum_cont.configure_simulator(micros(), obj_lum_system.lux_to_volt(obj_dist_controller.get_illuminance()), obj_dist_controller.get_illuminance());
        lum_cont.set_feedforward_voltage(obj_dist_controller.get_voltage());
        serial_state = 0;
        break;

      case 2: //Cost
        obj_dist_controller.set_cost(data);
        obj_dist_controller.reference_change();
        lum_cont.configure_simulator(micros(), obj_lum_system.lux_to_volt(obj_dist_controller.get_illuminance()), obj_dist_controller.get_illuminance());
        lum_cont.set_feedforward_voltage(obj_dist_controller.get_voltage());
        serial_state = 0;
        break;       
    }
  } 
}

//Reads push button
void push_button_routine() {
  if (debounce(switch_pin) == 0) { //Check if button was pushed with debounce
    if (button_flag) {
      button_flag = false; //Use flag for only changing state once each time button is pushed
      desk_occupied = not(desk_occupied); //Change desk state
      if (desk_occupied) {
        des_illum = occupied_desk;
      } else {
        des_illum = unoccupied_desk;
      }
      
      //Distributed controller actuation
      obj_dist_controller.set_des_illuminance(des_illum);
      obj_dist_controller.reference_change();
      lum_cont.configure_simulator(micros(), obj_lum_system.lux_to_volt(obj_dist_controller.get_illuminance()), obj_dist_controller.get_illuminance());
      lum_cont.set_feedforward_voltage(obj_dist_controller.get_voltage());
    }
  } else {
    button_flag = true;
  }
}

//Checks push button debounce without time delay
boolean debounce (int pin) {
  int button_current_state = digitalRead(pin);
  if (button_current_state != button_past_state) { //Check if button state changed
    if (debounce_flag) {
      last_debounce_time = millis();
      debounce_flag = false; //Flag allows to know when to reset the counter
    }
  } else {
    debounce_flag = true;
  }
  if ((millis() - last_debounce_time) > debounce_delay) { //Check if button has been pushed for enough time
    if (button_current_state != button_past_state) {
      button_past_state = button_current_state; //Changes button state
    }
  }
  return button_past_state;
}

//Send log data through I2C to raspberry py
void write_I2C() {
    
  byte message_i2c[13];
  int converted_value;

  //Message ID
  message_i2c[0] = 2;

  //Node ID
  message_i2c[1] = node_num;
  
  //Current illuminance
  converted_value = (int) ((obj_lum_system.volt_to_lux(current_voltage)+0.005)*100);
  message_i2c[2] = highByte(converted_value);
  message_i2c[3] = lowByte(converted_value);
  
  //Current duty cycle
  converted_value = (int) (((lum_cont.get_controller_voltage()/Vcc)+0.005)*100);
  message_i2c[4] = highByte(converted_value);
  message_i2c[5] = lowByte(converted_value);
  
  //Occupancy state
  if (des_illum == occupied_desk) {
    message_i2c[6] = 1;
  } else {
    message_i2c[6] = 0;
  }
  
  //Illuminance lower bound
  converted_value = (int) ((des_illum+0.005)*100);
  message_i2c[7] = highByte(converted_value);
  message_i2c[8] = lowByte(converted_value);
  
  //External illuminance
  float current_ext_dist = obj_lum_system.volt_to_lux(current_voltage);
  for (int i = 0; i < num_nodes; i++) {
    current_ext_dist -= obj_dist_controller.get_k(i)*obj_dist_controller.get_dimming_level(i);
  }
  if (current_ext_dist < 0) {
    current_ext_dist = 0;
  }
  obj_dist_controller.set_external_disturbance(current_ext_dist);
  converted_value = (int) ((current_ext_dist+0.005)*100);
  message_i2c[9] = highByte(converted_value);
  message_i2c[10] = lowByte(converted_value);
  
  //Luminance reference
  converted_value = (int) ((obj_dist_controller.get_illuminance()+0.005)*100);
  message_i2c[11] = highByte(converted_value);
  message_i2c[12] = lowByte(converted_value);

  //Send message
  Wire.beginTransmission(0); //get BUS
  Wire.write(message_i2c, sizeof(message_i2c));
  Wire.endTransmission();
}

//Sends relevant data through serial port (to be shown in Matlab Gui)
void write_serial_port() {
  if (counter == serial_data_rate) {
    
    byte message_serial[8];
    int converted_value;

    //Current illuminance
    converted_value = (int) ((obj_lum_system.volt_to_lux(current_voltage)+0.005)*100);
    message_serial[0] = highByte(converted_value);
    message_serial[1] = lowByte(converted_value);

    //Led voltage
    converted_value = (int) ((lum_cont.get_controller_voltage()+0.005)*100);
    message_serial[2] = highByte(converted_value);
    message_serial[3] = lowByte(converted_value);

    //Consensus voltage
    converted_value = (int) ((lum_cont.get_feedforward_voltage()+0.005)*100);
    message_serial[4] = highByte(converted_value);
    message_serial[5] = lowByte(converted_value);
    
    //Consensus illuminance
    converted_value = (int) ((obj_dist_controller.get_illuminance()+0.005)*100);
    message_serial[6] = highByte(converted_value);
    message_serial[7] = lowByte(converted_value);

    //Send message
    Serial.write(message_serial, sizeof(message_serial));
    
    write_I2C();
   
    counter = 0;
  } else {
    counter += 1;
  }
}
