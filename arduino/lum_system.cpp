#include "lum_system.h"
#include <math.h>
#include <Wire.h>

//Destructor
lum_system::~lum_system() {
  delete [] k;
}

//Set LDR parameters
void lum_system::set_LDR_parameters(unsigned long resistance, int voltage, float scope, float incline, float resolution, int measurements, int pin) {
  R1 = resistance;
  Vcc = voltage;
  m = scope;
  b = incline;
  max_ADC_resolution = resolution;
  measurements_LDR = measurements;
  LDR_pin = pin;
}

//Set LED parameters
void lum_system::set_LED_parameters(int pin, int resolution) {
  led_pin = pin;
  max_DAC_resolution = resolution;
}

//Set calibration parameters
void lum_system::set_calibration_parameters(int id, int total) {
  node_num = id;
  num_nodes = total;
  k = new float[num_nodes]; 
}

//Convert LDR lux to volt
float lum_system::lux_to_volt(float lux) {
  return ((R1*Vcc)/(pow(10,m*log10(lux)+b)+R1));
}

//Convert LDR volt to lux
float lum_system::volt_to_lux(float volt) {
  return pow(10,((log10((R1*Vcc/volt)-R1)-b)/m));
}

//Analog to digital converter
float lum_system::adc(int value) {
  return value*Vcc/max_ADC_resolution; 
}

//Digital to analog converter (with rounding)
int lum_system::dac(float value) {
  return (int) ((value*max_DAC_resolution/Vcc)+0.5);
}

//Read LDR voltage with average filter
float lum_system::LDR_voltage() {
  
  //Reset variables
  int i = 0;
  float V_R_sum = 0;

  //Measurements with average filter
  for (i = 0; i < measurements_LDR; i++) {
    V_R_sum += adc(analogRead(LDR_pin));
    delayMicroseconds(100);
  }

  //Return average voltage
  return V_R_sum/i;
}

//Calculate external disturbance
void lum_system::calc_ext_dist() {
  ext_dist = volt_to_lux(LDR_voltage());
}

//Calculate static gain
float lum_system::get_gain() {
  return (volt_to_lux(LDR_voltage())-ext_dist)/Vcc;
}

//Calibration routine
void lum_system::calibration_routine() {

  unsigned long current_time;
  if(node_num == 0) { //master 
    while(not wait_flag) {
      //Send message to all slaves and wait for responses
      byte message[2] = {0, 0};
      send_message(message, 0);
      current_time = millis();
      while (millis() - current_time < 1000) {
        compute_message();
      }
    }
    wait_flag = false;
  }

  for(i = 0; i < num_nodes; i++) {
    if (node_num == i) {
      calibrate(true);
      byte message[2] = {0, 2};
      send_message(message, 0);
    }
    while(not wait_flag) {
      compute_message();
    }
    wait_flag = false;
  }
}

//Send message using I2C
void lum_system::send_message(byte message[], int slave_address) {
  Wire.beginTransmission(slave_address); //get BUS
  Wire.write(message, sizeof(message)); //send byte to address on BUS
  Wire.endTransmission(); //release BUS 
}

//Receive character from I2C
void lum_system::receive_byte(byte received_byte) {
  message_id = received_byte;
  message_flag = true;
}

//Compute I2C data
void lum_system::compute_message() {

  if (message_flag) {
  
    if (message_id == 0) { //master request
      if (resp_master_flag) {
        resp_master_flag = false;
        byte message[2] = {0, 1};
        send_message(message, 0);
      }
    } 
    
    else if (message_id == 1 and node_num == 0) { //slave reply
      count_node_resp++;
      if (count_node_resp == num_nodes - 1) {
        count_node_resp = 0;
        wait_flag = true;
      }
    } 
    
    else if (message_id == 2) { //calibrate
      calibrate(false);
      byte message[2] = {0, 3};
      send_message(message, 0);
    } 
    
    else if (message_id == 3 and i == node_num) { //calibration done
      count_node_resp++;
      if (count_node_resp == num_nodes - 1) {
        count_node_resp = 0;
        analogWrite(led_pin, 0);
        delay(100); 
        wait_flag = true;
        byte message[2] = {0, 4};
        send_message(message, 0);
      }
    }
  
    else if (message_id == 4) { //next master
      wait_flag = true;
    }

    message_flag = false;
    
  }
}

//Calibrate (determine static gain)
void lum_system::calibrate(bool led_on) {
  if(led_on) {
    analogWrite(led_pin, max_DAC_resolution);
    delay(100); 
  }
  k[count_calib] = get_gain();
  count_calib++;
}

float lum_system::get_ext_dist() {
  return ext_dist;
}

float * lum_system::get_gains() {
  return k;
}
