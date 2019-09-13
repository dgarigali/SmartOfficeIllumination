#include <Arduino.h>

#ifndef lum_system_H
#define lum_system_H

class lum_system {
	private:
		//LDR parameters
		unsigned long R1;
		int Vcc, measurements_LDR, LDR_pin;
		float m, b, max_ADC_resolution;
		//LED parameters
		int led_pin, max_DAC_resolution;
		//Calibration parameters
		int node_num, num_nodes, count_node_resp = 0, count_calib = 0, i;
		bool wait_flag = false, resp_master_flag = true;
		volatile bool message_flag = false;
		float * k, ext_dist = 0;
		byte message_id = 255;
		void send_message(byte message[], int slave_address);
		void compute_message();
    
	public:
		~lum_system();
		void set_LDR_parameters(unsigned long resistance, int voltage, float scope, float incline, float resolution, int measurements, int pin);
		void set_LED_parameters(int pin, int resolution);
		void set_calibration_parameters(int id, int total);
		float lux_to_volt(float lux);
		float volt_to_lux(float volt);
		float adc(int value);
		int dac(float value);
		float LDR_voltage();
		void calc_ext_dist();
		float get_gain();
		void calibration_routine();
		void receive_byte(byte received_byte); 
		void calibrate(bool led_on);
		float * get_gains();
		float get_ext_dist();
};

#endif
