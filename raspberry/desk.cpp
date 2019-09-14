#include "desk.h"

//Constructor
desk::desk() : illuminance(num_readings), duty_cycle(num_readings) {
	initial_time = boost::posix_time::second_clock::local_time();
	energy = 0;
	num_data = 0;
	comfort_error = 0;
	flicker = 0;
}

//Set direct parameters
void desk::set_parameters(unsigned int node_num, float illum, float d, bool o, float lower_b, float ext, float ref) {
	
	//Direct I2C values
	ID = node_num;
	illuminance.push_back(illum);
	duty_cycle.push_back(d);
	occupancy = o;
	lower_bound = lower_b;
	ext_illum = ext;
	reference = ref;
	
	//Calculated values from I2C
	calculations();
}

//Performance metric calculations
void desk::calculations() {
	
	//Update data points number
	num_data++;
	
	//Power
	power = nominal_power*(duty_cycle[duty_cycle.size()-1]);
	
	//Accumulated energy
	boost::posix_time::ptime current_time_aux = boost::posix_time::second_clock::local_time();
	boost::posix_time::time_duration interval = current_time_aux - current_time;
	if (num_data > 1) {
		energy += duty_cycle[duty_cycle.size()-2]*interval.total_seconds();
	}
	
	//Accumulated comfort error
	float diff = reference - illuminance[illuminance.size()-1];
	if (diff < 0) {
		diff = 0;
	}
	comfort_error += diff;	
	
	//Accumulated comfort flicker
	if (num_data > 2) {
		float first_diff = abs(illuminance[illuminance.size()-1] - illuminance[illuminance.size()-2]);
		float second_diff = abs(illuminance[illuminance.size()-2] - illuminance[illuminance.size()-3]);
		if (first_diff * second_diff < 0) {
			flicker += (first_diff + second_diff) / (2*interval.total_seconds());
		}
	}
	
	//Time
	current_time = current_time_aux;
	
}

//Set new data flag value
void desk::set_data_flag(bool flag) {
	data_flag = flag;
}

//Return desk number
unsigned int desk::get_ID() {
	return ID;
}

//Return current illuminance
float desk::get_illuminance() {
	return illuminance[illuminance.size()-1];
}

//Return illuminance from certain position
float desk::get_illuminance(unsigned int pos) {
	return illuminance[pos];
}

//Return current duty cycle (in percentage)
float desk::get_duty_cycle() {
	return duty_cycle[duty_cycle.size()-1]*100;
}

//Return duty cycle (in percentage) from certain position
float desk::get_duty_cycle(unsigned int pos) {
	return duty_cycle[pos]*100;
}

//Return current desk occupancy state
bool desk::get_occupancy() {
	return occupancy;
}

//Return current illuminance lower bound 
float desk::get_lower_bound() {
	return lower_bound;
}

//Return current external illuminance
float desk::get_ext_illum() {
	return ext_illum;
}

//Return current illuminance control reference
float desk::get_reference() {
	return reference;
}

//Return current power consumption
float desk::get_power() {
	return power;
}

//Return accumulated energy consumption
float desk::get_energy() {
	return nominal_power*energy;
}

//Return accumulated comfort error
float desk::get_comfort_error() {
	return comfort_error/num_data;
}

//Return accumulated comfort flicker
float desk::get_flicker() {
	return flicker/num_data;
}

//Return elapsed time
unsigned int desk::get_time() {
	boost::posix_time::time_duration interval = current_time - initial_time;
	return interval.total_seconds();
}

//Return size of illuminance circular buffer
unsigned int desk::get_illum_buff_size() {
	return illuminance.size();
}

//Return size of duty cycle circular buffer
unsigned int desk::get_d_cycle_buff_size() {
	return duty_cycle.size();
}

//Return new data flag
bool desk::get_data_flag() {
	return data_flag;
}