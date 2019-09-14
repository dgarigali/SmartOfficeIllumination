#include <boost/circular_buffer.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#ifndef desk_H
#define desk_H

#define num_readings 60 //Buffer size
#define nominal_power 1 //Watts

using namespace std;
using namespace boost;

class desk {
	
	private:
	
		//directly from I2C
		unsigned int ID;
		circular_buffer <float> illuminance;
		circular_buffer <float> duty_cycle;
		bool occupancy;
		float lower_bound, ext_illum, reference; 
		
		//calculated from I2C data
		float power, energy, comfort_error, flicker;
		void calculations();
		
		//time
		boost::posix_time::ptime initial_time, current_time;
		
		//number of data points
		unsigned long num_data;
		
		//flag for indicating new data
		bool data_flag;
		
	public:
				
		desk();
		unsigned int get_ID();
		void set_parameters(unsigned int node_num, float illum, float d, bool o, float lower_b, float ext, float ref);
		void set_data_flag(bool flag);
		float get_illuminance();
		float get_illuminance(unsigned int pos);
		float get_duty_cycle();
		float get_duty_cycle(unsigned int pos);
		bool get_occupancy();
		float get_lower_bound();
		float get_ext_illum();
		float get_reference();
		float get_power();
		float get_energy();
		float get_comfort_error();
		float get_flicker();
		unsigned int get_time();	
		unsigned int get_illum_buff_size();
		unsigned int get_d_cycle_buff_size();
		bool get_data_flag();
};

#endif