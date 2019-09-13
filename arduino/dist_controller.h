#include <Arduino.h>

#ifndef dist_controller_H
#define dist_controller_H

class dist_controller {
	private:
		int num_elem, byte_num = 0, c;
		float * d, * d_av, * y, * k, *d_av_aux;
		float n = 0, m, o, rho, l;
		int index, d_max;
		float best_c, l_av = 0;
		float * d_aux, * z, k_times_z;
		volatile bool message_flag = false;
		bool wait_flag = false, reference_flag = false;
		int count_node_resp = 0;   
		byte * message; 
		byte consensus_flag = 0;
		bool check_feasibility(float d_array[]);
		float evaluate_cost(float d_array[]);
		bool compute_constrain(float d_array[]);
		void calculate_z();
		void linear_boundary_vector();
		void linear_boundary_0_vector();
		void linear_boundary_100_vector();
		void send_consensus();
		void routine();
		
	public:
		dist_controller(float des_illuminance, int cost, int node_index, float p, int array_size, int max_duty_cycle);
		~dist_controller();
		void configuration(float k_array[], float ext_illuminance);
		float get_voltage();
		float get_illuminance();
		float get_k(int index);
		void actuation();
		void receive_byte(byte received_byte, bool flag);
		bool compute_message();
		void set_des_illuminance(float reference);
		void set_cost(int cost);
		void reference_change();
		float get_dimming_level(int index);
		float set_external_disturbance(float disturbance);
};

#endif
