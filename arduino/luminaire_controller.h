#ifndef luminaire_controller_H
#define luminaire_controller_H

class luminaire_controller {
	private:
		float ext_dist, Go, u_feedforward; //Feedforward variables
		float a5, a4, a3, a2, a1, a0; //Simulator coefficientes
		unsigned long initial_time;
		float vi, vf, tau, v_ref; //Simulator variables
		float k1, k2; //PI variables
		float min_error, u_feedback, i, e, i_ant = 0, e_ant = 0; //Feedback variables
		float u; //Global controller voltage
		int u_min, u_max; //Anti-windup variables
		
	public:
		void set_feedforward_parameters(float o, float gain);
		void feedforward_controller(float des_illum);
		void set_simulator_coefficients(float coeff_5, float coeff_4, float coeff_3, float coeff_2, float coeff_1, float coeff_0); 
		void configure_simulator(unsigned long current_time, float final_voltage, float des_illum);
		void set_initial_voltage(float initial_voltage);
		void simulator(unsigned long current_time);
		void set_PI_parameters(float kp, float ki, int sample_time);
		void set_feedback_min_error(float e);
		void set_feedforward_voltage(float voltage);
		void set_reference_voltage(float reference_voltage); //only used if simulator is not used
		void feedback_controller(float current_voltage);
		float get_feedforward_voltage();
		float get_feedback_voltage();
		void set_controller_voltage(float voltage);
		void set_anti_windup_limits(int lim_min, int lim_max);
		void anti_windup();
		float get_controller_voltage();
		void update_feedback();
};

#endif
