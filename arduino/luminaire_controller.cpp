#include "luminaire_controller.h"
#include <math.h>

void luminaire_controller::set_feedforward_parameters(float o, float gain) {
	ext_dist = o;
	Go = gain;
}

void luminaire_controller::set_PI_parameters(float kp, float ki, int sample_time) {
	k1 = kp;
	k2 = kp * ki * sample_time / 2000;
}

void luminaire_controller::set_simulator_coefficients(float coeff_5, float coeff_4, float coeff_3, float coeff_2, float coeff_1, float coeff_0) {
	a5 = coeff_5;
	a4 = coeff_4;
	a3 = coeff_3;
	a2 = coeff_2;
	a1 = coeff_1;
	a0 = coeff_0;
}

void luminaire_controller::configure_simulator(unsigned long current_time, float final_voltage, float des_illum) {
	initial_time = current_time;
	vi = vf;
	vf = final_voltage;
	tau = a5 * pow(des_illum, 5) + a4 * pow(des_illum, 4) + a3 * pow(des_illum, 3) + a2 * pow(des_illum, 2) + a1 * pow(des_illum, 1) + a0;
	if (tau < 0) { //In case user inputs higher or lower desired illuminance that allowed
		tau = 0;
	}
}

void luminaire_controller::set_initial_voltage(float initial_voltage) {
	vi = initial_voltage;
}

//Updates feedforward controller each time new desired value is entered!
void luminaire_controller::feedforward_controller(float des_illum) {
	//Implement feedforward controller
	u_feedforward = (des_illum - ext_dist) / Go;
	//Feedforward saturation (helps anti-windup when changing reference illuminance with box opened due to different static gain)
	if (u_feedforward > 5) {
		u_feedforward = 5;
	} else if (u_feedforward < 0) {
		u_feedforward = 0;
	}
}

//Makes a delay for the feedback controller based on estimated time constant
void luminaire_controller::simulator(unsigned long current_time) {
	unsigned long val = current_time - initial_time;
	v_ref = vf - (vf - vi) * exp((val / tau)*(-1));
}

void luminaire_controller::set_feedback_min_error(float e) {
	min_error = e;
}

//Function must be used in case feedforward controller is replaced
void luminaire_controller::set_feedforward_voltage(float voltage) {
  u_feedforward = voltage;
  //Feedforward saturation (helps anti-windup when changing reference illuminance with box opened due to different static gain)
  if (u_feedforward > 5) {
    u_feedforward = 5;
  } else if (u_feedforward < 0) {
    u_feedforward = 0;
  }
}

//Function must be used in case simulator is not used before feedback controller
void luminaire_controller::set_reference_voltage(float reference_voltage) {
	v_ref = reference_voltage;
}

//Corrects system error based on discrete PI controller (especially useful when having external disturbances)
void luminaire_controller::feedback_controller(float current_voltage) {

	//Calculate error
	e = v_ref - current_voltage;
	
	//Error deadzone
	if (e >= min_error) {
		e = e - min_error;
	} else if (e <= -min_error) {
		e = e + min_error;
	} else {
		e = 0;
	}

	//Calculate PI components
	float p = k1 * (v_ref - current_voltage);
	i = i_ant + k2 * (e + e_ant);
	u_feedback = p + i;
}

float luminaire_controller::get_feedforward_voltage() {
	return u_feedforward;
}
	
float luminaire_controller::get_feedback_voltage() {
	return u_feedback;
}

void luminaire_controller::set_controller_voltage(float voltage) {
	u = voltage;
}

void luminaire_controller::set_anti_windup_limits(int lim_min, int lim_max) {
	u_min = lim_min;
	u_max = lim_max;
}
	
//Stops integral controller to always increase (or decrease) after system saturation (after saturation, it retains former value)
void luminaire_controller::anti_windup() {
	if (u > u_max) {
		u = u_max;
		i = i_ant;
	} else if (u < u_min) {
		u = u_min;
		i = i_ant;
	}
}

float luminaire_controller::get_controller_voltage() {
	return u;
}

void luminaire_controller::update_feedback() {
	i_ant = i;
	e_ant = e;
}
