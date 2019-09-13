#include "dist_controller.h"
#include <math.h>
#include <Wire.h>

dist_controller::dist_controller(float des_illuminance, int cost, int node_index, float p, int array_size, int max_duty_cycle) {
	num_elem = array_size;
	d = new float[num_elem];
	d_av = new float[num_elem];
	y = new float[num_elem];
	k = new float[num_elem];
	d_aux = new float[num_elem];
  d_av_aux = new float[num_elem];
	z = new float[num_elem];
  message = new byte[2*num_elem + 2];	
	l = des_illuminance;
	c = cost;
	index = node_index;
	rho = p;
  d_max = max_duty_cycle;
}

dist_controller::~dist_controller() {
	delete [] d;
	delete [] d_av;
	delete [] y;
	delete [] k;
	delete [] d_aux;
	delete [] z;	
  delete [] message;
}

void dist_controller::configuration(float k_array[], float ext_illuminance) {
  for (int i = 0; i < num_elem; i++) {
    n += pow(k_array[i], 2);
    k[i] = k_array[i];
    d[i] = 0;
    d_av[i] = 0;
    y[i] = 0;
  }
  m = n - pow(k_array[index], 2);
  o = ext_illuminance;
}

bool dist_controller::check_feasibility(float d_array[]) {
	float tol = 0.001;
	float d_minus_k = 0;
	for (int i = 0; i < num_elem; i++) {
		d_minus_k += d_array[i] * k[i];
	}
	if (d_array[index] < 0 - tol or d_array[index] > d_max + tol or d_minus_k < l - o - tol) {
		return false;
	} else {
		return true;
	}	
}

float dist_controller::evaluate_cost(float d_array[]) {
	float y_times_d_minus_dav = 0, norm_d_dav = 0, final_cost;
	for (int i = 0; i < num_elem; i++) {
		y_times_d_minus_dav += y[i] * (d_array[i] - d_av[i]);
		norm_d_dav += pow(d_array[i] - d_av[i], 2);
	}
	final_cost = c*d_array[index] + y_times_d_minus_dav + norm_d_dav*rho/2;
	return final_cost;
}

bool dist_controller::compute_constrain(float d_array[]) {
	if (check_feasibility(d_array)) {
		float cost = evaluate_cost(d_array);
		if (cost < best_c) {
			for (int i = 0; i < num_elem; i++) {
				d[i] = d_array[i];
				best_c = cost;
			}
			return false;
		}
	}
	return true;
}

void dist_controller::calculate_z() {
	k_times_z = 0;
	for (int i = 0; i < num_elem; i++) {
		if (i == index) {
			z[i] = d_av[i] - y[i]/rho - c/rho;
		} else {
			z[i] = d_av[i] - y[i]/rho;
		}
		d_aux[i] = z[i];
		k_times_z += k[i] * z[i];
	}
}

void dist_controller::linear_boundary_vector() {
	for (int i = 0; i < num_elem; i++) {
		d_aux[i] = z[i] - k[i]*(o - l + k_times_z)/n;
	}
}

void dist_controller::linear_boundary_0_vector() {
	for (int i = 0; i < num_elem; i++) {
		if (i == index) {
			d_aux[i] = 0;
		} else {
			d_aux[i] = z[i] - k[i]*(o - l)/m + k[i]*(k[index]*z[index] - k_times_z)/m;
		}
	}
}

void dist_controller::linear_boundary_100_vector() {
	for (int i = 0; i < num_elem; i++) {
		if (i == index) {
			d_aux[i] = d_max;
		} else {
			d_aux[i] = z[i] - k[i]*(o - l + d_max*k[index])/m + k[i]*(k[index]*z[index] - k_times_z)/m;
		}
	}
}

void dist_controller::routine() {
	
	//Calculate z (simplifies expressions)
	best_c = 1000000;
	calculate_z();
	
	//Compute unconstrained minimum
	if (compute_constrain(d_aux)) {
		
		//compute minimum constrained to 0 boundary
		d_aux[index] = 0;
		compute_constrain(d_aux);
		
		//compute minimum constrained to 100 boundary
		d_aux[index] = d_max;
		compute_constrain(d_aux);
		
		//compute minimum constrained to linear boundary
		linear_boundary_vector();
		compute_constrain(d_aux);
		
		//compute minimum constrained to linear and 0 boundary
		linear_boundary_0_vector();
		compute_constrain(d_aux);
		
		//compute minimum constrained to linear and 100 boundary
		linear_boundary_100_vector();
		compute_constrain(d_aux);	
	}
}

float dist_controller::get_voltage() {
	return d[index];
}

float dist_controller::get_illuminance() {
  return l_av;
}

float dist_controller::get_k(int index) {
  return k[index];
}

void dist_controller::send_consensus() {

  byte payload[num_elem*2 + 3];
  
  //Encode current averages in bytes
  payload[0] = 1; //ID
  payload[1] = 1; //MessageID
  payload[sizeof(payload) - 1] = consensus_flag; //Flag
  for (int j = 1; j < num_elem + 1 ; j++) { //Duty cycles
    int d_int = (int) ((d[j-1]+0.005)*100);
    payload[2*j] = highByte(d_int);
    payload[2*j + 1] = lowByte(d_int);
  }

  //Send message with current averages
  Wire.beginTransmission(0); //get BUS
  Wire.write(payload, sizeof(payload));
  Wire.endTransmission();
  
}

void dist_controller::actuation() {

  for (int i = 0; i < 100; i++) {

    //Calculate new duty cycle
    routine();

    //Check if node satisfied own need
    if(i != 0 and abs(d[index] - d_av[index]) < 0.002) {
      consensus_flag = 1;
    } else {
      consensus_flag = 0;
    }

    //Update new duty cycle in average
    for (int j = 0; j < num_elem; j++) {
      d_av_aux[j] = d[j];  
    }

    if (index == 0) { //master 
      send_consensus();
    }

    //Wait for other slaves reply
    while(not wait_flag) {
      compute_message();
    }
    wait_flag = false;

    //Update lagrangians
    for (int j = 0; j < num_elem; j++) {
      d_av[j] = d_av_aux[j]/num_elem;
      y[j] += rho*(d[j]-d_av[j]);
    }

    //Check if all nodes has already reached consensus
    if (consensus_flag == 1) {
      break;
    } 
  }

  //Update illuminance reference
  l_av = 0;
  for (int i = 0; i < num_elem; i++) {
    l_av += k[i]*d_av[i];
  } 
  l_av += o;
}

//Receive character from I2C
void dist_controller::receive_byte(byte received_byte, bool flag) {
  message[byte_num] = received_byte;
  if (flag) {
    message_flag = true;
    byte_num = 0;
  } else {
    byte_num++;
  }
}

//Compute I2C data
bool dist_controller::compute_message() {

  if (message_flag) {

    if (message[0] == 0) { //Reference request
      if (index != 0) { //Slave
        byte ref_message[2] = {1, 2};
        Wire.beginTransmission(0); //get BUS
        Wire.write(ref_message, 2);
        Wire.endTransmission();
      }
      message_flag = false;
      actuation();
      return true;
      
    } else if (message[0] == 1) { //consensus

      for (int i = 1; i < num_elem + 1; i++) { //decode bytes to float
        int value_int = message[2*i-1] << 8 | message[2*i]; 
        d_av_aux[i-1] += float(value_int)/100;
      }

      if (index != 0) { //slave
        send_consensus(); 
      }

      //Check if other nodes have satisfied their needs
      consensus_flag = consensus_flag and message[2*num_elem + 1];

      //Check if all nodes have replied
      count_node_resp++; 
      if (count_node_resp == num_elem - 1) {
        count_node_resp = 0;
        wait_flag = true;
      } 
    
    } else if (message[0] == 2) { //Reference reply
      reference_flag = true;
    }

    message_flag = false;
  }
  return false;
}

void dist_controller::reference_change() {
  byte data[2] = {1, 0};
  Wire.beginTransmission(0); //get BUS
  Wire.write(data, 2);
  Wire.endTransmission();
  while(index == 0 and not reference_flag) {
    compute_message();
  }
  reference_flag = false;
  actuation();
}

void dist_controller::set_des_illuminance(float reference) {
  l = reference;
}

void dist_controller::set_cost(int cost) {
  c = cost;
}

float dist_controller::get_dimming_level(int index) {
  return d[index];
}

float dist_controller::set_external_disturbance(float disturbance) {
  o = disturbance;
}
