#include <iostream>
#include <vector>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp> //is_any_of()

#include "desk.h"

using namespace std;

//Vector for real-time stream
struct real_time {
  bool data; //0 -> illuminance, 1 -> duty cycle
  unsigned int node;
  boost::posix_time::ptime start;
};
vector<real_time> real_time_vector;
mutex real_time_mutex;

//Get desk and mutex vectors from main
extern vector<desk> desk_vector; 
extern ptr_vector<mutex> mutex_vector;
extern mutex cout_mutex;

//Check if desk exists (return position + 1 if exists, otherwise returns 0)
unsigned int check_desk(unsigned int node) {
	for (int i = 0; i < desk_vector.size(); i++) {
		if (desk_vector[i].get_ID() == node) {
			return i+1;
		}
	}
	return 0;
}

//Check if node is a number
bool check_node_num(string node) {
	for (int i = 0; i < node.size(); i++) {
		if (!isdigit(node[i]))
			return false;
	}
	return true;
}

//Check if real time stream has object of specified node starting from certain position
bool check_real_time_stream_objects(unsigned int node, unsigned int pos) {
	unsigned int count = 0;
	for (int i = pos; i < real_time_vector.size(); i++) {
		if (real_time_vector[i].node == node)
			count++;
	}
	return (count > 1);
}

//returns data to be streammed in real time
vector<string> real_time_stream() {
	
	//Init auxiliar variables
	vector<string> aux_vector;	
	stringstream aux_str_stream;
	boost::posix_time::time_duration aux_time;
	
	//Lock real time mutex
	real_time_mutex.lock();
	
	for (int i = 0; i < real_time_vector.size(); i++) {
		
		//Lock desk mutex
		mutex_vector[real_time_vector[i].node].lock();
		
		//Check new data flags
		if (desk_vector[real_time_vector[i].node].get_data_flag()) {
		
			//Create message
			aux_time = boost::posix_time::microsec_clock::local_time() - real_time_vector[i].start;
			if (real_time_vector[i].data) //duty cycle
				aux_str_stream << "s d " << desk_vector[real_time_vector[i].node].get_ID() << " " << desk_vector[real_time_vector[i].node].get_duty_cycle() << " " << aux_time.total_milliseconds() << "\n";
			else //illuminance
				aux_str_stream << "s l " << desk_vector[real_time_vector[i].node].get_ID() << " " << desk_vector[real_time_vector[i].node].get_illuminance() << " " << aux_time.total_milliseconds() << "\n";
			
			//Add string to string vector
			aux_vector.push_back(aux_str_stream.str());
			
			//Clear stringstream
			aux_str_stream.str("");
			aux_str_stream.clear();
			
			//Change flag if all data from node was included in vector
			if (!check_real_time_stream_objects(real_time_vector[i].node, i))
				desk_vector[real_time_vector[i].node].set_data_flag(0);
		}
		
		//Unlock desk mutex
		mutex_vector[real_time_vector[i].node].unlock();
	}
	
	//Unlock real time mutex
	real_time_mutex.unlock();

	//Return string vector
	return aux_vector;
}

//Returns pos+1 to stop real time stream or 0 otherwise (to start it)
unsigned int check_real_time_stream(bool data, unsigned int node) {
	for (int i = 0; i < real_time_vector.size(); i++) {
		if (real_time_vector[i].data == data and real_time_vector[i].node == node)
			return i+1;
	}
	return 0;
}

string decode_message(string message) {
	
	//Prints message to console
	cout_mutex.lock();
	cout << "Message from client: " << message;
	cout_mutex.unlock();
	
	//Split message
	stringstream response;
	vector<string> message_splitted;
	split(message_splitted, message, is_any_of(" "));
	
	//Check message size
	if (message_splitted.size() == 3) {
		
		//Decode message
		string code = message_splitted[0];
		string type = message_splitted[1];
		string node = message_splitted[2];
		node.erase(remove(node.begin(), node.end(), '\n'), node.end()); //remove end of line
		
		//Check code and type sizes
		if (code.size() == 1 and type.size() == 1) {

			//Check if it is a total request
			if (code == "g" and node == "T") {
				
				//Lock mutexes
				float total = 0;
				for (int i = 0; i < desk_vector.size(); i++)
					mutex_vector[i].lock();
				
				switch(type[0]) {
					
					//Total power consumption
					case 'p':
						for (int i = 0; i < desk_vector.size(); i++)
							total += desk_vector[i].get_power();
						response << "p T " << fixed << setprecision(2) << total << "\n";
						break;
						
					//Total accumulated energy consumption
					case 'e':
						for (int i = 0; i < desk_vector.size(); i++)
							total += desk_vector[i].get_energy();
						response << "e T " << fixed << setprecision(2) << total << "\n";
						break;
						
					//Total accumulated comfort error
					case 'c':
						for (int i = 0; i < desk_vector.size(); i++)
							total += desk_vector[i].get_comfort_error();
						response << "c T " << fixed << setprecision(2) << total << "\n";
						break;
						
					//Total accumulated comfort flicker
					case 'v':
						for (int i = 0; i < desk_vector.size(); i++)
							total += desk_vector[i].get_flicker();
						response << "v T " << fixed << setprecision(2) << total << "\n";
						break;
	
					default:
						response << "Wrong variable of get command for all desks \n";
				}
				
				//Unlock mutexes
				for (int i = 0; i < desk_vector.size(); i++)
					mutex_vector[i].unlock();
			
			//Check if it is a individual node request
			} else if (check_node_num(node)) {

				//Check if desk exists
				unsigned int position = check_desk(stoul(node));
				if (position != 0) {
					
					//Lock mutex
					position --;
					mutex_vector[position].lock();
					
					switch(code[0]) {
						
						//Get command
						case 'g':
						
							switch(type[0]) {
								
								//Current measured illuminance
								case 'l':
									response << "l " << node << " " << fixed << setprecision(2) << desk_vector[position].get_illuminance() << "\n";
									break;
									
								//Current duty cycle
								case 'd':
									response << "d " << node << " " << fixed << setprecision(2) << desk_vector[position].get_duty_cycle() << "\n";
									break;
									
								//Current occupancy state
								case 's':
									response << "s " << node << " " << fixed << setprecision(2) << desk_vector[position].get_occupancy() << "\n";
									break;
									
								//Current illuminance lower bound
								case 'L':
									response << "L " << node << " " << fixed << setprecision(2) << desk_vector[position].get_lower_bound() << "\n";
									break;
									
								//Current external illuminance
								case 'o':
									response << "o " << node << " " << fixed << setprecision(2) << desk_vector[position].get_ext_illum() << "\n";
									break;
									
								//Current illuminance control reference
								case 'r':
									response << "r " << node << " " << fixed << setprecision(2) << desk_vector[position].get_reference() << "\n";
									break;

								//Current power consumption
								case 'p':
									response << "p " << node << " " << fixed << setprecision(2) << desk_vector[position].get_power() << "\n";
									break;	

								//Elapsed time
								case 't':
									response << "t " << node << " " << fixed << setprecision(2) << desk_vector[position].get_time() << "\n";
									break;	

								//Accumulated energy consumption
								case 'e':
									response << "e " << node << " " << fixed << setprecision(2) << desk_vector[position].get_energy() << "\n";
									break;	

								//Accumulated comfort error
								case 'c':
									response << "c " << node << " " << fixed << setprecision(2) << desk_vector[position].get_comfort_error() << "\n";
									break;	

								//Accumulated comfort flicker
								case 'v':
									response << "v " << node << " " << fixed << setprecision(2) << desk_vector[position].get_flicker() << "\n";
									break;	

								default:
									response << "Wrong variable of get command for individual desk\n";
							}
							break;
							
						//Buffer command
						case 'b':
						
							switch(type[0]) {
								
								//Last minute measured illuminance (or less)
								case 'l':
									response << "b l " << node << " ";
									for (unsigned int i = 0; i < desk_vector[position].get_illum_buff_size(); i++) {
										response << desk_vector[position].get_illuminance(i);
										if (i != desk_vector[position].get_illum_buff_size() - 1)
											response << ",";
									}
									response << "\n";
									break;
									
								//Last minute duty cycle (or less)
								case 'd':
									response << "b d " << node << " "; 
									for (unsigned int i = 0; i < desk_vector[position].get_d_cycle_buff_size(); i++) {
										response << desk_vector[position].get_duty_cycle(i);
										if (i != desk_vector[position].get_d_cycle_buff_size() - 1)
											response << ",";
									}	
									response << "\n";
									break;
									
								default:
									response << "Wrong variable for buffer command\n";
							}
							break;
							
						//Real-time stream command
						case 's':
						
							//Lock real time mutex
							real_time_mutex.lock();
							
							//Check if variable is illuminance or duty cycle
							if (type[0] == 'l' or type[0] == 'd') {
								unsigned int variable_type;
								
								//Check variable type
								if (type[0] == 'l')
									variable_type = 0;
								else 
									variable_type = 1;
								
								//Check if it is for starting real time stream
								unsigned int real_time_pos = check_real_time_stream(variable_type, position);
								if (real_time_pos == 0) {
								
									//Create auxiliar real_time object
									real_time real_time_obj;
									real_time_obj.data = variable_type;
									real_time_obj.node = position;
									real_time_obj.start = boost::posix_time::microsec_clock::local_time();
								
									//Add auxiliar real_time object to vector
									real_time_vector.push_back(real_time_obj);
									response << "";
								
								//Check if it is for stopping real time stream
								} else {
									
									real_time_pos--;
									response << "ack\n";
									
									//Delete real time object from vector
									real_time_vector.erase (real_time_vector.begin() + real_time_pos);
								}

							} else {
								response << "Wrong variable for real time command\n";
							}
							
							//Unlock real time mutex
							real_time_mutex.unlock();							
							break;
							
						default:
							response << "Wrong command for individual desk\n";
						
					}
					
					//Unlock mutex
					mutex_vector[position].unlock();						

				} else {
					response << "Unexistent desk\n";	
				}
				
			} else {
				response << "Wrong command and/or node\n";
			}
	
		} else {
			response << "Wrong command and/or variable\n";
		}
 		
	} else {
		response << "Wrong message format\n";
	}
	
	//if (response.str().empty())
	return response.str();
}