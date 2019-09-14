#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <vector>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/thread/mutex.hpp>

#include "server.cpp"
#include "I2C.cpp"
#include "desk.h"

using namespace std;
using namespace boost;
using namespace boost::asio;

//Initialize desk and mutex vectors
vector<desk> desk_vector; 
ptr_vector<mutex> mutex_vector;
mutex cout_mutex;

int main(int argc, char* argv[]) {
	
	//Check user input
	if (argc != 2) { 
		cerr << "Usage: server <port>\n"; 
		return 1; 
	}
	
	try {

		//Server thread
		io_service io;
		server s(io, atoi(argv[1]));
		thread ServerThread ([&]{ io.run(); });
		
		//I2C thread
		thread I2CThread { start_I2C };
		
		//Wait threads to end
		ServerThread.join(); 
		I2CThread.join(); 
		
	} catch (std::exception& e) {
		std::cerr << e.what() << endl;
	}
	return 0;
}