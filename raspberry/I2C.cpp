#include <pigpio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <vector>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/thread/mutex.hpp>

#include "desk.h"

#define address 0

using namespace std;
using namespace boost;

//Get desk and mutex vectors from main
extern vector<desk> desk_vector; 
extern ptr_vector<mutex> mutex_vector;

//Init I2C slave
void init_slave(bsc_xfer_t &xfer, int addr) {
	gpioSetMode(18, PI_ALT3);
	gpioSetMode(19, PI_ALT3);
	xfer.control = (addr<<16) | /* Slave address */
	(0x00<<13) | /* invert transmit status flags */
	(0x00<<12) | /* enable host control */
	(0x00<<11) | /* enable test fifo */
	(0x00<<10) | /* invert receive status flags */
	(0x01<<9) | /* enable receive */
	(0x01<<8) | /* enable transmit */
	(0x00<<7) | /* abort and clear FIFOs */
	(0x00<<6) | /* send control reg as 1st I2C byte */
	(0x00<<5) | /* send status regr as 1st I2C byte */
	(0x00<<4) | /* set SPI polarity high */
	(0x00<<3) | /* set SPI phase high */
	(0x01<<2) | /* enable I2C mode */
	(0x00<<1) | /* enable SPI mode */
	0x01 ; /* enable BSC peripheral */
} 

//Check if desk is already in vector (return position + 1 if exists, otherwise returns 0)
unsigned int check_node(unsigned int node) {
	for (int i = 0; i < desk_vector.size(); i++) {
		if (desk_vector[i].get_ID() == node) {
			return i+1;
		}
	}
	return 0;
}

//Read I2C data and stored it in respective desk vector
void start_I2C(){  

	//Check gpio initialization
	if (gpioInitialise() < 0) {
		cout << "Error Initialising pigpio" << endl;
		return;
	}
   
   //Init I2C slave
   bsc_xfer_t xfer;
   init_slave(xfer, address);

    //Loop
    while (1) { 
      
		//Check for I2C data
		bscXfer(&xfer);
		if(xfer.rxCnt > 0) {
        
			//Check if it is a log message
			if (unsigned(xfer.rxBuf[0]) == 2) {
				
				//Node number
				unsigned int node = unsigned(xfer.rxBuf[1]);
				
				//Current illuminance
				float illum = float(unsigned(xfer.rxBuf[2]) << 8 | unsigned(xfer.rxBuf[3]))/100 ; 

				//Current duty cycle
				float d_cycle = float(unsigned(xfer.rxBuf[4]) << 8 | unsigned(xfer.rxBuf[5]))/100; 

				//Desk occupancy state
				bool state = unsigned(xfer.rxBuf[6]);

				//Illuminance lower bound
				float lower_b = float(unsigned(xfer.rxBuf[7]) << 8 | unsigned(xfer.rxBuf[8]))/100; 

				//External illuminance
				float ext_illum = float(unsigned(xfer.rxBuf[9]) << 8 | unsigned(xfer.rxBuf[10]))/100; 

				//Illuminance reference
				float reference = float(unsigned(xfer.rxBuf[11]) << 8 | unsigned(xfer.rxBuf[12]))/100; 

				//Create desk and mutex vectors for new node
				unsigned int position = check_node(node);
				if (position == 0) {
					desk_vector.push_back(desk());
					mutex_vector.push_back(new mutex);
					position = desk_vector.size() - 1;
				} else {
					position --;
				}
				
				//Insert desk data
				mutex_vector[position].lock();
				desk_vector[position].set_parameters(node, illum, d_cycle, state, lower_b, ext_illum, reference);
				desk_vector[position].set_data_flag(1);
				mutex_vector[position].unlock();
			}
		}
	}
	gpioTerminate();
} 