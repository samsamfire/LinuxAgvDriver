#ifndef H_MOTOR_DRIVER
#define H_MOTOR_DRIVER

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdint.h>
#include <sys/time.h>
#include <sys/types.h>

//#include <string>


#include "can_message.h"
#include <thread>
#include <chrono>

#include <atomic>


#define TIMEOUT 500000 //This is the time before connection is decided to be lost between pc and microcontroller in us

class Motor
{
	public:
		Motor(int motor_address);
		

		/*Read certain number of bytes on can bus*/
		void readCAN();



		int16_t getVel(); //Gets the value
		uint16_t getPos();
		int16_t getTorque();

		void readPosVel();

		bool sendStart();
		bool sendStop();

		bool startDriver();
		bool restartDriver();
		bool stopDriver();

		bool writeVel(int16_t vel);
		void writePos(uint16_t pos);
		bool getState();

		bool setHdl(int s);
		uint8_t getHdl();
		int getAdress();
		bool getConnectionState();
		int getNbReceived();

		double getElapsedTime();

		bool createSocket();
		bool closeSocket();

		void endThread();
		


	private:

		std::thread mThread;

		//int16_t twist[3];
		int16_t velSens;
		int16_t pos;
		bool state; //On or Off
		uint8_t mode; // pos 1, vel 2, torque 3...
		int16_t vel_encoder;
		uint16_t pos_encoder;
		int16_t torque_encoder;

		bool error; //1 for yes 0 for no

		
		
		bool connection_state;

		int address;
		//CAN variables
		uint8_t update_rate_can;//Rate at which info is sent from pic default 50Hz
		bool receiving_msgs;
		uint8_t timeout;

		double elapsed_time_ms;

		//PID parameters to add




		//CAN bus info

		bool socket_state; 
		int ret;
	    int s,nbytes;
	    struct sockaddr_can addr;
	    struct ifreq ifr;
	    struct can_frame frame;
	    struct can_filter rfilter;
	    int nbMessagesReceived;
	    fd_set rfds;
	    struct timeval tv;


		

};











	


















#endif