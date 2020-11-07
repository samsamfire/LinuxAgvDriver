#ifndef H_MOTOR_DRIVER
#define H_MOTOR_DRIVER

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdint.h>

//#include <string>


#include "can_message.h"
#include <thread>
#include <chrono>


#define TIMEOUT 50 //This is the time before connection is decided to be lost between pc and microcontroller

class Motor
{
	public:
		Motor(int motor_addess);




		/*Read certain number of bytes on can bus*/
		void readCAN();



		int16_t getVel(); //Gets the value
		uint16_t getPos();
		int16_t getTorque();

		void readPosVel();
		bool readEncoder(); //Reads from picd

		bool sendStart();
		void sendStop();

		void startDriver();
		void stopDriver();

		void writeVel(int16_t vel);
		void writePos(uint16_t pos);
		bool getState();

		bool setHdl(int s);
		uint8_t getHdl();
		int getAdress();
		bool getConnexionState();

		int16_t readEncoderDirect();
		


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
		uint8_t connection_state;
		int address;
		//CAN variables
		uint8_t update_rate_can;//Rate at which info is sent from pic default 50Hz
		struct can_frame frame;
		uint8_t nbytes;
		bool receiving_msgs;
		int s; //Socket Handle 
		uint8_t timeout;

		//PID parameters to add


		

};











	


















#endif