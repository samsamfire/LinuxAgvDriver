#include "../include/Motor_Driver_DSPIC33.h"



Motor::Motor(int motor_address) {
	//Add constructor stuff
	this->address = motor_address;

	vel_encoder = 0;
	connection_state = 0;
	timeout = 0;

}


void Motor::readCAN(){

	uint8_t nbytes = 0;
	clock_t current_clock,previous_clock;
	auto t_start = std::chrono::high_resolution_clock::now();
	// the work...
	auto t_end = std::chrono::high_resolution_clock::now();
	double elapsed_time_ms = 0;
	

	int id = 0;

	while(1) {
		nbytes = read(s,&frame,sizeof(frame));

		if(nbytes>=0){
			id = frame.can_id & 0x7F;
			switch(id){

				case POS_VEL_TORQUE_SENS_ID :
				{
					t_start = std::chrono::high_resolution_clock::now();
					pos_encoder = (frame.data[1] << 8) + frame.data[0];
					vel_encoder = (frame.data[3] << 8) + frame.data[2];
					torque_encoder = (frame.data[5] << 8) + frame.data[4];	
					break;
				}


				default:
				{
					break;
				}
			}

			//Detect disconnect
			t_end = std::chrono::high_resolution_clock::now();
			elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
			receiving_msgs = 1;

			if(elapsed_time_ms > TIMEOUT ){
				stopDriver();
				timeout = 1;
			}


			
		}
	}
}



void Motor::startDriver(){

	//Thread is started when driver is started
	mThread = std::thread(&Motor::readCAN,this);

	usleep(100000);
	if(receiving_msgs == 1){
		printf("Driver with address %i has been detected\r\n",address);
		if(sendStart() == 1){
			printf("Start request sent to motor %i\r\n",address);
		}
		else{
			//ADD
		}
		connection_state = 1;
	}
	else{
		mThread.join();
		printf("No driver has been detected on Bus with address %i \r\n",address);
	}
	
}

void Motor::stopDriver(){

	if(mThread.joinable()){
		mThread.join();
	}
	if(timeout == 1){
		printf("Driver %i timeout \r\n",address );
	}
	if(connection_state == 1){
		sendStop();
		printf("Sending stop to motor %i\r\n", address);
	}
	else{
		printf("Driver stopped but connection_state is 0\r\n");
	}
	
	connection_state = 0;
}



bool Motor::sendStart(){

	frame.can_id = (address << 7) | START_ID;
	frame.can_dlc = 0;
	write(s, &frame, sizeof(frame));

	/*Todo perform checks*/
	state = 1;

	return true;
}


void Motor::sendStop(){
	frame.can_id = (address << 7) | STOP_ID;
	frame.can_dlc = 0;
	write(s, &frame, sizeof(frame));

	state = 0;
}

void Motor::writeVel(int16_t vel){
	frame.can_id = (address << 7) | SET_VEL_ID ;
	frame.can_dlc = 2;
	frame.data[0] = vel & 0xFF;
	frame.data[1] = vel >> 8;
	write(s,&frame,sizeof(frame));

}


void Motor::writePos(uint16_t pos){

	frame.can_id = (address << 7) | SET_POS_ID ;
	frame.can_dlc = 2;
	frame.data[0] = pos & 0xFF;
	frame.data[1] = pos >> 8;
	write(s,&frame,sizeof(frame));


}



int Motor::getAdress(){

	return address;
}

int16_t Motor::getVel(){

	return vel_encoder;
}

uint16_t Motor::getPos(){

	return pos_encoder;
}

int16_t Motor::getTorque(){

	return torque_encoder;
}

bool Motor::getState(){

	return state;
}

uint8_t Motor::getHdl(){

	return s;
}


bool Motor::setHdl(int s){

	this->s=s;
	return true;
}

bool Motor::getConnexionState(){

	return connection_state;
}