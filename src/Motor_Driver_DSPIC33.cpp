#include "../include/Motor_Driver_DSPIC33.h"



Motor::Motor(int motor_address){
	//Add constructor stuff
	this->address = motor_address;
	//Thread is started when driver is started
	std::thread can_read(&Motor::readCAN,this);	

	vel_encoder = 0;
}



void Motor::readCAN(){

	uint8_t nbytes = 0;

	int id = 0;

	while(1) {
		nbytes = read(s,&frame,sizeof(frame));

		if(nbytes>=0){
			id = frame.can_id;
			switch(id){

				case POS_VEL_TORQUE_SENS_ID :
				{
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



		}


	}
}





int16_t Motor::readEncoderDirect(){

	if(read(s, &frame, sizeof(frame))>0){
		vel_encoder = (frame.data[3] << 8) + frame.data[2];
	}
	else{
		printf("Error reading\r\n");
	}

	return vel_encoder;

}


void Motor::readPos(){
	//Add some filtering to get right motor
	read(s, &frame, sizeof(frame));
	pos = (frame.data[1] << 8) + frame.data[0];
}

void Motor::readVel(){
	//Add some filtering to get right motor
	read(s, &frame, sizeof(frame));
	velSens = (frame.data[1] << 8) + frame.data[0];
}

void Motor::readPosEncoder(){
	//Add some filtering to get right motor
	read(s, &frame, sizeof(frame));
	pos_encoder = (frame.data[1] << 8) + frame.data[0];
}

void Motor::readVelEncoder(){
	//Add some filtering to get right motor
	read(s, &frame, sizeof(frame));
	vel_encoder = (frame.data[1] << 8) + frame.data[0];
}

bool Motor::readEncoder(){

	/*TODO add an error mechanism*/
	read(s, &frame, sizeof(frame));

	pos_encoder = (frame.data[1] << 8) + frame.data[0];
	vel_encoder = (frame.data[3] << 8) + frame.data[2];
	torque_encoder = (frame.data[5] << 8) + frame.data[4];

	return 0;


}



void Motor::start(){

	frame.can_id = (address << 7) | START_ID;
	frame.can_dlc = 0;
	write(s, &frame, sizeof(frame));

	/*Todo perform checks*/
	state = 1;
}


void Motor::stop(){
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



bool Motor::setHdl(int s){

	this->s=s;
	return true;
}

uint8_t Motor::getHdl(){

	return s;
}
