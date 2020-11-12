#include "../include/Motor_Driver_DSPIC33.h"



Motor::Motor(int motor_address) {
	//Add constructor stuff
	this->address = motor_address;

	this->vel_encoder = 0;
	this->connection_state = 0;
	this->timeout = 0;
	this->nbMessagesReceived = 0;
	//connection_stat = atomic{false};

}



void Motor::readCAN(){

	uint8_t nbytes = 0;
	int retval = 0;
	int id = 0;
	

	while(connection_state) {

		FD_ZERO(&rfds);
	    FD_SET(s, &rfds);
	    tv.tv_sec = 0;
	    tv.tv_usec = TIMEOUT;

		retval = select(s+1, &rfds, NULL, NULL, &tv);

		if( retval == -1){
			perror("select() error");
		}

		if(FD_ISSET(s, &rfds)){
			nbytes = read(s,&frame,sizeof(frame));
			receiving_msgs = 1;
			id = frame.can_id & 0x7F;
			switch(id){

				case POS_VEL_TORQUE_SENS_ID :
				{
					pos_encoder = (frame.data[1] << 8) + frame.data[0];
					vel_encoder = (frame.data[3] << 8) + frame.data[2];
					torque_encoder = (frame.data[5] << 8) + frame.data[4];
					nbMessagesReceived ++;	
					break;
				}


				default:
				{
					break;
				}
			}
			
		}

		else{
			timeout = 1;
			printf("Driver %i timeout \r\n",address );
			connection_state = 0;
			
		}
	}
	return;

	
}


bool Motor::startDriver(){

	//Thread is started when driver is started
	connection_state = 1;
	timeout = 0;
	if (createSocket() == 1)
	{
		printf("Failed to create socket\r\n");
		return false;
	}
	
	mThread = std::thread(&Motor::readCAN,this);

	usleep(100000);
	if(receiving_msgs == 1){
		printf("Driver with address %i has been detected\r\n",address);
		if(sendStart() == 1){
			printf("Start request sent to motor %i\r\n",address);

			return true;
		}
		else{
			//ADD error if sendStart doesn't work
			return false;
		}
		
	}
	else{
	
		connection_state = 0;
		usleep(1000);
		printf("No driver has been detected on Bus with address %i \r\n",address);
		if(mThread.joinable()){
			mThread.join();

		}
		return false;
	}

	
}

bool Motor::stopDriver(){
	
	if(connection_state == 1){
		if(sendStop()==true){
			printf("Sending stop to motor %i\r\n", address);
		}
		else{
			//Add error checking 
		}

		

	}

	else{
		printf("Connection timed out, attempting to stop\r\n");
		if(sendStop()==true){
			printf("Sent stop to motor %i\r\n", address);
			
		}

		else{

			printf("Failed to send stop to motor\n");
		}

	}
	connection_state = 0;
	usleep(10000);
	if(mThread.joinable()){
		mThread.join();

	}
	return closeSocket();

}



bool Motor::restartDriver(){

	if(state == 1){
		
		return startDriver() and stopDriver();
	}

	return false;
}



void Motor::endThread(){

	if(mThread.joinable()){

		mThread.join();
	}
}





bool Motor::createSocket(){

	//Code from waveshare demo https://www.waveshare.com/wiki/RS485_CAN_HAT

	//1.Create socket 
    	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    	if (s < 0) {
	        perror("socket PF_CAN failed");
	        return 1;
    	}

    	//2.Set socket to non-blocking

    	//fcntl(s, F_SETFL, O_NONBLOCK);
    	


    	//3.Specify can0 device
	    strcpy(ifr.ifr_name, "can0");
	    ret = ioctl(s, SIOCGIFINDEX, &ifr);
	    if (ret < 0) {
	        perror("ioctl failed");
        	return 1;
        }


        //4.Bind the socket to can0
	    addr.can_family = AF_CAN;
	    addr.can_ifindex = ifr.ifr_ifindex;
	    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
	    if (ret < 0) {
	        perror("bind failed");
	        return 1;
	    }

	    //5.Define receive rules
	    rfilter.can_id = (getAdress() << 7); //Ids beggining with correct address
    	rfilter.can_mask = (15 << 7);//All lower bits are don't cares
    	printf("Added filter to motor address : %i \r\n",getAdress()<<7);
    	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
	

		//6.Give to motors socket handle

		setHdl(s);
		socket_state = 1;

		return 0;



}

bool Motor::closeSocket(){

	if(socket_state == 1){
		close(s);
		return true;
	}

	else{

		return false;
	}

}











bool Motor::sendStart(){

	frame.can_id = (address << 7) | START_ID;
	frame.can_dlc = 0;
	write(s, &frame, sizeof(frame));

	/*Todo perform checks*/
	state = 1;

	return true;
}


bool Motor::sendStop(){
	frame.can_id = (address << 7) | STOP_ID;
	frame.can_dlc = 0;
	write(s, &frame, sizeof(frame));

	state = 0;
	return true;
}

bool Motor::writeVel(int16_t vel){
	frame.can_id = (address << 7) | SET_VEL_ID ;
	frame.can_dlc = 2;
	frame.data[0] = vel & 0xFF;
	frame.data[1] = vel >> 8;
	write(s,&frame,sizeof(frame));

	return true;

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


double Motor::getElapsedTime(){

	return elapsed_time_ms;
}

int Motor::getNbReceived(){

	return nbMessagesReceived;
}

bool Motor::setHdl(int s){

	this->s=s;
	return true;
}

bool Motor::getConnectionState(){

	return connection_state;
}