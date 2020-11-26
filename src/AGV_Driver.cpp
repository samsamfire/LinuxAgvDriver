#include "../include/AGV_Driver.h"
#include "libsocketcan.h"


AGV::AGV(int ad_fl, int ad_fr, int ad_br, int ad_bl): m{Motor(ad_fl),Motor(ad_fr),Motor(ad_br),Motor(ad_bl)}{

	// int* state;
	//  can_get_state("can0",state);
	//  if(*state == CAN_STATE_BUS_OFF or *state== CAN_STATE_STOPPED){
	//  	//can_do_restart("can0");
	//   	openBus(500000);
	//  }
	
	
	
 }



/*Send desired velocities in world frame with an array*/

void AGV::writeVel( const double vel[4] ){
	
	//Cinematic model:
	this->vel[0] = vel[0];
	this->vel[1] = vel[1];
	this->vel[2] = vel[2];
	this->vel[3] = vel[3];

	double Vx = vel[0];
	double Vy = vel[1];
	double Vz = vel[2];
	double e = vel[3];

	//https://www.fh-dortmund.de/roehrig/papers/roehrigCCTA17.pdf

	

	//0 and 3 are same direction and 1 and 2 same opposite direction to 0 & 3
	//Vx and Vy working, not tested Vz
	wm_cmd.w[0] = (1/Rr)*(Vx-Vy-(La+Lb)*Vz+e)*F*Z;
	wm_cmd.w[1] = (1/Rr)*(-Vx-Vy-(La+Lb)*Vz-e)*F*Z; 
	wm_cmd.w[2] = (1/Rr)*(-Vx+Vy-(La+Lb)*Vz+e)*F*Z;
	wm_cmd.w[3] = (1/Rr)*(Vx+Vy-(La+Lb)*Vz-e)*F*Z;

	//If constraint_controller not active send speeds else, constraint controller does it
	
	for (int i = 0; i < 4; ++i)
	{
		//Check if motor is activated
		if(m[i].getConnectionState() == 1){
			m[i].writeVel(wm_cmd.w[i]);
		}
		else{

			//Try to reconnect
		}
		
	}
}


/*Send desired velocities in world frame for each axis*/

void AGV::writeVel(const double vx,const double vy, const double vtheta, const double e){


	vel[0] = vx;
	vel[1] = vy;
	vel[2] = vtheta;
	vel[3] = e;

	wm_cmd.w[0] = (1/Rr)*(vx-vy-(La+Lb)*vtheta+e)*F*Z;
	wm_cmd.w[1] = (1/Rr)*(-vx-vy-(La+Lb)*vtheta-e)*F*Z; 
	wm_cmd.w[2] = (1/Rr)*(-vx+vy-(La+Lb)*vtheta+e)*F*Z;
	wm_cmd.w[3] = (1/Rr)*(vx+vy-(La+Lb)*vtheta-e)*F*Z;

	for (int i = 0; i < 4; ++i)
	{
		//Check if motor is activated
		if(m[i].getConnectionState() == 1){
			m[i].writeVel(wm_cmd.w[i]);
		}
		else{

			//Try to reconnect
		}
		
	}


}

/*Blocking function to write an interpolated speed to driver */
/*This function will send send speed in timesteps  of 10ms with the given increment*/
bool AGV::writeVelSoft(double vel[3],int increment){

	double speed_difference_inc[3];
	double speed_cmd[4];

	int n = 0;
	double actual_speed[4];
	if(getVelEncoder(actual_speed)){

		//Don't interfer with kinematic constraint
		for (int i = 0; i < 3; ++i)
		{
			speed_difference_inc[i] = (vel[i]-actual_speed[i])/increment;
			speed_cmd[i] = actual_speed[i];
		}

		speed_cmd[3] = 0;
		while(n<increment){
			n++;
			//write vel, blocking
			writeVel(speed_cmd);
			for (int i = 0; i < 3; ++i)
			{
				speed_cmd[i]+=speed_difference_inc[i];
			}
			usleep(10000);


		}
		return true;


	}

	return false;

}






/*Blocking function that sets a desired speed to AGV whilst limiting the acceleration
by interpolating the desired speed*/


bool AGV::setVelSoft(double vel[3],int increment){

	double speed_difference_inc[3];
	double speed_cmd[3];
	int n = 0;
	double actual_speed[4];
	if(getVelEncoder(actual_speed)){

		//Don't interfer with kinematic constraint
		for (int i = 0; i < 3; ++i)
		{
			speed_difference_inc[i] = (vel[i]-actual_speed[i])/increment;
			speed_cmd[i] = actual_speed[i];
		}


		while(n<increment){
			n++;
			//Not write but only set
			setVelXYZ(speed_cmd);
			for (int i = 0; i < 3; ++i)
			{
				speed_cmd[i]+=speed_difference_inc[i];
			}
			usleep(10000);


		}
		return true;


	}

	return false;


}








void AGV::setVel(double vel[4]){

	this->vel[0] = vel[0];
	this->vel[1] = vel[1];
	this->vel[2] = vel[2];
	this->vel[3] = vel[3];

}

/*This sets desired velocity in X Y and theta without writing to agv
Write vel must be called in order to */
void AGV::setVelXYZ(double vel[3]){

	this->vel[0] = vel[0];
	this->vel[1] = vel[1];
	this->vel[2] = vel[2];
}



/*Start each motor of the agv*/
/*Bus needs to be opened before calling this function*/

bool AGV::start(){
	int j = 0;
	for (int i = 0; i < 4; ++i)
	{
		if(m[i].getAdress() != -1){
			if (m[i].startDriver() == false)
			{
				j++;		
			}

			else{

				//m[i].writeVel(0);
			}
			
			
		}
	}

	if(j==0){

		printf("Started Successfully AGV\r\n");
		return true;
	}

	
	else{
		printf("One or more drivers not started\r\n");
		return false;
	}


}


/*Stop each motor of the agv*/
/*Bus needs to be opened before calling this function*/


bool AGV::stop(){

	int j=0;

	for (int i = 0; i < 4; ++i)
	{
		if(m[i].getAdress() != -1){
			if(m[i].writeVel(0)==true and m[i].stopDriver() == true ){

			}
			else{
				j++;
			}
		}
		
	}

	if(j == 0){

		printf("Successfully stopped all drivers\r\n");
		return true;
	}
	else{

		printf("One or more drivers not stopped\r\n");
		return false;
	}
	

}


/*Start CAN bus on device*/

bool AGV::openBus(int bitrate){

	/*TODO
    -Add a dummy read or write to check if connexion is working 
    -Check the address of all connected devices*/

    

	char buff[100];
	
	/*Should remove the sudo */
	sprintf(buff,"sudo ip link set can0 type can bitrate %i",bitrate);
	system(buff);
    system("sudo ifconfig can0 up");

    printf("Initialized can at bitrate %i \r\n",bitrate);


    return 0;
}


/*Close CAN bus*/

bool AGV::closeBus(){

	
	system("sudo ifconfig can0 down");

	printf("Shut down can0\n");

	return 0;

}





Motor* AGV::getMotor(uint8_t nb){

	return &m[nb];

}

void AGV::startConstraintController(){

	stop_constraint =0;
	constraint_controller = 1;
	iterm = 0;
	pidThread = std::thread(&AGV::constraintController,this);

}


void AGV::constraintController(){

	double e;
	double dt = 10000; //10ms
	double vel_encoder[4];
	double vel_cmd[4];
	double vel_zero[4]={0};
	//Started constraint controller
	while(stop_constraint ==0){

		if(getVelEncoder(vel_encoder)== true){

			for (int i = 0; i < 4; ++i)
			{
				vel_cmd[i] = vel[i];
			}
			iterm = iterm - 1.5*vel_encoder[3]*0.01;
			//printf("Iterm value : %lf \r\n",iterm);
			vel_cmd[3] = iterm;
			writeVel(vel_cmd);
		}

		else{
			//writeVel(vel_zero);
		}
		
		
		
		usleep(dt);

	}
	
}

void AGV::stopConstraintController(){

	stop_constraint = 1;
	if(pidThread.joinable()){
		pidThread.join();
	}
}



void AGV::getVelEncoderAngular(double vel_angular[4]){

	for (int i = 0; i < 4; ++i)
	{
		vel_angular[i] = vel_angular_sens[i];
	}

}


void AGV::getVelCmd(double vel[3]){

	vel[0] = this->vel[0];
	vel[1] = this->vel[1];
	vel[2] = this->vel[2];


}

/*This function reads velocity info from encoder of all 4 drivers and computes 
actual dx, dy, dyaw via inverse kinematics*/

bool AGV::getVelEncoder(double vel_sens[4]){

	
	for (int i = 0; i < 4; ++i)
	{
		if(m[i].getConnectionState() == true){
			wm_sens.w[i] = m[i].getVel();
			vel_angular_sens[i] = (double) wm_sens.w[i]/F;

		}

		else{
			//Try to reconnect ..
			//m[i].restartDriver();
			return false;
		}
		
	}

	//Let's compute dx,dy,dyaw
	
	 vel_sens[0] = (double)(Rr/4)*(-wm_sens.w[1]+wm_sens.w[0]+wm_sens.w[3]-wm_sens.w[2])/F;
	 vel_sens[1] = (double)(Rr/4)*(-wm_sens.w[1]-wm_sens.w[0]+wm_sens.w[3]+wm_sens.w[2])/F;
	 vel_sens[2] = (double)(Rr/4)*(1/(La+Lb))*(-wm_sens.w[1]-wm_sens.w[0]-wm_sens.w[2]-wm_sens.w[3])/F;
	// //This term corresponds to kinematic constraints
	 vel_sens[3] = (double)(-wm_sens.w[1]+wm_sens.w[0]-wm_sens.w[3]+wm_sens.w[2])/F;

	 return true;
}