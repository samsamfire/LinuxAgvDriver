

#include "../include/Motor_Driver_DSPIC33.h"
#include "../include/AGV_Driver.h"

#include <stdio.h>
#include <iostream>
#include <string>
#include <cstdlib>
#include <ncurses.h>
#include <cmath>



#include <unistd.h>






int main(int argc, char const *argv[])
{

	

	double cmd_vel[4]={0};
	
	AGV agv(4,5,6,7);
	usleep(10000);

	if(agv.start()){

		cmd_vel[0] = 0.3;
		agv.writeVel(cmd_vel);
		usleep(2000000);
		cmd_vel[0] = -0.3;
		agv.writeVel(cmd_vel);
		usleep(2000000);
		cmd_vel[0] = 0;
		agv.writeVel(cmd_vel);
		agv.stop();

		
	}
	return 0;
}