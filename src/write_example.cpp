

#include "../include/Motor_Driver_DSPIC33.h"
#include "../include/AGV_Driver.h"

#include <stdio.h>
#include <iostream>
#include <string>
#include <cstdlib>

#include <unistd.h>




int main(int argc, char const *argv[])
{

	AGV * agv;
	double vel_cmd[4] = {0};
	vel_cmd[0] = 0.3;


	
	if(argc == 5){
		agv = new AGV(std::stoi(argv[0]),std::stoi(argv[1]),std::stoi(argv[2]),std::stoi(argv[3]));
	}
	else{

		agv = new AGV(4,5,6,7);
	}

	agv->openBus(500000);
	agv->start();

	agv->writeVel(vel_cmd);

	usleep(2000000);

	delete agv;

	
	return 0;
}