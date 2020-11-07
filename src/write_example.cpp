

#include "../include/Motor_Driver_DSPIC33.h"
#include "../include/AGV_Driver.h"

#include <stdio.h>
#include <iostream>
#include <string>
#include <cstdlib>

#include <unistd.h>




int main(int argc, char const *argv[])
{


	double vel_cmd[4] = {0};
	double *vel_encoder;
	vel_cmd[0] = 0.3;

	int count = 0;


	
	AGV agv(4,5,6,7);

	agv.openBus(500000);
	agv.start();

	agv.writeVel(vel_cmd);

	while(count < 100){

		agv.readVel();
		vel_encoder = agv.getVel();
		printf("This is x[0] speed %f\r\n",vel_encoder[0]);
		usleep(100000);
		count++;

	}
	



	agv.stop();
	agv.closeBus();


	
	return 0;
}