#include "../include/Motor_Driver_DSPIC33.h"
#include "../include/AGV_Driver.h"

#include <stdio.h>
#include <iostream>
#include <string>
#include <cstdlib>
#include <ncurses.h>
#include <cmath>





int main(int argc, char const *argv[])
{
	double vel_cmd[4]={0};
	if(argc == 1 or argc > 2){
		return 1;
	}

	else{
		vel_cmd[0]=std::atof(argv[1]);
		AGV agv(4,5,6,7);
		agv.start();
		agv.writeVel(vel_cmd);
		printf("Sent value\n");
	}
	return 0;
}