

#include "../include/Motor_Driver_DSPIC33.h"
#include "../include/AGV_Driver.h"

#include <stdio.h>
#include <iostream>
#include <string>






int main(int argc, char const *argv[])
{

	AGV * agv;
	
	if(argc == 5){
		agv = new AGV(std::stoi(argv[0]),std::stoi(argv[1]),std::stoi(argv[2]),std::stoi(argv[3]));
	}
	else{

		agv = new AGV(4,5,6,7);
	}

	agv->openBus(500000);


	agv->start();


	return 0;
}