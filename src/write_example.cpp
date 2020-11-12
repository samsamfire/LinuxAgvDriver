

#include "../include/Motor_Driver_DSPIC33.h"
#include "../include/AGV_Driver.h"

#include <stdio.h>
#include <iostream>
#include <string>
#include <cstdlib>
#include <ncurses.h>
#include <thread>
#include <cmath>

//#include <conio.h>

#define KEY_UP 65
#define KEY_DOWN 66
#define KEY_LEFT 68
#define KEY_RIGHT 67
#define SPACE_BAR 32


#include <unistd.h>


using namespace std;

std::atomic<bool> keyboard_monitor{true};
std::atomic<bool> state_controller_state{true};

double vel_cmd[3] = {0};

double pos_cmd[3] = {0};



void monitor_keyboard(AGV* agv){
	initscr();
	cbreak();
	noecho();
	int c = 0;

	while(keyboard_monitor == true){

			switch((c=getch())) {
			        case KEY_UP:
			        	vel_cmd[0] = 0.25;
			        	vel_cmd[1] = 0;
			        	vel_cmd[2] = 0;

			        	pos_cmd[0] = 0.3;
			        	pos_cmd[1] = 0;
			        	pos_cmd[2] = 0;

			        	cout << endl << "up" << endl;  // not arrow
			            break;
			        case KEY_DOWN:
			            vel_cmd[0] = -0.25;
			        	vel_cmd[1] = 0;
			        	vel_cmd[2] = 0;

			        	pos_cmd[0] = -0.3;
			        	pos_cmd[1] = 0;
			        	pos_cmd[2] = 0;

			        	cout << endl << "down" << endl;  // not arrow
			            break;
			        case KEY_LEFT:
			            vel_cmd[0] = 0;
			        	vel_cmd[1] = -0.25;
			        	vel_cmd[2] = 0;

			        	pos_cmd[0] = 0;
			        	pos_cmd[1] = -0.3;
			        	pos_cmd[2] = 0;
			        	cout << endl << "left" << endl;  // not arrow
			            break;
			        case KEY_RIGHT:
			            vel_cmd[0] = 0;
			        	vel_cmd[1] = 0.25;
			        	vel_cmd[2] = 0;

			        	pos_cmd[0] = 0;
			        	pos_cmd[1] = 0.3;
			        	pos_cmd[2] = 0;
			        	cout << endl << "right" << endl;  // not arrow
			            break;
			        case 48:

			        	pos_cmd[0] = 0;
			        	pos_cmd[1] = 0;
			        	pos_cmd[2] = 0;

			        	vel_cmd[0] = 0;
			        	vel_cmd[1] = 0;
			        	vel_cmd[2] = 0;
			        	cout << endl << "0" << endl;  // not arrow
			        	break;

			        case SPACE_BAR:
			        	echo();
			        	nocbreak();
			        	endwin();
			        	agv->stop();
			        	printf("Stopped from monitor \r\n");
			        	keyboard_monitor = false;
			        	break;



			}
		}
}

double limit(double to_limit, double limit_max, double limit_min){


	if (to_limit>limit_max)
	{
		to_limit = limit_max;
	}

	else if (to_limit<limit_min)
	{
		to_limit = limit_min;
	}

	return to_limit;
}


void state_controller(AGV* agv){

	double vel_ctrl[4];
	double pos_ctrl[4];
	double vel_inverse_m[4];
	double vel_encoder[4];

	double pos_agv_frame[3];
	double pos_world_frame[3];
	double error = 0;
	double usleep_time = 10000;
	double dt;
	double iterm = 0;
	double vxr =0 ,vyr = 0,vthetar = 0;
	double vxw = 0,vyw = 0,vthetaw = 0;
	double pxw = 0,pyw = 0,pthetaw = 0;
	double theta = 0;

	double itermx =0, itermy =0, itermtheta = 0;
	double errorx = 0, errory = 0, errortheta = 0;

	double Kp =1, Kd =0, Ki = 0.2;

	double antiwindup = 0.3;

	dt = usleep_time/1000000;
	while(state_controller_state == true){

		agv->getVelCmd(vel_ctrl);

		//vel_inverse_m contains velocities in x y and theta of robot frame
		agv->getVelEncoder(vel_inverse_m);
		agv->getVelEncoderAngular(vel_encoder);

		vxr = vel_inverse_m[0];
		vyr = vel_inverse_m[1];
		vthetar = vel_inverse_m[2];

		

		//Inverse rotation matrix
		vxw =(double) cos(theta)*vxr + sin(theta)*vyr;
		vyw =(double) -sin(theta)*vxr + cos(theta)*vyr;
		vthetaw = (double) vthetar;

		//Positions in world frame :
		pxw += vxw*dt;
		pyw += vyw*dt;
		theta +=vthetar*dt;

		error += vel_inverse_m[3]*0.01; 

		iterm = -0.3*error;


		if(iterm >0.3){
			iterm = antiwindup;
		}

		else if(iterm < -0.3){

			iterm = -antiwindup;
		}

		//vel_ctrl[3] = iterm;
		vel_ctrl[3] = 0;

		errorx = (double) pos_cmd[0]-pxw;
		errory = (double) pos_cmd[1]-pyw;
		errortheta = (double) pos_cmd[2]-theta;

		itermx += (double) Ki*errorx*dt;
		itermy += (double) Ki*errory*dt;
		itermtheta += (double) Ki*errortheta*dt;

		itermx = limit(itermx,0.3,-0.3);
		itermy = limit(itermy,0.3,-0.3);
		itermtheta = limit(itermtheta,0.3,-0.3);


		pos_ctrl[0] =(double) Kp*(pos_cmd[0]-pxw) + itermx;
		//pos_ctrl[1] = (double) Kp*(pos_cmd[1]-pyw) + itermy;
		//pos_ctrl[2] = (double) Kp*(pos_cmd[2] - theta) + itermtheta;

		pos_ctrl[0] = limit(pos_ctrl[0],0.3,-0.3);
		//pos_ctrl[1] = limit(pos_ctrl[1],0.3,-0.3);
		//pos_ctrl[2] = limit(pos_ctrl[2],0.3,-0.3);

		pos_ctrl[0] = 0;
		pos_ctrl[1] = 0;
		pos_ctrl[2] = 0;

		

		//limit ouputs



		printf("Vel vx: %lf, Vel vy: %lf, Vel theta %lf:, Vel constraint: %lf \r\n",vel_inverse_m[0],vel_inverse_m[1],vel_inverse_m[2],vel_inverse_m[3]);
		printf("Vel cmd x: %lf, Vel cmd y: %lf, Vel cmd theta: %lf",pos_ctrl[0],pos_ctrl[1],pos_ctrl[2]);
		//printf("Vel constraint measured : %lf, Vel constraint iterm : %lf\r\n",vel_inverse_m[3],iterm);
		printf("Pos x: %lf, Pos y: %lf, Pos theta: %lf\r\n", pxw,pyw,theta);

		printf("Vel encoder 1: %lf, Vel encoder 2: %lf, Vel encoder 3: %lf, Vel encoder 4: %lf \r\n", vel_encoder[0],vel_encoder[1],vel_encoder[2],vel_encoder[3]);
		agv->writeVel(pos_ctrl);

		usleep(usleep_time);


	}

}




int main(int argc, char const *argv[])
{

	

		
	double iterm = 0;

	double vel_start[4]={0};
	double vel_inverse_m_cmd[4]= {0};
	double vel_inverse_m[4] = {0};
	
	vel_cmd[0] = 0;

	int count = 0;
	


	
	AGV agv(4,5,6,7);
	usleep(10000);
	std::thread keyThread(monitor_keyboard,&agv);
	std::thread stateThread(state_controller,&agv);

	if(agv.start()){

			agv.writeVel(vel_start);
			while(1){

				agv.setVelSoft(pos_cmd,100);


			}


		
	}

	state_controller_state = false;

	keyThread.join();
	stateThread.join();
	
	echo();
	nocbreak();
	endwin();
	return 0;
}