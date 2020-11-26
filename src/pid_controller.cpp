

#include "../include/Motor_Driver_DSPIC33.h"
#include "../include/AGV_Driver.h"
#include "../include/MPU6050.h"

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


double pos_cmd[3] = {0};

MPU6050 device(0x68);



void monitor_keyboard(AGV* agv){
	initscr();
	cbreak();
	noecho();
	int c = 0;

	while(keyboard_monitor == true){

			switch((c=getch())) {
			        case KEY_UP:

			        	pos_cmd[0] += 0.3;

			        	cout << endl << "up" << endl;  // not arrow
			            break;
			        case KEY_DOWN:

			        	pos_cmd[0] += -0.3;

			        	cout << endl << "down" << endl;  // not arrow
			            break;
			        case KEY_LEFT:

			        	pos_cmd[1] += -0.3;

			        	cout << endl << "left" << endl;  // not arrow
			            break;
			        case KEY_RIGHT:

			        	pos_cmd[1] += 0.3;

			        	cout << endl << "right" << endl;  // not arrow
			            break;
			        case 48:

			        	pos_cmd[0] = 0;
			        	pos_cmd[1] = 0;
			        	pos_cmd[2] = 0;

			        	cout << endl << "0" << endl;  // not arrow
			        	break;

			        case SPACE_BAR:
			        	echo();
			        	nocbreak();
			        	endwin();
			        	agv->stop();
			        	printf("Stopped from monitor \r\n");
			        	keyboard_monitor = false;
			        	state_controller_state =false;
			        	break;

			        case 111:

			       		pos_cmd[2] += 0.3;
			       		break;

			       	case 105:

			       		pos_cmd[2] += -0.3;
			       		break;

			        default:
			        {
			        	printf("%i\n",c );
			        	break;
			        }



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
	double vel_ctrl[3] = {0};
	double vel_cmd[3] = {0};
	double pos_ctrl[4];
	double prev_pos_ctrl[4];
	double vel_inverse_m[4];
	double vel_encoder[4];

	double rate_limit = 0.03;

	float yaw,pitch,roll;

	double pos_agv_frame[3];
	double pos_world_frame[3];
	double error = 0;
	double usleep_time = 10000;
	double dt;
	double iterm = 0;
	double vxr =0 ,vyr = 0,vthetar = 0;
	double vxw = 0,vyw = 0,vthetaw = 0;
	double pxw = 0,pyw = 0,pthetaw = 0;
	
	float theta = 0;

	double itermx =0, itermy =0, itermtheta = 0, itermdtheta = 0;
	double dtermx=0, dtermy =0, dtermtheta = 0 , dtermdtheta = 0;
	double errorx = 0, errory = 0, errortheta = 0, errordtheta = 0;
	double prev_errorx = 0, prev_errory = 0, prev_errortheta =0, prev_errordtheta = 0;

	double Kpx =10, Kdx = 0.01, Kix = 0.01;
	double Kpy =10, Kdy = 0.01, Kiy = 0.01;
	double Kpz =5, Kdz = 0.01, Kiz = 0.01;
	double Kpdtheta = 2, Kddtheta = 0.001, Kidtheta = 0.01;

	double antiwindup = 0.3;

	dt = usleep_time/1000000;
	while(state_controller_state == true){

		agv->getVelCmd(vel_cmd);
		
		

		//vel_inverse_m contains velocities in x y and theta of robot frame
		agv->getVelEncoder(vel_inverse_m);
		agv->getVelEncoderAngular(vel_encoder);

		vxr = (double) vel_inverse_m[0];
		vyr = (double) vel_inverse_m[1];
		vthetar = (double) vel_inverse_m[2];

		

		//Inverse rotation matrix
		vxw =(double) cos(theta)*vxr + sin(theta)*vyr;
		vyw =(double) -sin(theta)*vxr + cos(theta)*vyr;


		//Use yaw from mpu 
		//vthetaw = (double) vthetar;
		device.getGyro(&roll,&pitch,&yaw);
		vthetaw = -yaw*3.14159/180;

		//Positions in world frame :
		pxw += vxw*dt;
		pyw += vyw*dt;
		//theta +=vthetar*dt;

		//Use yaw from mpu instead of inverse kinematics , value is given in degrees
		device.getAngle(2, &theta);
		
		theta = (float) -theta*3.14159/180;


		//Kinematic constraints
		error += vel_inverse_m[3]*0.01; 

		iterm = -0.8*error;

		//If error is lower than the tolerated precision then error = 0


		errorx = (double) pos_cmd[0]-pxw;

		
		errory = (double) pos_cmd[1]-pyw;

		

		errortheta = (double) pos_cmd[2]-theta;

		errordtheta = (double) 0 - vthetaw;


		

		dtermx = (double) Kdx*(errorx-prev_errorx)/dt;
		dtermy = (double) Kdy*(errory-prev_errory)/dt;
		//dtermtheta = (double) Kdz*(errortheta-prev_errortheta)/dt;
		dtermdtheta = (double)  Kddtheta*(errordtheta - prev_errordtheta)/dt;

		itermx += (double) Kix*errorx*dt;
		itermy += (double) Kiy*errory*dt;
		//itermtheta += (double) Kiz*errortheta*dt;
		itermdtheta += (double) Kidtheta*errordtheta*dt;

		itermx = limit(itermx,0.25,-0.25);
		itermy = limit(itermy,0.25,-0.25);
		//itermtheta = limit(itermtheta,0.6,-0.6);
		itermdtheta = limit(dtermdtheta,0.25,-0.25);


		pos_ctrl[0] =(double) Kpx*(pos_cmd[0]-pxw) + itermx + dtermx;
		pos_ctrl[1] = (double) Kpy*(pos_cmd[1]-pyw) + itermy + dtermy;
		//pos_ctrl[2] = (double) Kpz*(pos_cmd[2] - theta) + itermtheta;

		pos_ctrl[0] = limit(pos_ctrl[0],0.3,-0.3);
		pos_ctrl[1] = limit(pos_ctrl[1],0.3,-0.3);
		//pos_ctrl[2] = limit(pos_ctrl[2],0.6,-0.6);
		//pos_ctrl[3] = limit(iterm,0.3,-0.3);

		//pos_ctrl[0] = 0;
		//pos_ctrl[1] = 0;
		//pos_ctrl[2] = 0;
		pos_ctrl[3] = 0;

		

		//Rate limiting 

		for (int i = 0; i < 4; ++i)
		{
			if (abs(prev_pos_ctrl[i]-pos_ctrl[i])>rate_limit)
			{
				if(pos_ctrl[i]>prev_pos_ctrl[i]){
					pos_ctrl[i] = prev_pos_ctrl[i] + rate_limit;
					printf("pos_ctrl[0] %lf, prev_pos_ctrl[0] %lf \r\n", pos_ctrl[0], prev_pos_ctrl[0]);
				}
				else if(pos_ctrl[i]<prev_pos_ctrl[i]){
					pos_ctrl[i] = prev_pos_ctrl[i] - rate_limit;
					printf("pos_ctrl[0] %lf, prev_pos_ctrl[0] %lf \r\n", pos_ctrl[0], prev_pos_ctrl[0]);
				}

				
			}
		}

		pos_ctrl[2] = (double) Kpdtheta*errordtheta + itermdtheta + dtermdtheta;
		//printf("errordtheta %lf \r\n",pos_ctrl[2] );
		pos_ctrl[2] = limit(pos_ctrl[2],0.6,-0.6);



		// printf("Vel vx: %lf, Vel vy: %lf, Vel theta %lf:, Vel constraint: %lf \r\n",vel_inverse_m[0],vel_inverse_m[1],vel_inverse_m[2],vel_inverse_m[3]);
		// printf("Vel cmd x: %lf, Vel cmd y: %lf, Vel cmd theta: %lf",pos_ctrl[0],pos_ctrl[1],pos_ctrl[2]);
		// //printf("Vel constraint measured : %lf, Vel constraint iterm : %lf\r\n",vel_inverse_m[3],iterm);
		// printf("Pos x: %lf, Pos y: %lf, Pos theta: %lf, constraint: %lf\r\n", pxw,pyw,theta,iterm);
		// printf("Pos cmd x: %lf, Pos cmd y: %lf, Pos cmd theta: %lf \r\n", pos_cmd[0],pos_cmd[1],pos_cmd[2]);

		// printf("Vel encoder 1: %lf, Vel encoder 2: %lf, Vel encoder 3: %lf, Vel encoder 4: %lf \r\n", vel_encoder[0],vel_encoder[1],vel_encoder[2],vel_encoder[3]);
		

		if(abs(errorx)< 0.01){
			pos_ctrl[0] = 0;
			itermx = 0;
		}

		if(abs(errory)< 0.01){
			pos_ctrl[1] = 0;
			itermy =0;
		}

		// if(abs(errortheta)< 0.01){
		// 	pos_ctrl[2] = 0;
		// 	itermtheta = 0;
		// }

		 if(abs(errordtheta)< 0.05){
		 	pos_ctrl[2] = 0;
		 	itermtheta = 0;
		 }

		agv->writeVel(pos_ctrl);

		usleep(usleep_time);

		for (int i = 0; i < 4; ++i)
		{
			prev_pos_ctrl[i] = pos_ctrl[i];
		}

		prev_errorx = errorx;
		prev_errory = errory;
		prev_errortheta =errortheta;
		prev_errordtheta = errordtheta;


	}

}




int main(int argc, char const *argv[])
{

	float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values

		
	double iterm = 0;

	double vel_start[4]={0};
	double vel_inverse_m_cmd[4]= {0};
	double vel_inverse_m[4] = {0};
	

	int count = 0;
	


	
	AGV agv(4,5,6,7);
	usleep(1000000);
	std::thread keyThread(monitor_keyboard,&agv);
	std::thread stateThread(state_controller,&agv);

	

	if(agv.start()){

			agv.writeVel(vel_start);
			while(state_controller_state){

				agv.setVelXYZ(pos_cmd);

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