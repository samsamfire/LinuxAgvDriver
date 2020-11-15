

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

double vel_cmd[3] = {0};




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

			        	cout << endl << "up" << endl;  // not arrow
			            break;
			        case KEY_DOWN:
			            vel_cmd[0] = -0.25;
			        	vel_cmd[1] = 0;
			        	vel_cmd[2] = 0;

			        	cout << endl << "down" << endl;  // not arrow
			            break;
			        case KEY_LEFT:
			            vel_cmd[0] = 0;
			        	vel_cmd[1] = -0.25;
			        	vel_cmd[2] = 0;

			        	cout << endl << "left" << endl;  // not arrow
			            break;
			        case KEY_RIGHT:
			            vel_cmd[0] = 0;
			        	vel_cmd[1] = 0.25;
			        	vel_cmd[2] = 0;

			        	cout << endl << "right" << endl;  // not arrow
			            break;
			        case 48:

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

			        default:

			        {
			        	printf("%i\n",c );
			        	break;
			        }



			}
		}
}






int main(int argc, char const *argv[])
{

	

	
	
	AGV agv(4,5,6,7);
	usleep(10000);
	std::thread keyThread(monitor_keyboard,&agv);

	if(agv.start()){

			while(keyboard_monitor == true){

				agv.writeVelSoft(vel_cmd,100);

			}


		
	}

	keyThread.join();
	
	echo();
	nocbreak();
	endwin();
	return 0;
}