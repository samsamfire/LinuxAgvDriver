#ifndef  H_AGV_DRIVER
#define H_AGV_DRIVER

#include "Motor_Driver_DSPIC33.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdint.h>
#include <libsocketcan.h>
#include <can_netlink.h>
#include <atomic>
#include <thread>

#define Rr 0.045 //Rayon de la roue en m
#define Z 1 //Rapport de reduction
#define La 0.225//Demie longueur
#define Lb 0.20 //Demie largeur
/*Facteur d'aggrandissment pour ne pas perdre trop d'informations
Lors de l'envoie des vitesses, il faudra prendre ca en compte dans le micro
Cela suppose que les vitesses sont contenus entre -30m/s et +30m/s(largement le cas)
*/
#define F 1000 

/*Facteur d'aggrandissment pour les vitesses angulaires des roues, meme raison*/
#define H 1000






/*Driver for controlling the hole AGV*/

/*The AGV accepts controls such as x velocity, y velocity, z velocity
As well as x and y and yaw positions*/


struct
{
	double* input;
	double* output;
	double* setpoint;

	double Kp;
	double Ki;
	
}PID;


class AGV
{
public:

	AGV(); //fl fr br bl;
	AGV(int ad);
	AGV(int ad_fl, int ad_fr, int ad_br, int ad_bl);
	

	void setMode();

	void writePos();
	void writeVel(const double vel[4]);
	bool writeVelSoft(double vel[3],int increment);


	bool readVel(void);
	void setVel(double vel[4]);
	void setVelXYZ(double vel[3]);

	bool getVelEncoder(double vel[4]);
	void getVelCmd(double vel[3]);

	bool start(); //True if all motors started successfully.
	bool stop();  //Trure if all motors stopped successfully.

	uint8_t* getState();
	Motor* getMotor(uint8_t nb);


	bool openBus(int bitrate);
	bool closeBus();


	void startConstraintController();
	void constraintController();
	void stopConstraintController();

	bool setVelSoft(double vel[3],int increment);
	void getVelEncoderAngular(double vel_angular[4]);







private:
	//Declare the 4 motor drivers

	Motor m[4];

	double vel_angular_sens[4];

	double vel[4],pos[3];

	double vel_sens[4],pos_sens[3];

	//CAN bus info
	bool constraint_controller;
	bool stop_constraint;

	//PID constraintPID;
	double iterm;

	std::thread pidThread;





	
};




























#endif