# LinuxAgvDriver
Agv robot driver for linux, written in c++



#Requirements :
Tested on raspberry Pi 3b+ running ubuntu 20.04LTS.
ncurses and libsocketcan need to be installed.


#Install:
Run makefile 


#Usage:

The driver contains several examples:
-teleop_keyboard : which is a simple teleoperation using arrow keys for movement. Space bar stops the program and the AGV, 0 stops movement.
-write_example : for send simple forward command.
-pid_controller : implementation of pid controller for better control of the AGV 


