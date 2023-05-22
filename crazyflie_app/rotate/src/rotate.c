/*-----------------------------------------------------------------------------
 Copyright (C) 2021-2022 University of Bologna, Italy. 
 All rights reserved.                                                           
                                                                               
Thesis based on autonomous driving of drones able to map an unknown
room autonomously through the sensors of the multiranger deck of the
crazyflie 2.0 drone;
rotation mode: the drone performs a 360 ° rotation on itself and in the
meantime acquires eight distance measurements which it will then use
to determine the maximum distance; at this point it moves by a
predetermined distance in that direction.
                                                                               
 File:    rotate.c         
 Author:  Lorenzo Lamberti      <lorenzo.lamberti@unibo.it>                                             
 		   Davide Graziani      <davide.graziani4@studio.unibo.it>
 Date:    08.04.2022                                                           
-------------------------------------------------------------------------------*/

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "app.h"
#include "FreeRTOS.h"
#include "system.h"
#include "task.h"
#include "debug.h"
#include "stabilizer_types.h"
#include "estimator_kalman.h"
#include "commander.h"
#include "log.h"
#include "param.h"
#include <math.h>
#include "config_main.h"


/* --------------- GUI PARAMETERS --------------- */
// Global variables for the parameters
float forward_vel = FORWARD_VELOCITY;
float flying_height = TARGET_H;
float straight_distance = TARGET_D;

// My parameters for enabling/disabling some parts ofss code. 1=Active, 0=Non active
uint8_t debug = 1; 		// activate debug prints

// START / STOP mission parameter
uint8_t fly = 0; 		// Takeoff/landing command (GUI parameter)
uint8_t landed = 0; 	// Flag for indicating whether the drone landed

/* --------------- GLOBAL VARIABLES --------------- */

#define ANGULAR_ROTATION 45

static setpoint_t fly_setpoint;
int16_t valDistance[8], valFront;
uint8_t stateFront;
logVarId_t  idFrontVal, idFrontState, idX, idY, idYaw;
float posXrotate, posYrotate, posYawrotate;

/* --------------- FUNCTION DEFINITION --------------- */ 
void takeoff(float height);
void goStraight();
int rotate();
void land(void);
static void create_setpoint(setpoint_t* setpoint, float x_vel, float y_vel, float z_pos, float yaw_att);
void headToPosition(float x, float y, float z, float yaw);
void headToSetpoint (float x, float y, float z, float yaw);

/* --------------- FUNCTIONS --------------- */ 
// Fly forward functions
static void create_setpoint(setpoint_t* setpoint, float x_vel, float y_vel, float z_pos, float yaw_att)
{    
    memset(setpoint, 0, sizeof(setpoint_t));
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.z = modeAbs;
    setpoint->mode.yaw = modeAbs;
    setpoint->velocity.x = x_vel;
    setpoint->velocity.y = y_vel;
    setpoint->position.z = z_pos;
    setpoint->attitude.yaw = yaw_att;
    setpoint->velocity_body = true;
}

void headToPosition(float x, float y, float z, float yaw)
{
	setpoint_t setpoint;
	memset(&setpoint, 0, sizeof(setpoint_t));

	setpoint.mode.x = modeAbs;
	setpoint.mode.y = modeAbs;
	setpoint.mode.z = modeAbs;
	setpoint.mode.yaw = modeAbs;

	setpoint.position.x = x;
	setpoint.position.y = y;
	setpoint.position.z = z;
	setpoint.attitude.yaw = yaw;
	commanderSetSetpoint(&setpoint, 3);
}

void headToSetpoint (float x, float y, float z, float yaw)
{
    create_setpoint (&fly_setpoint, x, y, z, yaw);
    commanderSetSetpoint (&fly_setpoint, 3);
}

// EXPLORATION FUNCTIONS
void takeoff(float height)
{
	point_t pos;
	memset(&pos, 0, sizeof(pos));
	estimatorKalmanGetEstimatedPos(&pos);

	// first step: taking off gradually, from a starting height of 0.2 to the desired height
	int endheight = (int)(100*(height-0.2f));
	for(int i=0; i<endheight; i++)
	{
		headToPosition(pos.x, pos.y, 0.2f + (float)i / 100.0f, 0);
		vTaskDelay(50);
	}
	// keep constant height
	for(int i=0; i<100; i++)
	{
		headToPosition(pos.x, pos.y, height, 0);
		vTaskDelay(50);
	}
}

void goStraight()
{
	posYawrotate = logGetFloat(idYaw);
    float posXstart = logGetFloat(idX);
    float posYstart = logGetFloat(idY);
	float posdiff = 0;
	
    do
    {
		headToSetpoint (forward_vel, 0.0f, flying_height, posYawrotate);

        posXrotate = logGetFloat(idX);
        posYrotate = logGetFloat(idY);
		posdiff = abs((posXstart - posXrotate)*1000) + abs((posYstart - posYrotate)*1000);

    } while (posdiff <= straight_distance);

	/* --------------- Correction to neglect the braking distance  --------------- */
    posXrotate = logGetFloat(idX);
    posYrotate = logGetFloat(idY);
	for (int i = 0; i < 50; i++)
	{
		headToPosition (posXrotate, posYrotate, flying_height, posYawrotate);
		vTaskDelay(50);
	}
}

int rotate()
{
    int MaxValue;
	float posYawbegin = logGetFloat(idYaw);
	posXrotate = logGetFloat(idX);
	posYrotate = logGetFloat(idY);

	/* --------------- Distance acquisition  --------------- */
	for (int k = 0; k < 8; k++)
	{
		valDistance[k] = 0;

		if (fly == 0)
			return 8;
		posYawrotate = logGetFloat(idYaw);

		/* --------------- Static before measure  --------------- */
		for (int i = 0; i < 10; i++)
		{
			headToPosition (posXrotate, posYrotate, flying_height, posYawrotate);
			vTaskDelay(50);
		}
		
		/* --------------- Filter filling  --------------- */
		for (int i = 0; i < 5; i++)
		{
			headToPosition (posXrotate, posYrotate, flying_height, posYawrotate);
			stateFront = logGetInt(idFrontState);
			valFront = logGetInt(idFrontVal);
        	if (k != 4)
        	{
    			while ((stateFront != 0) && (stateFront != 2))
	    		{
		    		headToPosition (posXrotate, posYrotate, flying_height, posYawrotate);
			    	stateFront = logGetInt(idFrontState);
    				valFront = logGetInt(idFrontVal);
		    	}
	    		if (stateFront == 2)
					valFront = 4000;
        	}
			else
				valFront = 0;
			valDistance[k] = valDistance[k] + valFront;
		}

		/* --------------- Gradual rotation  --------------- */
		for (int i = 0; i < 10; i++)
		{
			headToPosition (posXrotate, posYrotate, flying_height, posYawrotate + (ANGULAR_ROTATION * i)*0.1f);
			vTaskDelay(50);
		}

		/* --------------- Angular correction  --------------- */
		for (int i = 0; i < 10; i++)
		{
			headToPosition (posXrotate, posYrotate, flying_height, posYawrotate + ANGULAR_ROTATION);
			vTaskDelay(50);
		}
	}

	/* --------------- Returning to starting yaw value  --------------- */
	for (int i = 0; i < 10; i++)
	{
		headToPosition (posXrotate, posYrotate, flying_height, posYawbegin);
		vTaskDelay(50);
	}

	/* --------------- Checking max distance  --------------- */
	MaxValue = 0;
	for (int i = 1; i < 8; i++)
	{
		if (valDistance[i] > valDistance[MaxValue])
			MaxValue = i;
	}

	posYawrotate = logGetFloat (idYaw);

	/* --------------- Gradual rotation  --------------- */
	for (int i = 0; i < (10 * MaxValue); i++)
	{
		headToPosition (posXrotate, posYrotate, flying_height, posYawbegin + (ANGULAR_ROTATION * i)*0.1f);
		vTaskDelay(50);
	}

	/* --------------- Angular correction  --------------- */
	for (int i = 0; i < 20; i++)
	{
		headToPosition (posXrotate, posYrotate, flying_height, posYawbegin + (ANGULAR_ROTATION * MaxValue));
		vTaskDelay(50);
	}
	
    return MaxValue;
}

void land(void)
{
	float posX = logGetFloat(idX);
	float posY = logGetFloat(idY);
	float posYaw = logGetFloat(idYaw);

	for(int i=(int)100*flying_height; i>5; i--)
	{
		headToPosition(posX, posY, (float)i / 100.0f, posYaw);
		vTaskDelay(20);
	}
	vTaskDelay(200);
}

void appMain()
{
	DEBUG_PRINT("Dronet v2 started! \n");
	systemWaitStart();
	vTaskDelay(1000);
	/* ------------------------- NOT FLYING ------------------------- */

	while(!fly)
	{
		if (debug==1) DEBUG_PRINT("Waiting start \n");			
		vTaskDelay(100);
	}

	/* ------------------------- TAKING OFF ------------------------- */

	// reset the estimator before taking off	
	estimatorKalmanInit();
	uint8_t isMax = 9;
	// id acquisition
    idFrontVal = logGetVarId("mRange", "ValF");
    idFrontState = logGetVarId("mRange", "StatF");
    idX = logGetVarId("stateEstimate", "x");
    idY = logGetVarId("stateEstimate", "y");
    idYaw = logGetVarId("stateEstimate", "yaw");
	// TAKE OFF
	takeoff(flying_height);	

	/* ------------------------ Flight Loop ------------------------ */

	while(1) {
		vTaskDelay(5);
		if (fly==0 && landed==0)//land
		{
			land();
			landed = 1;
		}	
		if (fly==1 && landed==1) //start flying again
		{
			estimatorKalmanInit();  
			takeoff(flying_height);				
			landed = 0;
		}			
		if (debug==1) DEBUG_PRINT("flying\n");			

		// Give setpoint to the controller
		if (fly==1)
		{
            isMax = rotate();

			if (isMax != 8)
            	goStraight();
		}
		
		vTaskDelay(30);
	}
}


/* --- TIP for Logging or parameters --- */
// The variable name: PARAM_ADD(TYPE, NAME, ADDRESS)
// both for logging (LOG_GROUP_START) or for parameters (PARAM_GROUP_START) 
// should never exceed 9 CHARACTERS, otherwise the firmware won't start correctly

/* --- PARAMETERS --- */ 
PARAM_GROUP_START(START_STOP)
	PARAM_ADD(PARAM_UINT8, fly, &fly)
PARAM_GROUP_STOP(DRONET_PARAM)

// Activate - deactivate functionalities: 0=Non-active, 1=active
PARAM_GROUP_START(FUNCTIONALITIES)
	PARAM_ADD(PARAM_UINT8, debug, &debug) // debug prints
PARAM_GROUP_STOP(DRONET_SETTINGS)

// Filters' parameters
PARAM_GROUP_START(DRONET_PARAMS)
	PARAM_ADD(PARAM_FLOAT, velocity, &forward_vel)
	PARAM_ADD(PARAM_FLOAT, height, &flying_height)
	PARAM_ADD(PARAM_FLOAT, distance, &straight_distance)
PARAM_GROUP_STOP(DRONET_SETTINGS)

LOG_GROUP_START(dist)
LOG_ADD(LOG_INT16, Val0, &valDistance[0])
LOG_ADD(LOG_INT16, Val1, &valDistance[1])
LOG_ADD(LOG_INT16, Val2, &valDistance[2])
LOG_ADD(LOG_INT16, Val3, &valDistance[3])
LOG_ADD(LOG_INT16, Val4, &valDistance[4])
LOG_ADD(LOG_INT16, Val5, &valDistance[5])
LOG_ADD(LOG_INT16, Val6, &valDistance[6])
LOG_ADD(LOG_INT16, Val7, &valDistance[7])
LOG_GROUP_STOP(dist)