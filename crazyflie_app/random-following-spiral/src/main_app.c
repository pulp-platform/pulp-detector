/*-----------------------------------------------------------------------------
 Copyright (C) 2023 University of Bologna, Italy.
 All rights reserved.

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 See LICENSE.apache.md in the top directory for details.
 You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 File:   	main_app.c
 Authors:
            Lorenzo Lamberti 	<lorenzo.lamberti@unibo.it>
            Luca Bompani  		<luca.bompani5@unibo.it>
            Manuele Rusci 		<manuele.rusci@kuleuven.be>
            Daniele Palossi 	<dpalossi@ethz.ch> <daniele.palossi@supsi.ch>
 Date:   	01.04.2023
-------------------------------------------------------------------------------*/

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

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
#include <stdlib.h>

/* --------------- GUI PARAMETERS --------------- */

// START / STOP mission parameter
uint8_t fly = 0; 		// Takeoff/landing command (GUI parameter)
uint8_t policy = 0; 		//
uint8_t invert_maze_after_n_laps = INVERT_MAZE_AFTER_N_LAPS;
// Flight
float forward_vel 			= FORWARD_VELOCITY;	// [m/s]
float flying_height 		= TARGET_H;			// [m]
float max_side_speed 		= MAX_SIDE_SPEED;	// [m/s]

// Manouver: Spin -- parameters
float spin_time 			= SPIN_TIME; 			// [ms]
float spin_yawrate 			= SPIN_YAW_RATE; 		// [deg/s]
float spin_angle 			= SPIN_ANGLE; 			// [deg]
float max_rand_angle 		= RANDOM_SPIN_ANGLE; 	// [deg]

// Tof parameters
int8_t tof_state_check 		= TOF_STATE_CHECK;			// 0 or 1
int16_t tof_front_dist_th 	= TOF_FRONT_DIST_THRESHOLD; // [mm]
int16_t side_distance 		= SIDE_DISTANCE; 			// [mm]
int16_t side_tolerance		= SIDE_TOLERANCE; 			// [mm]
int8_t clockwise 			= CLOCKWISE;

// My parameters for enabling/disabling some parts ofss code. 1=Active, 0=Non active
uint8_t debug = 9; 	// activate debug prints
uint8_t motors_on = 1; 	// activate motors
uint8_t slow_down_while_loop = 0; // slows down while(1) loop

/* --------------- GLOBAL VARIABLES --------------- */

// Flight
float side_speed = 0.0;
// -- Flags
uint8_t landed = 1; 	// Flag for indicating whether the drone landed
// -- Counters
int8_t front_err_counter 		= 0; 	// front errors on ToF status
int8_t front_distance_counter 	= 0;	// front value < threshold
int8_t side_distance_counter 	= 0;	// side value > threshold
// -- Counters thresholds
int8_t front_counter_thresh		= FRONT_COUNTERS_THRESHOLD;	// triggers stop and spin
int8_t side_counter_thresh 	 	= SIDE_COUNTERS_THRESHOLD;	// triggers stop and spin
int8_t err_counter_thresh 	 	= ERROR_COUNTERS_THRESHOLD;	// triggers stop and spin
// -- ToF
int8_t en_process_tof_flags		= PROCESS_TOF_FLAGS;
int8_t en_indent				= 1;
int16_t valFront, valRight, valLeft;
uint8_t stateFront, stateRight, stateLeft;
logVarId_t  idFrontVal, idFrontState, idRightVal, idRightState, idLeftVal, idLeftState;
// -- State Estimation
logVarId_t idX, idY, idYaw;

/* ---------------     DEFINES    --------------- */


/* ---------------    STRUCTURES    --------------- */
typedef	struct tof_s{
        int16_t val;
        int8_t  state;
    }tof_t;
typedef struct multiranger_s{
    tof_t front;
    tof_t right;
    tof_t left;
    tof_t back;
} multiranger_t;

multiranger_t multiranger;

/* -------------- FUNCTION DEFINITION -------------- */
void takeoff(float height);
void flight_loop();
void rotate();
void land(void);
void velocity_setpoint(setpoint_t* setpoint, float x_vel, float y_vel, float z_pos, float yaw_att);
void headToPosition(float x, float y, float z, float yaw);
void headToVelocity (float x, float y, float z, float yaw);
void debug_prints();

/* ----------------------------------------------------------------------- */
/* ------------------------------ FUNCTIONS ------------------------------ */
/* ----------------------------------------------------------------------- */

/* --------------- Setpoint Utils --------------- */

setpoint_t fly_setpoint;
setpoint_t create_velocity_setpoint(float x_vel, float y_vel, float z_pos, float yaw_rate)
{
    setpoint_t setpoint;
    memset(&setpoint, 0, sizeof(setpoint_t));
    setpoint.mode.x 	= modeVelocity;
    setpoint.mode.y 	= modeVelocity;
    setpoint.mode.z 	= modeAbs;
    setpoint.mode.yaw 	= modeVelocity;
    setpoint.velocity.x 	= x_vel;
    setpoint.velocity.y 	= y_vel;
    setpoint.position.z 	= z_pos;
    setpoint.attitude.yaw 	= yaw_rate;
    setpoint.velocity_body 	= true;
    return setpoint;
}

void headToVelocity(float x_vel, float y_vel, float z_pos, float yaw_rate)
{
    fly_setpoint = create_velocity_setpoint(x_vel, y_vel, z_pos, yaw_rate);
    if (motors_on) commanderSetSetpoint(&fly_setpoint, 3);
}

setpoint_t create_position_setpoint(float x, float y, float z, float yaw)
{
    setpoint_t setpoint;
    memset(&setpoint, 0, sizeof(setpoint_t));
    setpoint.mode.x 	= modeAbs;
    setpoint.mode.y 	= modeAbs;
    setpoint.mode.z 	= modeAbs;
    setpoint.mode.yaw 	= modeAbs;
    setpoint.position.x 	= x;
    setpoint.position.y 	= y;
    setpoint.position.z 	= z;
    setpoint.attitude.yaw 	= yaw;
    return setpoint;
}

void headToPosition(float x, float y, float z, float yaw)
{
    fly_setpoint = create_position_setpoint(x, y, z, yaw);
    if (motors_on) commanderSetSetpoint(&fly_setpoint, 3);
}


/* --------------- Takeoff and Landing --------------- */

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

#define FINAL_LANDING_HEIGHT    0.07f
void land(void)
{
    point_t pos;
    memset(&pos, 0, sizeof(pos));
    estimatorKalmanGetEstimatedPos(&pos);

    float height = pos.z;
    float current_yaw = logGetFloat(logGetVarId("stateEstimate", "yaw"));

    for(int i=(int)100*height; i>100*FINAL_LANDING_HEIGHT; i--) {
        headToPosition(pos.x, pos.y, (float)i / 100.0f, current_yaw);
        vTaskDelay(20);
    }
    vTaskDelay(200);
}

/* --------------- ToF Utilities --------------- */

multiranger_t get_tof_state(multiranger_t multiranger){
    multiranger.front.state = logGetInt(idFrontState);
    multiranger.right.state = logGetInt(idRightState);
    multiranger.left.state = logGetInt(idLeftState);
    // DEBUG_PRINT("\n [get_tof_state]: front.state %d, right.state %d, left.state %d \n", multiranger.front.state, multiranger.right.state, multiranger.left.state);
    return multiranger;
}

multiranger_t get_tof_val(multiranger_t multiranger){
    multiranger.front.val = logGetInt(idFrontVal);
    multiranger.right.val = logGetInt(idRightVal);
    multiranger.left.val = logGetInt(idLeftVal);
    // DEBUG_PRINT("\n [get_tof_measurement]: front.val %d, right.val %d, left.val %d \n", multiranger.front.val, multiranger.right.val, multiranger.left.val);
    return multiranger;
}

tof_t process_tof_flags(tof_t ToF){
    // usually, tof.state == 2 means that the distance is > 4m, but the measurements are very unstable. We use this flag to overwrite the tof measurement
    if(ToF.state==2){
        ToF.val=4000;
    }
    return ToF;
}


multiranger_t get_tof_measurement(multiranger_t multiranger){
    multiranger = get_tof_state(multiranger);
    multiranger = get_tof_val(multiranger);
    if (en_process_tof_flags==1){
        multiranger.front = process_tof_flags(multiranger.front);
        multiranger.front = process_tof_flags(multiranger.front);
        multiranger.front = process_tof_flags(multiranger.front);
    }
    return multiranger;
}


int8_t ToF_status_isvalid(tof_t ToF, int8_t check_ToF_status){
    /**
     * returns 1 if the current measurement is valid, 0 if it is not valid
     * 		- flag=0 or flag=2 are considered valid
     * Variables:
     * 	- check_status: setting check_status to zero always returns 1, so that we consider any measurement as valid
     * 	- ToF: structure of current tof [front, right, left, back, up]
     */

    // if (debug==8) DEBUG_PRINT("state is %d\n", ToF.state);

    // don't check ToF error status -- always returns current measurement as correct
    if (check_ToF_status == 0){
        return 1; // valid
    }

    // check ToF error status: return 1 for valid, return 0 for invalid
       switch(ToF.state){
        case 0  : // 	VL53L1_RANGESTATUS_RANGE_VALID: 		Ranging measurement is valid
            return 1;
        case 1  : // 	VL53L1_RANGESTATUS_SIGMA_FAIL: 			Raised if sigma estimator check is above the internal defined threshold
            return 1;
        case 2  : // 	VL53L1_RANGESTATUS_SIGNAL_FAIL: 		Raised if signal value is below the internal defined threshold
            return 1;
        case 3  : // 	VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED: Target is below minimum detection threshold.
            return 1;
        case 4  : // 	VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL: 	Raised when phase is out of bounds
            return 1;
        case 5  : // 	VL53L1_RANGESTATUS_HARDWARE_FAIL: 		Raised in case of HW or VCSEL failure
            return 0;
        case 6  : // 	VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL: 	The Range is valid but the wraparound check has not been done.
            return 0;
        case 7  : // 	VL53L1_RANGESTATUS_WRAP_TARGET_FAIL:	Wrapped target, not matching phases
            return 1;
        case 8  : // 	VL53L1_RANGESTATUS_PROCESSING_FAIL: 	Internal algorithm underflow or overflow
            return 1;
        case 14 : // 	VL53L1_RANGESTATUS_RANGE_INVALID: 		The reported range is invalid
            return 0;
        default:
            return 1;
    }
}

uint8_t count_statusError(tof_t ToF, int8_t error_counter, int8_t state_check){
    /**
     * Note: setting state_check==1 keeps the counter to 0 no matter what!
     */

    // NO STATUS ERROR: reset error counter
    if (ToF_status_isvalid(ToF,state_check)==1){
        error_counter = 0;
    }
    // STATUS ERROR: increase error counter
    else{
        error_counter++;
    }
    return error_counter;
}

uint8_t count_ToF_obstacle_detection(tof_t ToF, int16_t distance_thresh, int8_t tof_counter, int8_t state_check){

    // if (debug==8) DEBUG_PRINT("state is_valid %d, ToF.val %d, thresh %d, tof_counter %d\n", ToF_status_isvalid(ToF,state_check), ToF.val, distance_thresh, tof_counter);
    if (ToF_status_isvalid(ToF,state_check)==1){
    // if (debug==5) DEBUG_PRINT("[count_ToF_obstacle_detection] ToF.val %d,\t distance_thresh %d,\t tof_counter %d\n", ToF.val, distance_thresh, tof_counter);
        // check if front measurement is < thresh: increase the counter

        if (ToF.val<distance_thresh){
            tof_counter++;
            // DEBUG_PRINT("[count_ToF_obstacle_detection] tof_counter++!\n");
            // DEBUG_PRINT("[count_ToF_obstacle_detection] [after if] ToF.val %d,\t distance_thresh %d,\t tof_counter %d\n", ToF.val, distance_thresh, tof_counter);
        }
        else{
            // DEBUG_PRINT("[count_ToF_obstacle_detection] tof_counter=0!\n");
            tof_counter=0;
        }
    }
    return tof_counter;
}

uint8_t count_ToF_free_path(tof_t ToF, int16_t distance_thresh, int8_t tof_counter, int8_t state_check){
    // if (debug==5) DEBUG_PRINT("[count_ToF_free_path] ToF.val %d,\t distance_thresh %d,\t tof_counter %d\n", ToF.val, distance_thresh, tof_counter);

    if (ToF_status_isvalid(ToF,state_check)==1){
        // DEBUG_PRINT("[count_ToF_free_path] valid!\n");
        // check if front measurement is < thresh: increase the counter

        if (ToF.val>distance_thresh){
            // DEBUG_PRINT("[count_ToF_free_path] tof_counter++!\n");
            tof_counter++;
        }
        else{
            // DEBUG_PRINT("[count_ToF_free_path] tof_counter=0!\n");
            tof_counter=0;
        }
    }
    return tof_counter;
}

/* --------------- Other Manouvers --------------- */

void spin_in_place_t_cost(float angle, float time){
    /*
    angle [deg]: given the current orientation, spin by "angle" degrees in place;
    time   [ms]: how much time to perform the entire manuever --> impacts the spinning speed;
    */

    float current_yaw;					// fetch current yaw self estimation
    float t_steps = 1; 					// [ms] time steps
    float n_steps = (time/(t_steps)); 	// number of  steps
    float r_steps = (angle/n_steps); 	// angle steps
    float new_yaw;						// new yaw given to the controller. This parameter is updated by the for loop

    // access self estimation
    point_t pos;
    memset(&pos, 0, sizeof(pos));
    estimatorKalmanGetEstimatedPos(&pos);
    current_yaw = logGetFloat(logGetVarId("stateEstimate", "yaw"));
    if (debug==2) DEBUG_PRINT("\n\n[spin_in_place_t_cost]\n current_yaw %f, t_steps %f, n_steps %f, r_steps %f\n\n", current_yaw, t_steps, n_steps, r_steps);

    // perform manuever
    for (int i = 0; i <= n_steps; i++) {
        new_yaw = (i*r_steps) + current_yaw;
        // if (debug==3) DEBUG_PRINT("%f\n",(double)new_yaw);
        headToPosition(pos.x, pos.y, pos.z, new_yaw);
        vTaskDelay(M2T(t_steps));
    }
}

void spin_in_place_yawrate_cost(float angle, float yaw_rate){
    /*
    angle 	 [deg]  : given the current orientation, spin by "angle" degrees in place;
    yaw_rate [deg/s]: constant yaw rate for rotation --> impacts the spinning time;
    */
    float time = abs((angle/yaw_rate) * 1000); // [ms]
    if (debug==2) DEBUG_PRINT("\n\n [spin_in_place_yawrate_cost]\n angle %f, yaw_rate %f, time %f\n\n", angle, yaw_rate, time);
    spin_in_place_t_cost(angle, time);
}

void spin_right(float yaw_rate){
    float angle = -90.0;
    spin_in_place_yawrate_cost(angle, yaw_rate);
}

void spin_left(float yaw_rate){
    float angle = +90.0;
    spin_in_place_yawrate_cost(angle, yaw_rate);
}


void spin_in_place_random(float starting_random_angle, float yaw_rate, float rand_range){
    /**
     * spin to a random angle. The random angle is chosen between starting_random_angle +/- rand_range
     */
    // calculate a random spinning angle
    float random_angle = starting_random_angle - rand_range + (2*rand_range) * (float)rand()/(float)(RAND_MAX);
    // if the random angle is >180°, then spin to the opposite side
    if (random_angle > 180){
        random_angle = -(360 - random_angle);
    }
    if (debug==2) DEBUG_PRINT("\n\n [spin_in_place_random]:\n starting_random_angle %f, yaw_rate %f, rand_range %f, random_angle %f", starting_random_angle, yaw_rate, rand_range, random_angle);
    spin_in_place_yawrate_cost(random_angle, yaw_rate);

}


float keep_safe_side_distance(multiranger_t multiranger, int16_t side_distance, float max_side_speed, int8_t state_check){
    float roll_left  =  max_side_speed;
    float roll_right = -max_side_speed;
    float output_side_speed = 0.0;

    // if we are too close to right walls:
    if (multiranger.right.val< side_distance && ToF_status_isvalid(multiranger.right, state_check) == 1){ // roll to left
        output_side_speed = roll_left;
    }
    // if we are too close to left walls:
    else if (multiranger.left.val< side_distance && ToF_status_isvalid(multiranger.left, state_check) == 1){ // roll to right
        output_side_speed = roll_right;
    }
    else{
        output_side_speed = 0.0; // go straight
    }

    return output_side_speed;
}

float keep_determined_side_distance(tof_t tof, int8_t left, int16_t side_distance, float max_side_speed, int8_t check_status){
    /**
     * left=1 -> keeps constant distance from left wall
     * left=0 -> keeps constant distance from right wall
     */
    // roll speed
    float roll_left  =  max_side_speed;
    float roll_right = -max_side_speed;
    float output_side_speed = 0.0;
    // tolerances
    // float tolerance = 20; // percentage [%]
    // int16_t side_tolerance = (int16_t)(tolerance*side_distance/100); // 10% of side_distance

    // keep constant distance from left wall
    if (left==1){
        // if we are too close to left walls:
        if (tof.val < (side_distance-side_tolerance) && ToF_status_isvalid(tof, check_status) == 1){ // roll to right
            output_side_speed = roll_right;
        }
        // if we are too far to left wall:
        else if (tof.val > (side_distance+side_tolerance) && ToF_status_isvalid(tof, check_status) == 1){ // roll to left
            output_side_speed = roll_left;
        }
        else{
            output_side_speed = 0.0; // go straight
        }
    }

    // keep constant distance from right wall
    if (left==0){
        // if we are too close to right wall:
        if (tof.val < (side_distance-side_tolerance) && ToF_status_isvalid(tof, check_status) == 1){ // roll to left
            output_side_speed = roll_left;
        }
        // if we are too far to right wall:
        else if (tof.val > (side_distance+side_tolerance) && ToF_status_isvalid(tof, check_status) == 1){ // roll to right
            output_side_speed = roll_right;
        }
        else{
            output_side_speed = 0.0; // go straight
        }

    }
    return output_side_speed;
}

/* --------------- Processing --------------- */

float low_pass_filtering(float data_new, float data_old, float alpha)
{
    float output;
    // Low pass filter the forward velocity
    output = (1.0f - alpha) * data_new + alpha * data_old;
    return output;
}


/* --------------- Check on front/side tofs --------------- */

int8_t is_front_obstacle_detected(tof_t tof, int16_t front_dist_th, int8_t tof_state_check)	{

    /* ---------- Count Front state errors and detections ---------- */
    front_distance_counter = count_ToF_obstacle_detection(tof, front_dist_th, front_distance_counter, tof_state_check);
    front_err_counter = count_statusError(tof, front_err_counter, tof_state_check);
    // DEBUG_PRINT("[is_front_obstacle_detected] front_err_counter %d,\t front_distance_counter %d\n", front_err_counter, front_distance_counter);

    /* ---------- we generate spin signal if errors/detectionr > threshold ---------- */

    if (front_distance_counter>front_counter_thresh){
        // DEBUG_PRINT("[is_front_obstacle_detected] counter thresh reached: front_distance_counter = 0\n");
        front_distance_counter = 0;
        return 1;
    }
    else if (front_err_counter > err_counter_thresh){
        // DEBUG_PRINT("[is_front_obstacle_detected] counter thresh reached: front_err_counter = 0;\n");

        front_err_counter = 0;
        return 1;
    }
    else{
        return 0;
    }
}

/** THRESH_SIDE_INDENTATION: this is the threshold distance for wall following.
 *  If the right/left sensors measure a free path of > side_dist+THRESH_SIDE_INDENTATION,
 *  then there is an indentation on the environment and we can turn in that direction
 */
uint8_t is_side_empty(tof_t tof_wall, int16_t side_threshold, int8_t check_status){

    side_distance_counter = count_ToF_free_path(tof_wall, side_threshold, side_distance_counter, check_status);
    if(side_distance_counter>side_counter_thresh){
        if (debug==5) DEBUG_PRINT("Side ToF.val= %d -> has empty path -> spin 90°\n", tof_wall.val);
        side_distance_counter=0;
        return 1;
    }
    else{
        if (debug==5) DEBUG_PRINT("Side ToF.val= %d -> DOESNT have empty path -> go straight\n", tof_wall.val);
        return 0;
    }
}

/* ----------------------------------------------------------------------- */
/* ----------------------------- Flight Loop ----------------------------- */
/* ----------------------------------------------------------------------- */

void random_policy()
{
    /* --------------- Sensor acquisition (left & right) --------------- */
    multiranger = get_tof_measurement(multiranger);
    if (debug==3) DEBUG_PRINT("\nmultiranger: front.val %d,\t right.val   %d,\t left.val   %d \n", multiranger.front.val, multiranger.right.val, multiranger.left.val);
    if (debug==3) DEBUG_PRINT("multiranger: front.state %d,\t right.state %d,\t left.state %d \n", multiranger.front.state, multiranger.right.state, multiranger.left.state);

    /* --------------- Correction for left & right obstacles --------------- */

    side_speed = keep_safe_side_distance(multiranger, side_distance, max_side_speed, tof_state_check);

    /* --------------- Count Front state errors: stop if too many !--------------- */

    front_err_counter = count_statusError(multiranger.front, front_err_counter, tof_state_check);
    front_distance_counter = count_ToF_obstacle_detection(multiranger.front, tof_front_dist_th, front_distance_counter, tof_state_check);
    if (debug==4)  DEBUG_PRINT("multiranger.front.val %d, tof_front_dist_th %d\n",  multiranger.front.val, tof_front_dist_th);
    if (debug==4)  DEBUG_PRINT("front_err_counter %d, front_distance_counter %d\n", front_err_counter, front_distance_counter);

    /* --------------- Set setpoint --------------- */

    if (front_err_counter > err_counter_thresh || front_distance_counter>front_counter_thresh){
        // stop & spin
        if (debug==9) DEBUG_PRINT("SPIN\n");
        spin_in_place_random(spin_angle, spin_yawrate, max_rand_angle);
        front_err_counter = 0;
        front_distance_counter = 0;
    }
    else{
        // go forward
        headToVelocity(forward_vel, side_speed, flying_height, 0.0);
    }

    if (debug==1) DEBUG_PRINT("forward_vel: %f \t side_speed: %f\n", forward_vel, side_speed);
    if (debug==9) DEBUG_PRINT("forward_vel: %f \t side_speed: %f \t front.val/state   %d/%d \t coll_counter: %d \t front_err_counter: %d\n", forward_vel, side_speed, multiranger.front.val, multiranger.right.state, front_distance_counter, front_err_counter);

}


tof_t tof_wall;
uint8_t left;
void wall_following_policy()
{
    // set clockwise or couter-clockwise wall-following process
    if (clockwise){
        tof_wall = multiranger.left;
        left = 1;
        spin_angle = -90; // turn right
    }
    else{
        tof_wall = multiranger.right;
        left = 0;
        spin_angle = +90; // turn left
    }

    /* --------------- Sensor acquisition (front, left, right) --------------- */
    multiranger = get_tof_measurement(multiranger);

    /* --------------- Correction for left & right obstacles --------------- */
    side_speed = keep_determined_side_distance(tof_wall, left, side_distance, max_side_speed, tof_state_check);

    /* --------------- Check front obstacle: if yes, then spin --------------- */
    if (is_front_obstacle_detected(multiranger.front, tof_front_dist_th, tof_state_check)){
        // stop & spin
        if (debug==9) DEBUG_PRINT("SPIN\n");
        spin_in_place_yawrate_cost(spin_angle, spin_yawrate);
    }
    else{
        // go forward
        headToVelocity(forward_vel, side_speed, flying_height, 0.0);
    }

    /* --------------- Check side empty: if yes, then spin --------------- */
    if (en_indent){
        int16_t side_indentation_threshold = side_distance + THRESH_SIDE_INDENTATION;
        if(is_side_empty(tof_wall, side_indentation_threshold, tof_state_check)){
            float spin_angle_opposite = - spin_angle;
            spin_in_place_yawrate_cost(spin_angle_opposite, spin_yawrate);
        }
    }
    debug_prints();
}



// function to check whether 'n' is
// a multiple of 4 or not
int8_t isAMultipleOf4(int8_t n){
    // if true, then 'n' is a multiple of 4
    if ((n % 4)==0 && (n!=0))
        return 1;
    // else 'n' is not a multiple of 4
    return 0;
}




tof_t tof_wall;
uint8_t left;
int16_t side_distance_maze;
int16_t front_distance_maze;
int counter_spins = 0;
int full_room_lap = 1;
int increment = 1;

int increment_or_decrement_lap(int full_room_lap, int increment){
    if (increment)
        full_room_lap ++;
    else
        full_room_lap--;
    return full_room_lap;
}

void maze_policy()
{
    // set clockwise or couter-clockwise wall-following process
    if (clockwise){
        tof_wall = multiranger.left;
        left = 1;
        spin_angle = -90; // turn right
    }
    else{
        tof_wall = multiranger.right;
        left = 0;
        spin_angle = +90; // turn left
    }


    /* The room is 6.6 x 5.5 meters
    side distance gets doubled every time

    default:side_distance = 500
    lap1: 	side_distance = 500
    lap2: 	side_distance = 1000
    lap3: 	side_distance = 1500
    lap4: 	side_distance = 2000
    lap5: 	side_distance = 2500
    lap6: 	side_distance = 3000

    at this point, the room is 5m wide, so the drone is in position side_distance/2 ~ 2.5m and starts to spin in place
    To counter this I start decreasing again the side distance to get closer to the walls again */

    if (full_room_lap==1)  	increment=1; // increase !
    if (full_room_lap==invert_maze_after_n_laps)  increment=0; // decrease !

    // every lap completed we increase the distance from the sides
    if(isAMultipleOf4(counter_spins)){
        full_room_lap = increment_or_decrement_lap(full_room_lap, increment);
        counter_spins=0;
    }
    side_distance_maze = side_distance * full_room_lap;
    front_distance_maze = tof_front_dist_th + (side_distance*(full_room_lap-1));
    if (side_distance_maze>4000) side_distance_maze=4000;
    if (front_distance_maze>4000) front_distance_maze=4000;

    /* --------------- Sensor acquisition (front, left, right) --------------- */
    multiranger = get_tof_measurement(multiranger);

    /* --------------- Correction for left & right obstacles --------------- */
    side_speed = keep_determined_side_distance(tof_wall, left, side_distance_maze, max_side_speed, tof_state_check);

    /* --------------- Check front obstacle: if yes, then spin --------------- */
    if (is_front_obstacle_detected(multiranger.front, front_distance_maze, tof_state_check)){
        // stop & spin
        if (debug==9 || debug==10) DEBUG_PRINT("SPIN\n");
        spin_in_place_yawrate_cost(spin_angle, spin_yawrate);
        counter_spins++;
    }
    else{
        // go forward
        headToVelocity(forward_vel, side_speed, flying_height, 0.0);
    }

    /* --------------- Check side empty: if yes, then spin --------------- */
    if (en_indent){
        int16_t side_indentation_threshold = side_distance_maze + THRESH_SIDE_INDENTATION;

        if(is_side_empty(tof_wall, side_indentation_threshold, tof_state_check)){
            DEBUG_PRINT("SPIN OPPOSITE\n");
            float spin_angle_opposite = - spin_angle;
            spin_in_place_yawrate_cost(spin_angle_opposite, spin_yawrate);
            counter_spins--;
            if (counter_spins<0) counter_spins=0; // maze only fix.
        }
    }
    debug_prints();
}

void debug_prints(){
    /* general */
    // print multiranger [front, right, left]: [val, state]
    if (debug==3) DEBUG_PRINT("\nmultiranger: front.val %d,\t right.val   %d,\t left.val   %d \n", multiranger.front.val, multiranger.right.val, multiranger.left.val);
    if (debug==3) DEBUG_PRINT("multiranger: front.state %d,\t right.state %d,\t left.state %d \n", multiranger.front.state, multiranger.right.state, multiranger.left.state);
    // print multiranger [front, right, left]: [val, state] [err_counter,dist_counter]
    if (debug==9) DEBUG_PRINT("multiranger: front.val/state   %d/%d,\t right.val/state   %d/%d,\t left.val/state   %d/%d \n", multiranger.front.val, multiranger.front.state, multiranger.right.val, multiranger.right.state, multiranger.left.val, multiranger.left.state);
    if (debug==9) DEBUG_PRINT("front_err_counter %d,\t front_distance_counter %d,\t side_distance_counter %d\n", front_err_counter, front_distance_counter, side_distance_counter);
    // print multiranger [front]: [val, thresh], [err_counter, dist_counter]
    if (debug==4) DEBUG_PRINT("multiranger.front.val %d, tof_front_dist_th %d\n",  multiranger.front.val, tof_front_dist_th);
    if (debug==4) DEBUG_PRINT("front_err_counter %d, front_distance_counter %d\n", front_err_counter, front_distance_counter);
    /* wall following */
    // print multiranger [front, side]: [velocity, val, target_side_dist] [err_counter, dist_counter]
    if (debug==5) DEBUG_PRINT("forward_vel: %f \t front.val: %d \t side_speed: %f \t side.val: %d \t target_side_distance: %d\n", forward_vel, multiranger.front.val, side_speed, tof_wall.val, side_distance);
    if (debug==5) DEBUG_PRINT("front_err_counter %d,\t front_distance_counter %d,\t side_distance_counter %d\n", front_err_counter, front_distance_counter, side_distance_counter);
    /* MAZE */
    // print multiranger [front, side]: [velocity, val, speed, target_side_dist] counters[ spin, laps]
    if (debug==10) DEBUG_PRINT("[dist] forward_vel: %f \t front.val: %d \t front*(laps-1): %d \t side_speed: %f \t side.val: %d \t side*laps: %d\n", forward_vel, multiranger.front.val, front_distance_maze, side_speed, tof_wall.val, side_distance_maze);
    if (debug==11) DEBUG_PRINT("[count] counter_spins: %d \t full_room_lap: %d \t side_distance_maze: %d \t side.val: %d \n", counter_spins, full_room_lap, side_distance_maze, tof_wall.val);
}

/* ---------------------------------------------------------------------- */
/* -----------------------------    MAIN    ----------------------------- */
/* ---------------------------------------------------------------------- */
void appMain()
{
    DEBUG_PRINT("Starting the system! \n");
    systemWaitStart();
    vTaskDelay(1000);

    /* ------------------------- TAKING OFF ------------------------- */

    // reset the estimator before taking off
    estimatorKalmanInit();
    DEBUG_PRINT("Resetting Kalman Estimator\n");

    // multiranger id
    DEBUG_PRINT("Getting Multiranger ids\n");
    idFrontVal = logGetVarId("mRange", "ValF");
    idFrontState = logGetVarId("mRange", "StatF");
    idRightVal = logGetVarId("mRange", "ValR");
    idRightState = logGetVarId("mRange", "StatR");
    idLeftVal = logGetVarId("mRange", "ValL");
    idLeftState = logGetVarId("mRange", "StatL");

    //position id
    DEBUG_PRINT("Getting state estimator ids\n");
    idX = logGetVarId("stateEstimate", "x");
    idY = logGetVarId("stateEstimate", "y");
    idYaw = logGetVarId("stateEstimate", "yaw");

    /* ------------------------ while(1) Loop ------------------------ */
    DEBUG_PRINT("Begining flight loop\n");
    while(1) {
        vTaskDelay(30);
        if (slow_down_while_loop==1) vTaskDelay(M2T(500));

        // wait
        if (fly==0 && landed==1){
            DEBUG_PRINT("Waiting start \n");
            multiranger = get_tof_measurement(multiranger);
            // if (debug==9) DEBUG_PRINT("multiranger: front.val   %d,\t right.val   %d,\t left.val   %d \n", multiranger.front.val, multiranger.right.val, multiranger.left.val);
            // if (debug==9) DEBUG_PRINT("multiranger: front.state %d,\t right.state %d,\t left.state %d \n\n", multiranger.front.state, multiranger.right.state, multiranger.left.state);
            if (debug==9) DEBUG_PRINT("multiranger: front.val/state   %d/%d,\t right.val/state   %d/%d,\t left.val/state   %d/%d \n", multiranger.front.val, multiranger.front.state, multiranger.right.val, multiranger.right.state, multiranger.left.val, multiranger.left.state);

            vTaskDelay(100);
        }

        //land
        if (fly==0 && landed==0){
            DEBUG_PRINT("Landing \n");
            land();
            landed=1;
        }

        //start flying again
        if (fly==1 && landed==1){
            DEBUG_PRINT("Take off \n");
            estimatorKalmanInit();  // reset the estimator before taking off
            takeoff(flying_height);
            landed=0;
        }

        // Give setpoint to the controller
        if (fly==1)
        {
            if (policy==0) random_policy();
            if (policy==1) wall_following_policy();
            if (policy==2) maze_policy();
        }
    }
}


/* -------------------------------------------------------------------------------- */
/* ------------------------------ Logging/Parameters ------------------------------ */
/* -------------------------------------------------------------------------------- */
/* --- TIP for Logging or parameters --- */
// The variable name: PARAM_ADD(TYPE, NAME, ADDRESS)
// both for logging (LOG_GROUP_START) or for parameters (PARAM_GROUP_START)
// should never exceed 9 CHARACTERS, otherwise the firmware won't start correctly
/* --------------- LOGGING --------------- */
LOG_GROUP_START(VARIABLES_LOG)
    LOG_ADD(LOG_FLOAT, side_vel, &side_speed)  	// side_speed
LOG_GROUP_STOP(VARIABLES_LOG)

/* --------------- PARAMETERS --------------- */
PARAM_GROUP_START(START_STOP)
    PARAM_ADD(PARAM_UINT8, fly, &fly)
PARAM_GROUP_STOP(START_STOP)

PARAM_GROUP_START(FLIGHT)
    PARAM_ADD(PARAM_FLOAT, velocity, &forward_vel)
    PARAM_ADD(PARAM_FLOAT, side_vel, &max_side_speed)
    PARAM_ADD(PARAM_FLOAT, height, &flying_height)
PARAM_GROUP_STOP(FLIGHT)

PARAM_GROUP_START(POLICY)
    PARAM_ADD(PARAM_INT8,  policy, &policy) // for wall-following and maze
    PARAM_ADD(PARAM_INT8,  en_indent, &en_indent) // for wall-following and maze
    PARAM_ADD(PARAM_INT8,  clockwise, &clockwise) // for wall-following and maze
    PARAM_ADD(PARAM_INT8,  inv_laps, &invert_maze_after_n_laps) // for wall-following and maze
PARAM_GROUP_STOP(POLICY)

PARAM_GROUP_START(SPINNING)
    PARAM_ADD(PARAM_FLOAT, spin_ang,  &spin_angle)
    PARAM_ADD(PARAM_FLOAT, spin_time, &spin_time)
    PARAM_ADD(PARAM_FLOAT, spin_yawr, &spin_yawrate)
    PARAM_ADD(PARAM_FLOAT, rnd_angle, &max_rand_angle)
PARAM_GROUP_STOP(SPINNING)

PARAM_GROUP_START(ToF)
    PARAM_ADD(PARAM_INT16, frnt_dist, &tof_front_dist_th)
    PARAM_ADD(PARAM_INT16, side_dist, &side_distance)
    PARAM_ADD(PARAM_INT16, side_tole, &side_tolerance)
    PARAM_ADD(PARAM_INT8,  en_tof_ck, &tof_state_check)
    PARAM_ADD(PARAM_INT8,  prcss_flg, &en_process_tof_flags)
PARAM_GROUP_STOP(ToF)

PARAM_GROUP_START(ToF_TH_COUNT)
PARAM_ADD(PARAM_INT8, frnt_c_th, &front_counter_thresh)
PARAM_ADD(PARAM_INT8, side_c_th, &side_counter_thresh)
PARAM_ADD(PARAM_INT8, err_c_th,  &err_counter_thresh)
PARAM_GROUP_STOP(ToF_TH_COUNT)

PARAM_GROUP_START(DEBUG)
    PARAM_ADD(PARAM_UINT8, debug, &debug) // debug prints
    PARAM_ADD(PARAM_UINT8, motors_on, &motors_on) // enables/disables motors!
    PARAM_ADD(PARAM_UINT8, slow, &slow_down_while_loop) // debug prints
PARAM_GROUP_STOP(DEBUG)