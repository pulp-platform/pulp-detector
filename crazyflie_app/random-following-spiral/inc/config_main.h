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

 File:   	SSD_tin_can_bottle.py
 Authors:
	  		Lorenzo Lamberti 	<lorenzo.lamberti@unibo.it>
         	Luca Bompani  		<luca.bompani5@unibo.it>
			Manuele Rusci 		<manuele.rusci@kuleuven.be>
			Daniele Palossi 	<dpalossi@ethz.ch> <daniele.palossi@supsi.ch>                   
 Date:   	01.04.2023                                                          
-------------------------------------------------------------------------------*/

// Flight 
#define FORWARD_VELOCITY          0.10f   // Max forward speed [m/s].  Default: 1.0f
#define MAX_SIDE_SPEED            0.20f   // Max forward speed [m/s].  Default: 1.0f
#define TARGET_H		              0.50f   // Target height for drone's flight [m].  Default: 0.5f

// Policy
#define CLOCKWISE                 1

// SPINNING
#define SPIN_TIME                 1500.0    // [ms]
#define SPIN_YAW_RATE             90.0      // [deg/s]
#define SPIN_ANGLE 	              180.0     // [deg]
#define RANDOM_SPIN_ANGLE         90.0      // [deg] add randomness to SPIN_ANGLE +/- RANDOM_SPIN_ANGLE

// TOF
#define TOF_FRONT_DIST_THRESHOLD  1000     // Target distance from obastacle [mm].  Default: 400.0f
#define SIDE_DISTANCE             600.0f  // Target distance from side walls [mm].  Default: 400.0f
#define SIDE_TOLERANCE            100.0f  // Target distance from side walls [mm].  Default: 100.0f
#define TOF_STATE_CHECK           0       // 0 or 1 to check tof.state -> enbles the error counter for invalid measurements
#define PROCESS_TOF_FLAGS         1       // process tof.state==2 into a distance =4 meters

// TOF counters
#define FRONT_COUNTERS_THRESHOLD  5       // 
#define SIDE_COUNTERS_THRESHOLD   5       // 
#define ERROR_COUNTERS_THRESHOLD  5       // 

/** THRESH_SIDE_INDENTATION: this is the threshold distance for wall following. 
 *  If the right/left sensors measure a free path of > side_dist+THRESH_SIDE_INDENTATION, 
 *  then there is an indentation on the environment and we can turn in that direction
 */
// Wallfollowing / Maze specific
#define THRESH_SIDE_INDENTATION 500 // [mm]

// Maze specific
#define INVERT_MAZE_AFTER_N_LAPS 4 // laps