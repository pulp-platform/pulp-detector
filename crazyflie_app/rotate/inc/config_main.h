/*-----------------------------------------------------------------------------
 Copyright (C) 2023 University of Bologna, Italy, ETH Zurich, Switzerland. 
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

 File:    config_main.h                                                              
 Author:  Lorenzo Lamberti      <lorenzo.lamberti@unibo.it>                           
 Date:    18.02.2022                                                           
-------------------------------------------------------------------------------*/


// Flight mission
#define FORWARD_VELOCITY    0.30f    // Max forward speed [m/s].  Default: 1.0f
#define TARGET_H		        0.50f   // Target height for drone's flight [m].  Default: 0.5f
#define TARGET_D		        400.0f  // Target distance from obastacle [mm].  Default: 400.0f
