/*-----------------------------------------------------------------------------
     Copyright (C) 2023 University of Bologna, Italy, ETH Zürich, Switzerland.
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

 File:      SSD_tin_can_bottle.c
 Authors:
            Lorenzo Lamberti    <lorenzo.lamberti@unibo.it>
            Luca Bompani        <luca.bompani5@unibo.it>
 Date:      01.04.2023
-------------------------------------------------------------------------------*/

#ifndef __OCRSSD_H__
#define __OCRSSD_H__

#define __PREFIX(x) SSD_tin_can_bottle ## x

#include "Gap.h"

#ifdef __EMUL__
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/param.h>
#include <string.h>
#endif

extern AT_HYPERFLASH_FS_EXT_ADDR_TYPE __PREFIX(_L3_Flash);

#endif
