#-------------------------------------------------------------------------------#
# Copyright (C) 2023 University of Bologna, Italy, ETH Zürich, Switzerland.     #
# All rights reserved.                                                          #
#                                                                               #
# Licensed under the Apache License, Version 2.0 (the "License");               #
# you may not use this file except in compliance with the License.              #
# See LICENSE.apache.md in the top directory for details.                       #
# You may obtain a copy of the License at                                       #
#                                                                               #
#   http://www.apache.org/licenses/LICENSE-2.0                                  #
#                                                                               #
# Unless required by applicable law or agreed to in writing, software           #
# distributed under the License is distributed on an "AS IS" BASIS,             #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.      #
# See the License for the specific language governing permissions and           #
# limitations under the License.                                                #
#                                                                               #
# Authors:                                                                      #
#           Lorenzo Lamberti    <lorenzo.lamberti@unibo.it>                     #
#           Luca Bompani        <luca.bompani5@unibo.it>                        #
# Date:     01.04.2023                                                          #
#-------------------------------------------------------------------------------#

# enable app support
APP=1
APP_STACKSIZE=300
SRC_DIR = src
INC_DIR = inc

VPATH += $(SRC_DIR)/
SOURCES_LOCAL = $(wildcard $(SRC_DIR)/*.c)
OBJECTS_LOCAL := $(SOURCES_LOCAL:.c=.o)
PROJ_OBJ += $(notdir $(OBJECTS_LOCAL))
INCLUDES += -I$(INC_DIR)

include ../Makefile.common
