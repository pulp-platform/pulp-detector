# Copyright (C) 2020 GreenWaves Technologies
# All rights reserved.

# This software may be modified and distributed under the terms
# of the BSD license.  See the LICENSE.BSD.md file for details.

ifndef GAP_SDK_HOME
  $(error Source sourceme in gap_sdk first)
endif

include common.mk

io=host
HAVE_HIMAX?=0
USE_CAMERA?=0
USE_LCD?=0
SILENT?=0
SHOW_PERF?=1

MEAS?=0
QUANT_BITS=8
MODEL_SQ8=1
$(info Building $(TARGET_CHIP_FAMILY) mode with $(QUANT_BITS) bit quantization)

IMAGE=$(CURDIR)/tflite_model/test_1_out.ppm
MAIN?=$(MODEL_PREFIX_SSD).c

include common/model_decl.mk

APP = SSD_tin_can_bottle

APP_SRCS += $(MAIN) $(MODEL_COMMON_SRCS) $(CNN_LIB)
APP_SRCS += BUILD_MODEL_SSD/SSD_tin_can_bottleKernels.c

APP_CFLAGS += -O3
APP_CFLAGS += -I. -I$(MODEL_COMMON_INC) -I$(TILER_EMU_INC) -I$(TILER_INC) $(CNN_LIB_INCLUDE)
APP_CFLAGS += -IBUILD_MODEL_SSD
APP_LDFLAGS += -lgaplib


PMSIS_OS=pulpos

JENKINS?=1
ifeq ($(JENKINS), 1)
	APP_CFLAGS += -DONE_ITER -DTEST #-DPERF
else
	ifeq ($(platform), gvsoc)
		APP_CFLAGS +=  -DONE_ITER -DHAVE_LCD #-DPERF
	else
		APP_CFLAGS += -DSILENT -DHAVE_HIMAX -DHAVE_LCD
	endif
endif

# FC stack size:
#MAIN_STACK_SIZE=4096
# CL stack size:

ifeq '$(TARGET_CHIP_FAMILY)' 'GAP9'
	CLUSTER_STACK_SIZE=4096
	ifeq '$(PMSIS_OS)' 'freertos'
		CLUSTER_SLAVE_STACK_SIZE=2048
	else
		CLUSTER_SLAVE_STACK_SIZE=1024
	endif
	TOTAL_STACK_SIZE=$(shell expr $(CLUSTER_STACK_SIZE) \+ $(CLUSTER_SLAVE_STACK_SIZE) \* 8)
	FREQ_CL?=50
	FREQ_FC?=50
else #GAP8
	CLUSTER_STACK_SIZE=4096
	CLUSTER_SLAVE_STACK_SIZE=1024
	TOTAL_STACK_SIZE=$(shell expr $(CLUSTER_STACK_SIZE) \+ $(CLUSTER_SLAVE_STACK_SIZE) \* 7)
	ifeq '$(TARGET_CHIP)' 'GAP8_V3'
	FREQ_CL?=175
	else
		FREQ_CL?=160
	endif
	FREQ_FC?=250
endif

APP_CFLAGS += -DSTACK_SIZE=$(CLUSTER_STACK_SIZE) -DSLAVE_STACK_SIZE=$(CLUSTER_SLAVE_STACK_SIZE)

APP_CFLAGS += -DAT_IMAGE=$(IMAGE) -DFREQ_CL=$(FREQ_CL) -DFREQ_FC=$(FREQ_FC)
APP_CFLAGS += -DAT_INPUT_HEIGHT_SSD=$(AT_INPUT_HEIGHT_SSD) -DAT_INPUT_WIDTH_SSD=$(AT_INPUT_WIDTH_SSD) -DAT_INPUT_COLORS_SSD=$(AT_INPUT_COLORS_SSD)
ifdef VERBOSE
	APP_CFLAGS +=-DVERBOSE=0
endif
ifdef FROM_JTAG
	APP_CFLAGS +=-DFROM_JTAG=0
endif
ifdef PERFORMANCE
	APP_CFLAGS +=-DPERFORMANCE=0
endif
#ifeq ($(PERF), 1)
#	APP_CFLAGS += -DPERF
#endif

MODEL_TENSORS = BUILD_MODEL_SSD/SSD_tin_can_bottle_L3_Flash_Const.dat
READFS_FILES=$(abspath $(MODEL_TENSORS))
PLPBRIDGE_FLAGS += -f



BUILD_MODEL_SSD/ssdlite_ocrKernels.c:
	make -f ssd.mk model CLUSTER_STACK_SIZE=$(CLUSTER_STACK_SIZE) CLUSTER_SLAVE_STACK_SIZE=$(CLUSTER_SLAVE_STACK_SIZE)


# all depends on the models
all:: BUILD_MODEL_SSD/ssdlite_ocrKernels.c

clean::
	rm -rf BUILD*

clean_models:
	rm -rf BUILD_MODEL*

#$(info APP_SRCS... $(APP_SRCS))
#$(info APP_CFLAGS... $(APP_CFLAGS))
include $(RULES_DIR)/pmsis_rules.mk

