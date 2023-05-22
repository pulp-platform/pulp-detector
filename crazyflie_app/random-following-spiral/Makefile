# enable app support
APP=1
APP_STACKSIZE=500
SRC_DIR = src
INC_DIR = inc

VPATH += $(SRC_DIR)/
SOURCES_LOCAL = $(wildcard $(SRC_DIR)/*.c)
OBJECTS_LOCAL := $(SOURCES_LOCAL:.c=.o)
PROJ_OBJ += $(notdir $(OBJECTS_LOCAL))
INCLUDES += -I$(INC_DIR)

include ../Makefile.common
