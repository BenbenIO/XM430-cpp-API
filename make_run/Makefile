##################################################
# PROJECT: Dynamixel XM-430 motors API, compile example running the API
# AUTHOR : Benjamin IOLLER
##################################################

#---------------------------------------------------------------------
# Makefile template inspired by the original Dynamixel Makefile
#
# This Makefile will set the compiler option for a x86_64 (Ubuntu) or armv71 (Raspberry 3)
# Add additionnal library (OPENCV, ...) with the LIBRARIES variable
# Add additionnal sources code with the SOURCES variable
#
# Makefile will work for c and cpp source code.
#---------------------------------------------------------------------

# *** ENTER THE TARGET NAME HERE ***
TARGET      =  exampleServo

# Directories path creation: Include and Object
DIR_DXL    = ../
DIR_OBJS   = .objects

# Get platform architecture, set the correct Dynamixel library, and compiler option
ARCH :=$(shell arch)
ifeq ($(ARCH),armv7l)
	LIBRARIES += -ldxl_sbc_cpp
	CCFLAGS   = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) -g
	CXFLAGS   = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) -g
	INCLUDES  += -I$(DIR_DXL)/include/dynamixel_sdk_raspberry
endif

ifeq ($(ARCH),x86_64)
	LIBRARIES   += -ldxl_x64_cpp
	CCFLAGS     = -std=c++11 -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
	CXFLAGS     = -std=c++11 -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
	INCLUDES    += -I$(DIR_DXL)/include/dynamixel_sdk_ubuntu
	FORMAT      = -m64
endif
	
# General compiler options
CC          = gcc
CX          = g++
LNKCC       = $(CX)
LNKFLAGS    = $(CXFLAGS)

#---------------------------------------------------------------------
# Core components (all of these are likely going to be needed)
#---------------------------------------------------------------------
LIBRARIES  += -lrt

#---------------------------------------------------------------------
# Source files
#---------------------------------------------------------------------
SOURCES = exampleServo.cpp
SOURCES += motorMX430.cpp

#---------------------------------------------------------------------
# Object recipe
#---------------------------------------------------------------------
OBJECTS  = $(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))

#---------------------------------------------------------------------
# Compiling Rules
#---------------------------------------------------------------------
$(TARGET): make_directory $(OBJECTS)
	$(LNKCC) $(LNKFLAGS) $(OBJECTS) -o $(TARGET) $(LIBRARIES)

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(DIR_OBJS) core *~ *.a *.so *.lo

make_directory:
	mkdir -p $(DIR_OBJS)/

$(DIR_OBJS)/%.o: ../%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: ../%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

#---------------------------------------------------------------------
# End of Makefile
#---------------------------------------------------------------------
