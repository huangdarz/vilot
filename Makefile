################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=-U_U -U_L -U_N -U_S -U_P -U_C -U_X -U_B \
             -DUNIT_LIB_DISABLE_IOSTREAM \
             -DDISABLE_PREDEFINED_UNITS \
             -DENABLE_PREDEFINED_LENGTH_UNITS \
             -DENABLE_PREDEFINED_TIME_UNITS \
             -DENABLE_PREDEFINED_ANGLE_UNITS \
             -DENABLE_PREDEFINED_VELOCITY_UNITS \
             -DENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS \
             -DENABLE_PREDEFINED_ACCELERATION_UNITS \
             -DENABLE_PREDEFINED_VOLTAGE_UNITS \
             -DENABLE_PREDEFINED_FREQUENCY_UNITS \
             -DUNIT_LIB_DEFAULT_TYPE=float \
             -DEIGEN_DONT_VECTORIZE

EXTRA_CXXFLAGS=-U_U -U_L -U_N -U_S -U_P -U_C -U_X -U_B \
               -DUNIT_LIB_DISABLE_IOSTREAM \
               -DDISABLE_PREDEFINED_UNITS \
               -DENABLE_PREDEFINED_LENGTH_UNITS \
               -DENABLE_PREDEFINED_TIME_UNITS \
               -DENABLE_PREDEFINED_ANGLE_UNITS \
               -DENABLE_PREDEFINED_VELOCITY_UNITS \
               -DENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS \
               -DENABLE_PREDEFINED_ACCELERATION_UNITS \
               -DENABLE_PREDEFINED_VOLTAGE_UNITS \
               -DENABLE_PREDEFINED_FREQUENCY_UNITS \
               -DUNIT_LIB_DEFAULT_TYPE=float \
               -DEIGEN_DONT_VECTORIZE

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS! 
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
