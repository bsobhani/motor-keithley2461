# Makefile
TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library
LIBRARY_IOC  = keithley2461Motor
#LIBRARY_IOC += aaaqq

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += devKeithley2461Motor.dbd

INC += keithley2461MotorDriver.h
#INC += smarActMCS2TriggerDriver.h

2Motor_CXXFLAGS += -w

# The following are compiled and added to the Support library
keithley2461Motor_SRCS += keithley2461MotorDriver.cpp
#smarActMCS2Motor_SRCS += keithley2461TriggerDriver.cpp
#smarActMCSMotor_SRCS += MCS2Example_Movement.c 

keithley2461Motor_LIBS += motor
keithley2461Motor_LIBS += asyn
#smarActMCS2Motor_SYS_LIBS += smaractctl 
keithley2461Motor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

