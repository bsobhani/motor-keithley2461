/**
 * Motor record driver for Keithley 2461 SourceMeter
 *
 * Author: Alex Sobhani
 *
 */


#include <iocsh.h>

#include <asynOctetSyncIO.h>
#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <keithley2461MotorDriver.h>
#include <errlog.h>

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <exception>

#include <math.h>

#include <epicsString.h>
#include <epicsExport.h>

#include "iocsh.h"
#include "asynPortDriver.h"
#include <epicsTypes.h>
#include <cstdlib>

enum Keithley2461Status {
	Stopped     = 0,
	Stepping    = 1,
	Scanning    = 2,
	Holding     = 3,
	Targeting   = 4,
	MoveDelay   = 5,
	Calibrating = 6,
	FindRefMark = 7,
	Locked      = 9
};

Keithley2461Exception::Keithley2461Exception(Keithley2461ExceptionType t, const char *fmt, ...)
	: t_(t)
{
va_list ap;
	if ( fmt ) {
		va_start(ap, fmt);
		epicsVsnprintf(str_, sizeof(str_), fmt, ap);
		va_end(ap);
	} else {
		str_[0] = 0;
	}
};

Keithley2461Exception::Keithley2461Exception(Keithley2461ExceptionType t, const char *fmt, va_list ap)
		: t_(t)
{
	epicsVsnprintf(str_, sizeof(str_), fmt, ap);
}

Keithley2461Controller::Keithley2461Controller(const char *portName, const char *IOPortName, int numAxes, double movingPollPeriod, double idlePollPeriod)
	: asynMotorController(portName, numAxes,
	                      0, // parameters
	                      0, // interface mask
	                      0, // interrupt mask
	                      ASYN_CANBLOCK | ASYN_MULTIDEVICE,
	                      1, // autoconnect
	                      0,0) // default priority and stack size
		, asynUserMot_p_(0)	
{

	asynStatus status;
	char* buf;
	buf="*IDN?";
	char read_buf[100];
	status = pasynOctetSyncIO->connect(IOPortName, 0, &asynUserMot_p_, NULL);	
	/*
	if(status){
		printf("Error connecting to controller\n");
	}
	else{
		printf("Connected!\n");
	}
	*/
	size_t nbytesOut;
	size_t nbytesIn;
	int eomReason;
	pasynOctetSyncIO->setInputEos ( asynUserMot_p_, "\n", 1 );
	pasynOctetSyncIO->setOutputEos( asynUserMot_p_, "\n", 1 );
	pasynOctetSyncIO->writeRead( asynUserMot_p_, buf, strlen(buf), read_buf, 100, 10, &nbytesOut, &nbytesIn, &eomReason);
	printf(read_buf);
	printf("\n");
	startPoller(.5,.5,0);

}

asynStatus Keithley2461Axis::stop(double acceleration){
	return asynSuccess;
}

asynStatus Keithley2461Axis::poll(bool* moving_p){
	int val;
	comStatus_=asynSuccess;
	if(comStatus_!=asynSuccess){
		*moving_p=false;
		asynPrint(c_p_->pasynUserSelf,ASYN_TRACE_ERROR,"Error reading axis, channel=%d, comstatus=%d!\n", channel_,comStatus_);
		setIntegerParam(c_p_->motorStatusProblem_,1);
		setIntegerParam(c_p_->motorStatusCommsError_,1);
		setIntegerParam(c_p_->motorStatusMoving_,*moving_p);
		setIntegerParam(c_p_->motorStatusDone_, ! *moving_p);
		//initmcs();
		callParamCallbacks();
		return comStatus_;
	}
	int state;
	//setIntegerParam(c_p_->motorStatusHighLimit_, (state & SA_CTL_CH_STATE_BIT_END_STOP_REACHED)/SA_CTL_CH_STATE_BIT_END_STOP_REACHED);
	//setIntegerParam(c_p_->motorStatusLowLimit_, (state & SA_CTL_CH_STATE_BIT_END_STOP_REACHED)/SA_CTL_CH_STATE_BIT_END_STOP_REACHED);
	//setIntegerParam(c_p_->motorStatusFollowingError_, (state & SA_CTL_CH_STATE_BIT_FOLLOWING_LIMIT_REACHED)/SA_CTL_CH_STATE_BIT_FOLLOWING_LIMIT_REACHED);
	char* buf;
	if(type_==VOLTAGE){
		buf="MEAS:VOLT?";
	}
	else if(type_==CURRENT){
		buf="MEAS:CURR?";
	}
	char read_buf[100];
	size_t nbytesOut;
	size_t nbytesIn;
	int eomReason;
	pasynOctetSyncIO->writeRead( c_p_->asynUserMot_p_, buf, strlen(buf), read_buf, 100, 10, &nbytesOut, &nbytesIn, &eomReason);
	double voltage;
	voltage=atof(read_buf);
	voltage=voltage*1e7;
	double accel;
	val=val/1000;
	setDoubleParam(c_p_->motorEncoderPosition_,voltage);
	setDoubleParam(c_p_->motorPosition_,voltage);
	if(false==true){
		*moving_p=true;
	}
	else{
		*moving_p=false;
	}
	setIntegerParam(c_p_->motorStatusProblem_,0);
	setIntegerParam(c_p_->motorStatusCommsError_,0);

	setIntegerParam(c_p_->motorStatusMoving_,*moving_p);
	setIntegerParam(c_p_->motorStatusDone_, ! *moving_p);
	callParamCallbacks();
	return asynSuccess;
}
Keithley2461Axis::Keithley2461Axis(class Keithley2461Controller *cnt_p, int axis, int channel, const char* type)
	: asynMotorAxis(cnt_p, axis), c_p_(cnt_p)
{
	channel_=channel;
	if(strcmp(type,"voltage")==0){
		type_ = VOLTAGE;
	}
	else if(strcmp(type,"current")==0){
		type_ = CURRENT;
	}
	else{
		printf("Error: Unknown type\n");
		return;
	}
}

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
static const iocshArg cc_a0 = {"Port name [string]",               iocshArgString};
static const iocshArg cc_a1 = {"I/O port name [string]",           iocshArgString};
static const iocshArg cc_a2 = {"Number of axes [int]",             iocshArgInt};
static const iocshArg cc_a3 = {"Moving poll period (s) [double]",  iocshArgDouble};
static const iocshArg cc_a4 = {"Idle poll period (s) [double]",    iocshArgDouble};

static const iocshArg * const cc_as[] = {&cc_a0, &cc_a1, &cc_a2, &cc_a3, &cc_a4};

static const iocshFuncDef cc_def = {"keithley2461CreateController", sizeof(cc_as)/sizeof(cc_as[0]), cc_as};



asynStatus  
Keithley2461Axis::move(double position, int relative, double min_vel, double max_vel, double accel)
{
	/*if(comStatus_!=asynSuccess){
		printf("Move Error\n");
		return asynError;
	}
	asynPrint(c_p_->pasynUserSelf,ASYN_TRACE_FLOW,"Moving to %f, velo=%f, accel=%f, channel=%d!\n", position,max_vel,accel,channel_);*/
	char buf[100];
	if(type_==VOLTAGE){
		sprintf(buf,"SOUR:FUNC VOLT\nSOUR:VOLT %lf\nOUTP ON",position/1e7);
	}
	else if(type_==CURRENT){
		sprintf(buf,"SOUR:FUNC CURR\nSOUR:CURR %lf\nOUTP ON",position/1e7);
	}
	char read_buf[100];
	size_t nbytesOut;
	size_t nbytesIn;
	int eomReason;
	pasynOctetSyncIO->writeRead( c_p_->asynUserMot_p_, buf, strlen(buf), read_buf, 100, .01, &nbytesOut, &nbytesIn, &eomReason);

	return asynSuccess;
} 

asynStatus  
Keithley2461Axis::home(double min_vel, double max_vel, double accel, int forwards)
{
	return asynSuccess;
} 


extern "C" void *
keithley2461CreateController(
	const char *motorPortName,
	const char *ioPortName,
	int         numAxes,
	double      movingPollPeriod,
	double      idlePollPeriod)
{
	
	return new Keithley2461Controller(motorPortName,ioPortName,numAxes,movingPollPeriod,idlePollPeriod);
	//return 0;
}

static void cc_fn(const iocshArgBuf *args)
{
	keithley2461CreateController(
		args[0].sval,
		args[1].sval,
		args[2].ival,
		args[3].dval,
		args[4].dval);
}


static const iocshArg ca_a0 = {"Controller Port name [string]",    iocshArgString};
static const iocshArg ca_a1 = {"Axis number [int]",                iocshArgInt};
static const iocshArg ca_a2 = {"Channel [int]",                    iocshArgInt};
static const iocshArg ca_a3 = {"Unit Type [string]",                    iocshArgString};

static const iocshArg * const ca_as[] = {&ca_a0, &ca_a1, &ca_a2, &ca_a3};

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
/* smarActMCS2CreateAxis called to create each axis of the smarActMCS2 controller*/
static const iocshFuncDef ca_def = {"keithley2461CreateAxis", 4, ca_as};

extern "C" void *
keithley2461CreateAxis(
	const char *controllerPortName,
	int        axisNumber,
	int        channel,
	const char*	   ttype)

{
	Keithley2461Controller *pC;
	pC = (Keithley2461Controller*) findAsynPortDriver(controllerPortName);
	if (!pC) {
		printf("keithley2461CreateAxis: Error port %s not found\n", controllerPortName);
		return 0;
	}
	pC->lock();
	Keithley2461Axis* sa = new Keithley2461Axis(pC, axisNumber, channel, ttype);
	pC->unlock();
	
	return sa;
}

static void ca_fn(const iocshArgBuf *args)
{
	keithley2461CreateAxis(
		args[0].sval,
		args[1].ival,
		args[2].ival,
		args[3].sval);
}
void cc_ft(const iocshArgBuf* args);
extern iocshFuncDef ta_def;

static void keithley2461MotorRegister(void)
{
  iocshRegister(&cc_def, cc_fn);  // smarActMCS2CreateController
  iocshRegister(&ca_def, ca_fn);  // smarActMCS2CreateAxis
  //iocshRegister(&ta_def, cc_ft);  // smarActMCS2CreateTrigger 
}

extern "C" {
epicsExportRegistrar(keithley2461MotorRegister);
}
