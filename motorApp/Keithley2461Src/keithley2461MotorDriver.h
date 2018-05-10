#ifndef KEITHLEY_2461_MOTOR_DRIVER_H
#define KEITHLEY_2461_MOTOR_DRIVER_H

/* Motor driver support for smarAct MCS RS-232 Controller   */

/* Derived from ACRMotorDriver.cpp by Mark Rivers, 2011/3/28 */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 9/11 */

#ifdef __cplusplus

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <stdarg.h>
#include <exception>

enum Keithley2461ExceptionType {
	MCS2UnknownError,
	MCS2ConnectionError,
	MCS2CommunicationError,
};

class Keithley2461Exception : public std::exception {
public:
	Keithley2461Exception(Keithley2461ExceptionType t, const char *fmt, ...);
	Keithley2461Exception(Keithley2461ExceptionType t)
		: t_(t)
		{ str_[0] = 0; }
	Keithley2461Exception()
		: t_(MCS2UnknownError)
		{ str_[0] = 0; }
	Keithley2461Exception(Keithley2461ExceptionType t, const char *fmt, va_list ap);
	Keithley2461ExceptionType getType()
		const { return t_; }
	virtual const char *what()
		const throw() {return str_ ;}
protected:
	char str_[100];	
	Keithley2461ExceptionType t_;
};

//void printddd();
void printccc();
class Keithley2461Axis : public asynMotorAxis
{
public:
	Keithley2461Axis(class Keithley2461Controller *cnt_p, int axis, int channel, const char* type);
	asynStatus move(double position, int relative, double min_vel, double max_vel, double accel);
	asynStatus home(double min_vel, double max_vel, double accel, int forwards);
	asynStatus stop(double acceleration);
	asynStatus poll(bool* moving_p);
	int channel_;
private:
	Keithley2461Controller* c_p_;
	asynStatus comStatus_;
	enum type_enum {VOLTAGE, CURRENT};
	int type_;

};

class Keithley2461Controller : public asynMotorController
{
public:
	Keithley2461Controller(const char *portName, const char *IOPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
	//virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
	//int64_t motorEncoderPosition_;
	//int64_t motorPosition_;
	asynStatus sendCmd(size_t *got_p, char *rep, int len, double timeout, const char *fmt, va_list ap);
	asynStatus sendCmd(size_t *got_p, char *rep, int len, double timeout, const char *fmt, ...);
	asynStatus sendCmd(size_t *got_p, char *rep, int len, const char *fmt, ...);
	asynStatus sendCmd(char *rep, int len, const char *fmt, ...);
private:
	asynUser *asynUserMot_p_;

friend class Keithley2461Axis;
};

int main1();

#endif // _cplusplus
#endif // SMARACT_MCS2_MOTOR_DRIVER_H
