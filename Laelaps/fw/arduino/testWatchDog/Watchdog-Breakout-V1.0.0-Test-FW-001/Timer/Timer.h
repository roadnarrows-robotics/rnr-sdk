// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	Timer.h
/// @brief	Timer algo

#ifndef Timer_h
#define Timer_h

#include <AP_Common.h>
#include <math.h>		// for fabs()

/// @class	Timer.h
/// @brief	Timer algo
class Timer {
	public:
		Timer(int delay_millis);
		boolean trigger();
		void offset(int offset_millis);
	private:
		int delta_time;             // number of mills in an iteration
		unsigned long last_trigger; //time of last iteration
		unsigned long current_time; //last sample of time
};

#endif
