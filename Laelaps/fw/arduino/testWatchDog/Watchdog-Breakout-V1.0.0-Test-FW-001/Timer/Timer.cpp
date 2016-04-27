// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	Timer.cpp
/// @brief	Timer algo

#include <math.h>

#include "Timer.h"

Timer::Timer(int delay_millis){
	current_time = millis();
	delta_time = delay_millis;
	last_trigger = current_time;
}

boolean Timer::trigger(){
	current_time = millis();
	if(current_time >= last_trigger + delta_time){
		last_trigger = last_trigger + delta_time;
		return true;
	} else {
		return false;
	}
}

void Timer::offset(int offset_millis){
	last_trigger = last_trigger + offset_millis;
}