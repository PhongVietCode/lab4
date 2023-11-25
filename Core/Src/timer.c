/*
 * timer.c
 *
 *  Created on: Nov 24, 2023
 *      Author: phongtran
 */
#include "timer.h"
#define TIME_CYCLE 10
int timer1_flag = 0;
int timer1_duration = 0;

void timerRun(){
	if(timer1_duration > 0){
		timer1_duration--;
		if(timer1_duration <= 0) timer1_flag = 1;
	}
}
void setTimer1(int duration){
	timer1_duration = duration/TIME_CYCLE;
	timer1_flag = 0;
}

