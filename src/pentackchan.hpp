//=========================================================
//  pentackchan.hpp :  pentackchan common Header
//  History    	: Vo.0  2025-05-09 New Create(K.Ohe)
//=========================================================
#pragma once
#include <stdint.h>
//=============================================================
// io definition
//=============================================================
#define FR1_PIN  		 25      // 0 Motor Driving	AIN1/A_PWM 
#define FR2_PIN  		 26      // 1 Motor Driving	AIN1/A_PWM 
#define FL1_PIN  		 33      // 2 Motor Driving	AIN1/A_PWM 
#define FL2_PIN  		 32      // 3 Motor Driving	AIN1/A_PWM 
#define RR1_PIN  		 12      // 4 Motor Driving	AIN1/A_PWM 
#define RR2_PIN  		 13      // 5 Motor Driving	AIN1/A_PWM 
#define RL1_PIN  		 14      // 6 Motor Driving	AIN1/A_PWM 
#define RL2_PIN  		 27      // 7 Motor Driving	AIN1/A_PWM 

#define TIRE_FR1		 0		// Front Tire
#define TIRE_FR2		 1		// Front Tire
#define TIRE_FL1		 2		// Front Tire
#define TIRE_FL2		 3		// Front Tire
#define TIRE_RR1		 4		// Rear Tire
#define TIRE_RR2		 5		// Rear Tire
#define TIRE_RL1		 6		// Rear Tire
#define TIRE_RL2		 7		// Rear Tire

#define SPEED_TYP		128		// Speed Type
#define SPEED_ZERO		  0		// Speed Max
#define SPEED_MAX		255		// Speed Max


#define LED_BUILTIN		 2		// Builtin LED
#define REMOTE_PIN		 15		// IR Remote Control

//=============================================================
//  RumiCar function definition
//=============================================================

//走行用の設定
enum {
	FREE	= 0,
	REVERSE	= 1,
	FORWARD	= 2,
	BRAKE	= 3,
};

// PWM function
#define PWM_LEVEL 	8
#define PWM_FREQ    490         // 490Hz
