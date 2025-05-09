//=========================================================
//  pentackchan.hpp :  pentackchan common Header
//  History    	: Vo.0  2025-05-09 New Create(K.Ohe)
//=========================================================
#pragma once
#include <stdint.h>
//=============================================================
// io definition
//=============================================================
#define FR1_PIN  		 13      // Motor Driving	AIN1/A_PWM 
#define FR2_PIN  		 12      // Motor Driving	AIN1/A_PWM 
#define FL1_PIN  		 14      // Motor Driving	AIN1/A_PWM 
#define FL2_PIN  		 27      // Motor Driving	AIN1/A_PWM 
#define RR1_PIN  		 26      // Motor Driving	AIN1/A_PWM 
#define RR2_PIN  		 25      // Motor Driving	AIN1/A_PWM 
#define RL1_PIN  		 33      // Motor Driving	AIN1/A_PWM 
#define RL2_PIN  		 32      // Motor Driving	AIN1/A_PWM 
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
