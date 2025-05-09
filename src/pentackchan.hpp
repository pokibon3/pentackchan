//=========================================================
//  pentackchan.hpp :  pentackchan common Header
//  History    	: Vo.0  2025-05-09 New Create(K.Ohe)
//=========================================================
#pragma once
#include <stdint.h>
//=============================================================
// io definition
//=============================================================
/*
#define FR1_PIN  		 13      // 0 Motor Driving	AIN1/A_PWM 
#define FR2_PIN  		 12      // 1 Motor Driving	AIN1/A_PWM 
#define FL1_PIN  		 14      // 2 Motor Driving	AIN1/A_PWM 
#define FL2_PIN  		 27      // 3 Motor Driving	AIN1/A_PWM 
#define RR1_PIN  		 26      // 4 Motor Driving	AIN1/A_PWM 
#define RR2_PIN  		 25      // 5 Motor Driving	AIN1/A_PWM 
#define RL1_PIN  		 33      // 6 Motor Driving	AIN1/A_PWM 
#define RL2_PIN  		 32      // 7 Motor Driving	AIN1/A_PWM 
*/
#define FR1_PIN  		 25      // 0 Motor Driving	AIN1/A_PWM 
#define FR2_PIN  		 26      // 1 Motor Driving	AIN1/A_PWM 
#define FL1_PIN  		 33      // 2 Motor Driving	AIN1/A_PWM 
#define FL2_PIN  		 32      // 3 Motor Driving	AIN1/A_PWM 
#define RR1_PIN  		 12      // 4 Motor Driving	AIN1/A_PWM 
#define RR2_PIN  		 13      // 5 Motor Driving	AIN1/A_PWM 
#define RL1_PIN  		 14      // 6 Motor Driving	AIN1/A_PWM 
#define RL2_PIN  		 27      // 7 Motor Driving	AIN1/A_PWM 

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
