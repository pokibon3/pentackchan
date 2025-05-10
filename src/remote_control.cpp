//=========================================================
//  remote_control.cpp :  remote control for mover
//  History     : V0.0  2025-05-10 new create
//=========================================================
#include <Arduino.h>
#include <stdint.h>
#include "pentackchan.hpp"
#include "remote_control.hpp"

//=========================================================
//  Remote control command
//=========================================================
void remote_fwd(uint8_t speed)
{
    // FR
    ledcWrite(TIRE_FR1, speed); 
    ledcWrite(TIRE_FR2, SPEED_ZERO);
    //FL
    ledcWrite(TIRE_FL1, speed);
    ledcWrite(TIRE_FL2, SPEED_ZERO);
    //RR
    ledcWrite(TIRE_RR1, speed);
    ledcWrite(TIRE_RR2, SPEED_ZERO);
    //RL
    ledcWrite(TIRE_RL1, speed);
    ledcWrite(TIRE_RL2, SPEED_ZERO);
}

void remote_rwd(uint8_t speed)
{
    // FR
    ledcWrite(TIRE_FR1, SPEED_ZERO); 
    ledcWrite(TIRE_FR2, speed);
    //FL
    ledcWrite(TIRE_FL1, SPEED_ZERO);
    ledcWrite(TIRE_FL2, speed);
    //RR
    ledcWrite(TIRE_RR1, SPEED_ZERO);
    ledcWrite(TIRE_RR2, speed);
    //RL
    ledcWrite(TIRE_RL1, SPEED_ZERO);
    ledcWrite(TIRE_RL2, speed);
}

void remote_right(uint8_t speed)
{
    // FR
    ledcWrite(TIRE_FR1, speed); 
    ledcWrite(TIRE_FR2, SPEED_ZERO);
    // FL
    ledcWrite(TIRE_FL1, SPEED_ZERO);
    ledcWrite(TIRE_FL2, speed);
    // RR
    ledcWrite(TIRE_RR1, SPEED_ZERO);
    ledcWrite(TIRE_RR2, speed);
    // RL
    ledcWrite(TIRE_RL1, speed);
    ledcWrite(TIRE_RL2, SPEED_ZERO);
}

void remote_rightUp(uint8_t speed)
{
    // FR
    ledcWrite(TIRE_FR1, speed); 
    ledcWrite(TIRE_FR2, SPEED_ZERO);
    // FL
    ledcWrite(TIRE_FL1, SPEED_MAX);
    ledcWrite(TIRE_FL2, SPEED_MAX);
    // RR
    ledcWrite(TIRE_RR1, SPEED_MAX);
    ledcWrite(TIRE_RR2, SPEED_MAX);
    // RL
    ledcWrite(TIRE_RL1, speed);
    ledcWrite(TIRE_RL2, SPEED_ZERO);
}

void remote_leftDown(uint8_t speed)
{
    // FR
    ledcWrite(TIRE_FR1, SPEED_ZERO); 
    ledcWrite(TIRE_FR2, speed);
    // FL
    ledcWrite(TIRE_FL1, SPEED_MAX);
    ledcWrite(TIRE_FL2, SPEED_MAX);
    // RR
    ledcWrite(TIRE_RR1, SPEED_MAX);
    ledcWrite(TIRE_RR2, SPEED_MAX);
    // RL
    ledcWrite(TIRE_RL1, SPEED_ZERO);
    ledcWrite(TIRE_RL2, speed);
}

void remote_turnLeft(uint8_t speed)
{
    // FR
    ledcWrite(TIRE_FR1, speed); 
    ledcWrite(TIRE_FR2, SPEED_ZERO);
    // FL
    ledcWrite(TIRE_FL1, SPEED_ZERO);
    ledcWrite(TIRE_FL2, speed);
    // RR
    ledcWrite(TIRE_RR1, speed);
    ledcWrite(TIRE_RR2, SPEED_ZERO);
    // RL
    ledcWrite(TIRE_RL1, SPEED_ZERO);
    ledcWrite(TIRE_RL2, speed);
}

void remote_left(uint8_t speed)
{
    // FR
    ledcWrite(TIRE_FR1, SPEED_ZERO); 
    ledcWrite(TIRE_FR2, speed);
    //FL
    ledcWrite(TIRE_FL1, speed);
    ledcWrite(TIRE_FL2, SPEED_ZERO);
    //RR
    ledcWrite(TIRE_RR1, speed);
    ledcWrite(TIRE_RR2, SPEED_ZERO);
    //RL
    ledcWrite(TIRE_RL1, SPEED_ZERO);
    ledcWrite(TIRE_RL2, speed);
}
void remote_leftUp(uint8_t speed)
{
    // FR
    ledcWrite(TIRE_FR1, SPEED_MAX); 
    ledcWrite(TIRE_FR2, SPEED_MAX);
    //FL
    ledcWrite(TIRE_FL1, speed);
    ledcWrite(TIRE_FL2, SPEED_ZERO);
    //RR
    ledcWrite(TIRE_RR1, speed);
    ledcWrite(TIRE_RR2, SPEED_ZERO);
    //RL
    ledcWrite(TIRE_RL1, SPEED_MAX);
    ledcWrite(TIRE_RL2, SPEED_MAX);
}

void remote_rightDown(uint8_t speed)
{
    // FR
    ledcWrite(TIRE_FR1, SPEED_MAX); 
    ledcWrite(TIRE_FR2, SPEED_MAX);
    //FL
    ledcWrite(TIRE_FL1, SPEED_ZERO);
    ledcWrite(TIRE_FL2, speed);
    //RR
    ledcWrite(TIRE_RR1, SPEED_ZERO);
    ledcWrite(TIRE_RR2, speed);
    //RL
    ledcWrite(TIRE_RL1, SPEED_MAX);
    ledcWrite(TIRE_RL2, SPEED_MAX);
}

void remote_turnRight(uint8_t speed)
{
    // FR
    ledcWrite(TIRE_FR1, SPEED_ZERO); 
    ledcWrite(TIRE_FR2, speed);
    //FL
    ledcWrite(TIRE_FL1, speed);
    ledcWrite(TIRE_FL2, SPEED_ZERO);
    //RR
    ledcWrite(TIRE_RR1, SPEED_ZERO);
    ledcWrite(TIRE_RR2, speed);
    //RL
    ledcWrite(TIRE_RL1, speed);
    ledcWrite(TIRE_RL2, SPEED_ZERO);

}

void remote_stop(uint8_t speed)
{
    ledcWrite(TIRE_FR1, speed);
    ledcWrite(TIRE_FR2, speed);
    ledcWrite(TIRE_FL1, speed);
    ledcWrite(TIRE_FL2, speed);
    ledcWrite(TIRE_RR1, speed);
    ledcWrite(TIRE_RR2, speed);
    ledcWrite(TIRE_RL1, speed);
    ledcWrite(TIRE_RL2, speed);
}


