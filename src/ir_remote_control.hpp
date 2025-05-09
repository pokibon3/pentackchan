//=========================================================
//  ir_remote_control.hpp :  ir remote control header 
//  History    : V1.0  2021-12-28 New Create(K.Ohe)
//=========================================================
#pragma once

#define REMOCON_AITENDO     0x00u
#define REMOCON_AKIZUKI     0x10u
//=============================================================
// Prototype definition
//=============================================================
extern void     ir_setup();
extern uint8_t  ir_read(void);

