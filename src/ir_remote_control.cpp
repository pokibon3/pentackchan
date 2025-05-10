//=========================================================
//  ir_remote_control.cpp :  RumiCar ir remote control for mover
//  History     : V0.0  2025-05-10 new create
//=========================================================
#include <stdint.h>
#include "pentackchan.hpp"
#include "ir_remote_control.hpp"
#include <IRremote.hpp>

//=========================================================
//  IR Remote control definiton
//=========================================================
#define IR_RECEIVE_PIN	15	// IR Receiver 
#ifdef REPEAT
#undef REPEAT
#endif
#define REPEAT			2	// repeat 3 times
IRData *ir_results;

//=============================================================
// Prototype definition
//=============================================================
static uint8_t decode_aitendo(uint8_t ir_cmd);
static uint8_t decode_akizuki(uint8_t ir_cmd);

//=========================================================
//  ir_setup()
//=========================================================
void ir_setup()
{
	IrReceiver.begin(IR_RECEIVE_PIN);
}

static int8_t repeat_count = 0;	// cmd repeat count
//=========================================================
//  ir_read
//=========================================================
uint8_t ir_read() 
{
    uint8_t  ir_addr = 0;
    uint8_t  ir_cmd = 0;
    static uint8_t  cmd = 0;

    if (IrReceiver.available()) {
		ir_results 	= IrReceiver.read(); 
        if (ir_results -> protocol != NEC) {
            Serial.println(F("#Unknown IR Protocol"));
		} else {
			ir_cmd 		= ir_results -> command; 
			ir_addr 	= ir_results -> address;

			if 			(ir_addr == REMOCON_AITENDO) {
	        	cmd = decode_aitendo(ir_cmd);
			} else if 	(ir_addr == REMOCON_AKIZUKI) {
				cmd = decode_akizuki(ir_cmd);
			}
			Serial.print("#irAddr: ");
			Serial.print(ir_addr, HEX);
			Serial.print("\tirCode: ");
			Serial.print(ir_cmd, HEX);
			Serial.print("\t cmd: ");
			Serial.print((char)cmd);
			Serial.print("\t:0x");
			Serial.println(cmd, HEX);
//			SerialBT.print("\t:0x");
//			SerialBT.println(cmd, HEX);
		}
        IrReceiver.resume();         // clear buffer
    } else {
		if (repeat_count > 0) {
			repeat_count--;
		} else {
			cmd = 0;
		}
	}
    return cmd;
}

//=========================================================
//  IR Command decoder for Aitendo remocon
//=========================================================
static uint8_t decode_aitendo(uint8_t ir_cmd)
{
	int8_t  cmd;

	switch(ir_cmd) {				// AITENDO	CN
		case 0x46 :					// '↑'		'2'
		case 0x18 :					// '5'		'↑'
			repeat_count = REPEAT;
			cmd = 'F';
			break;
		case 0x15 :					// '↓'		'8'
		case 0x52 :					// '0'		'↓'
			repeat_count = REPEAT;
			cmd = 'B';
			break;
		case 0x44 :					// '←' 		'4'
		case 0x08 :					// '7'		'←'
			repeat_count = REPEAT;
			cmd = '<';
			break;
		case 0x43 :					// '→'		'6'
		case 0x5a :					// '9'		'→'
			repeat_count = REPEAT;
			cmd = '>';
			break;
		case 0x45 :					// 			'1'
		case 0x0c :					// '4'
			repeat_count = REPEAT;
			cmd = 'L';
			break;
		case 0x47 :					// 			'3'
		case 0x5e :					// '6'
			repeat_count = REPEAT;
			cmd = 'R';
			break;
		case 0x07 :					// 			'7'
		case 0x42 :					// '*'
			repeat_count = REPEAT;
			cmd = 'l';
			break;
		case 0x09 :					// 			'9'
		case 0x4a :					// '#'
			repeat_count = REPEAT;
			cmd = 'r';
			break;
		case 0x40 :					// 'OK'		'5'
		case 0x1c :					// '8'		'OK'
			cmd = 'Q';
			break;
		case 0x16 :					// ’1’		'*'
			cmd = 'A';
			break;
		case 0x19 :					// '2'		'0'
			cmd = 'M';
			break;
		case 0x0d :					// '3'		'#'
			cmd = 'D';
			break;
		default   :
			cmd = 0;
	}
	return cmd;  
}

//=========================================================
//  IR Command decoder for Akizuki remocon
//=========================================================
static uint8_t decode_akizuki(uint8_t ir_cmd)
{
	int8_t  cmd;

	switch(ir_cmd) {
		case 0xa0 :				// '↑'
			repeat_count = REPEAT;
			cmd = 'F';
			break;
		case 0x00 :				// '↓'
			repeat_count = REPEAT;
			cmd = 'B';
			break;
		case 0x10 :				// '←'
			repeat_count = REPEAT;
			cmd = '<';
			break;
		case 0x80 :				// '→'
			repeat_count = REPEAT;
			cmd = '>';
			break;
		case 0xB1 :				// '↖'
			repeat_count = REPEAT;
			cmd = 'L';
			break;
		case 0x21 :				// '↗'
			repeat_count = REPEAT;
			cmd = 'R';
			break;
		case 0x11 :				// '↙'
			repeat_count = REPEAT;
			cmd = 'l';
			break;
		case 0x81 :				// '↘'
			repeat_count = REPEAT;
			cmd = 'r';
			break;
		case 0xd8 :				// POWER
		case 0x20 :				// 空白
			cmd = 'Q';
			break;
		case 0xf8 :				// 'A'
			cmd = 'A';
			break;
		case 0x78 :				// 'B'
			cmd = 'B';
			break;
		case 0x58 :				// 'C'
			cmd = 'C';
			break;
		default   :				// other button
			cmd = 0;
	}
	return cmd;  
}
