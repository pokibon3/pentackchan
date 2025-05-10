//=========================================================
//  main.cpp :    pentackchan-rover
//  History     : V0.0  2025-05-08 new create
//=========================================================
#include <Arduino.h>
#include "pentackchan.hpp"
#include "ir_remote_control.hpp"
#include "remote_control.hpp"

//=========================================================
//  prototypes definition
//=========================================================
static uint8_t remote_control(void);

//=========================================================
//  Arduino setup function
//=========================================================
void pwm_setup()
{
    pinMode(FR1_PIN, OUTPUT);
    pinMode(FR2_PIN, OUTPUT);
    pinMode(FL1_PIN, OUTPUT);
    pinMode(FL2_PIN, OUTPUT);
    pinMode(RR1_PIN, OUTPUT);
    pinMode(RR2_PIN, OUTPUT);
    pinMode(RL1_PIN, OUTPUT);
    pinMode(RL2_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(REMOTE_PIN, INPUT);

    ledcSetup(0, PWM_FREQ, PWM_LEVEL);
    ledcSetup(1, PWM_FREQ, PWM_LEVEL);
    ledcSetup(2, PWM_FREQ, PWM_LEVEL);
    ledcSetup(3, PWM_FREQ, PWM_LEVEL);
    ledcSetup(4, PWM_FREQ, PWM_LEVEL);
    ledcSetup(5, PWM_FREQ, PWM_LEVEL);
    ledcSetup(6, PWM_FREQ, PWM_LEVEL);
    ledcSetup(7, PWM_FREQ, PWM_LEVEL);
    
    ledcAttachPin(FR1_PIN, 0);
    ledcAttachPin(FR2_PIN, 1);
    ledcAttachPin(FL1_PIN, 2);
    ledcAttachPin(FL2_PIN, 3);
    ledcAttachPin(RR1_PIN, 4);
    ledcAttachPin(RR2_PIN, 5);
    ledcAttachPin(RL1_PIN, 6);
    ledcAttachPin(RL2_PIN, 7);
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    ledcWrite(4, 0);
    ledcWrite(5, 0);
    ledcWrite(6, 0);
    ledcWrite(7, 0);
    digitalWrite(LED_BUILTIN, HIGH);
}

void setup()
{
    Serial.begin(115200);

    Serial.println("# Start RumiCar ROVER!");
    pwm_setup();
    ir_setup();
}

//=========================================================
//  Arduino Main function
//=========================================================
void loop()
{
    uint8_t cmd;
  
    cmd = remote_control();
//  remoteCmd(cmd);
    delay(1);
}

//=========================================================
//  remote Control
//=========================================================
static uint8_t remote_control()
{
    uint8_t  cmd    = 0;;
    uint8_t  ir_cmd = 0;                  // ir remote command
    //--------------------------
    //   Read IR Remocon 
    //--------------------------
    ir_cmd = ir_read();

    if (ir_cmd != 0) {
        cmd = ir_cmd;
    } else {
        cmd = 0;
    }

    if (cmd == 'F') {           // FR
        remote_fwd(SPEED_TYP);
    } else if (cmd ==  'B') {   // FL
        remote_rwd(SPEED_TYP);
    } else if (cmd ==  '>') {   // RR
        remote_right(SPEED_TYP);
    } else if (cmd ==  '<') {   // RL
        remote_left(SPEED_TYP);
    } else if (cmd ==  'Q') {
        remote_stop(SPEED_MAX);
    } 
//    Serial.printf("cmd = %c(0x%02x)\n", cmd, cmd & 0xff);
    return cmd;
}

