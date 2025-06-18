//=========================================================
//  main.cpp :    pentackchan-rover with IR Remote + Dabble JoyPad
//  History     : V1.1  2025-06-07 IR Remote + Dabble JoyPad control
//=========================================================
#include <Arduino.h>
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include "pentackchan.hpp"
#include "ir_remote_control.hpp"
#include "remote_control.hpp"

//=========================================================
//  prototypes definition
//=========================================================
static uint8_t remote_control(void);
static void dabble_joypad_control(void);
static uint8_t apply_motor_calibration(uint8_t base_speed, float ratio);
static void execute_motor_command(uint8_t cmd);
static void setMotorPWM(uint8_t chFwd, uint8_t chRev, uint8_t speed, bool forward);

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
    
    // Initialize all PWM to 0
    for(int i = 0; i < 8; i++) {
        ledcWrite(i, 0);
    }
    
    digitalWrite(LED_BUILTIN, HIGH);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("# Start RumiCar ROVER with IR Remote + Dabble JoyPad!");
    
    pwm_setup();
    ir_setup();  // Initialize IR remote
    
    // Initialize Dabble
    Dabble.begin("A_PentackChan");  // Bluetooth device name
    Serial.println("# IR Remote and Dabble initialized.");
    Serial.println("# Connect via Dabble app or use IR remote control.");
}

//=========================================================
//  Arduino Main function
//=========================================================
void loop()
{
    static ControlMode active_control = CONTROL_NONE;
    static unsigned long last_activity_time = 0;
    static bool dabble_connected_last = false;
    
    uint8_t ir_cmd = 0;
    uint8_t control_cmd = 0;
    
    // Process Dabble input
    Dabble.processInput();
    bool dabble_connected = Dabble.isAppConnected();
    
    // Check Dabble connection status change
    if (dabble_connected != dabble_connected_last) {
        if (dabble_connected) {
            Serial.println("# Dabble app connected!");
            digitalWrite(LED_BUILTIN, LOW);  // LED on when connected
        } else {
            Serial.println("# Dabble app disconnected!");
            digitalWrite(LED_BUILTIN, HIGH); // LED off when disconnected
            if (active_control == CONTROL_DABBLE) {
                remote_stop(SPEED_MAX);  // Stop when Dabble disconnects
                active_control = CONTROL_NONE;
            }
        }
        dabble_connected_last = dabble_connected;
    }
    
    // Read IR Remote command
    ir_cmd = ir_read();
    
    // Check for IR Remote activity
    if (ir_cmd != 0) {
        active_control = CONTROL_IR;
        last_activity_time = millis();
        control_cmd = ir_cmd;
        Serial.println("# Control mode: IR Remote");
    }
    
    // Check for Dabble JoyPad activity (only if connected)
    if (dabble_connected) {
        int x_axis = GamePad.getXaxisData();
        int y_axis = GamePad.getYaxisData();
        //Serial.printf("# Dabble JoyPad: X=%d, Y=%d\n", x_axis, y_axis);

        bool button_pressed = false;
        
        // Check if any button is pressed
        for (int i = 0; i < 8; i++) {
            if (GamePad.isPressed(i)) {
                button_pressed = true;
                break;
            }
        }
        
        // Check for significant joystick movement or button press
        if (abs(x_axis) > JOYPAD_DEADZONE || abs(y_axis) > JOYPAD_DEADZONE || button_pressed) {
            active_control = CONTROL_DABBLE;
            last_activity_time = millis();
            Serial.println("# Control mode: Dabble JoyPad");
        }
    }
    
    // Execute control based on active mode
    switch (active_control) {
        case CONTROL_IR:
            if (ir_cmd != 0) {
                execute_motor_command(ir_cmd);
            }
            // Auto-stop if no IR command for a while
            if (millis() - last_activity_time > 100) {
                remote_stop(SPEED_MAX);
                active_control = CONTROL_NONE;
            }
            break;
            
        case CONTROL_DABBLE:
            if (dabble_connected) {
                dabble_joypad_control();
            } else {
                active_control = CONTROL_NONE;
            }
            break;
            
        case CONTROL_NONE:
        default:
            // No active control - robot stops
            break;
    }
    
    delay(10);  // Small delay for stability
}

//=========================================================
//  IR Remote Control (original function)
//=========================================================
static uint8_t remote_control()
{
    uint8_t  cmd    = 0;
    uint8_t  ir_cmd = 0;
    
    ir_cmd = ir_read();

    if (ir_cmd != 0) {
        cmd = ir_cmd;
    } else {
        cmd = 0;
    }

    execute_motor_command(cmd);
    return cmd;
}

//=========================================================
//  Execute Motor Command (common for IR and special commands)
//=========================================================
static void execute_motor_command(uint8_t cmd)
{
    if        (cmd == 'F') {   // Forward
        remote_fwd(SPEED_TYP);
    } else if (cmd == 'B') {   // Backward
        remote_rwd(SPEED_TYP);
    } else if (cmd == '>') {   // Right
        remote_right(SPEED_TYP);
    } else if (cmd == 'R') {   // RightUp
        remote_rightUp(SPEED_TYP);
    } else if (cmd == 'r') {   // RightDown
        remote_rightDown(SPEED_TYP);
    } else if (cmd == '<') {   // Left
        remote_left(SPEED_TYP);
    } else if (cmd == 'L') {   // LeftUp
        remote_leftUp(SPEED_TYP);
    } else if (cmd == 'l') {   // LeftDown
        remote_leftDown(SPEED_TYP);
    } else if (cmd == 'A') {   // Turn Left
        remote_turnLeft(SPEED_TYP);
    } else if (cmd == 'C') {   // Turn Right
        remote_turnRight(SPEED_TYP);
    } else if (cmd == 'Q') {   // Stop
        remote_stop(SPEED_MAX);
    }
}

//=========================================================
//  Dabble JoyPad Control
//=========================================================
static void dabble_joypad_control()
{
    // Get joystick values
    int x_axis = - GamePad.getXaxisData();  // -100 to +100 (left/right)
    int y_axis = GamePad.getYaxisData();  // -100 to +100 (forward/backward)
    
    // Check for emergency stop button
    if (GamePad.isPressed(0)) {
        remote_stop(SPEED_MAX);
        Serial.println("# Emergency Stop!");
        return;
    }
    
    // Check for preset movement buttons
    if (GamePad.isTrianglePressed()) {         // Triangle - Forward
        execute_motor_command('F');
        Serial.println("# Move Forward"); 
        return;
    } else if (GamePad.isCrossPressed()) {  // Cross - Backward
        execute_motor_command('B');

        return;
    } else if (GamePad.isStartPressed()) {  // Start - Stop
        execute_motor_command('Q');
        Serial.println("# Stop");
        return;
    } else if (GamePad.isSelectPressed()) {  // Select - Stop
        execute_motor_command('Q');
        Serial.println("# Stop");
        return;
    } else if (GamePad.isSquarePressed()) {  // Select - Turn Left
        execute_motor_command('A');
        Serial.println("# Turn Left");
        return;
    } else if (GamePad.isCirclePressed()) {  // Turn Right  
        execute_motor_command('C');
        Serial.println("# Turn Right");
        return;
    }
    
    // Apply deadzone to joystick
    if (abs(x_axis) <= JOYPAD_DEADZONE) x_axis = 0;
    if (abs(y_axis) <= JOYPAD_DEADZONE) y_axis = 0;
    
    // If no significant joystick movement, stop
    if (x_axis == 0 && y_axis == 0) {
        remote_stop(SPEED_MAX);
        return;
    }

    float x = x_axis * JOY_FACTOR;
    float y = y_axis * JOY_FACTOR;

    // メカナム用ミキシング
    float fl = y + x;   // Front-Left
    float fr = y - x;   // Front-Right
    float rl = y - x;   // Rear-Left
    float rr = y + x;   // Rear-Right
    // 正規化：いずれかが±100を超えたら全体をスケーリング
    float maxVal = max( max(abs(fl), abs(fr)), max(abs(rl), abs(rr)) );
    if (maxVal > JOYPAD_MAX_VALUE) {
        float scale = JOYPAD_MAX_VALUE / maxVal;
        fl *= scale;  fr *= scale;
        rl *= scale;  rr *= scale;
    }
    // PWM化（255/100 ≒ 2.55f）
    uint8_t pwmFL = abs(fl) * 2.0f;
    uint8_t pwmFR = abs(fr) * 2.0f;
    uint8_t pwmRL = abs(rl) * 2.0f;
    uint8_t pwmRR = abs(rr) * 2.0f;
        // キャリブレーション適用
    pwmFL = apply_motor_calibration(pwmFL, FL_POWER_RATIO);
    pwmFR = apply_motor_calibration(pwmFR, FR_POWER_RATIO);
    pwmRL = apply_motor_calibration(pwmRL, RL_POWER_RATIO);
    pwmRR = apply_motor_calibration(pwmRR, RR_POWER_RATIO);

    Serial.printf("# Dabble JoyPad Control: FL=%d, FR=%d, RL=%d, RR=%d\n", pwmFL, pwmFR, pwmRL, pwmRR);

    // モーター出力（chFwd, chRev, PWM値, forward?）
    setMotorPWM(TIRE_FL1, TIRE_FL2, pwmFL, fl >= 0);  // Front-Left
    setMotorPWM(TIRE_FR1, TIRE_FR2, pwmFR, fr >= 0);  // Front-Right
    setMotorPWM(TIRE_RL1, TIRE_RL2, pwmRL, rl >= 0);  // Rear-Left
    setMotorPWM(TIRE_RR1, TIRE_RR2, pwmRR, rr >= 0);  // Rear-Right
}
// chFwd: 前進用の LEDC チャネル（例: TIRE_FL1）
// chRev: 後進用の LEDC チャネル（例: TIRE_FL2）
// speed: 0〜255 の PWM 値
// forward: true→前進, false→後進
static void setMotorPWM(uint8_t chFwd, uint8_t chRev, uint8_t speed, bool forward)
{
    if (forward) {
        // 前進
        ledcWrite(chFwd, speed);
        ledcWrite(chRev, 0);
    } else {
        // 後進
        ledcWrite(chFwd, 0);
        ledcWrite(chRev, speed);
    }
}
//=========================================================
//  Apply Motor Calibration
//=========================================================
static uint8_t apply_motor_calibration(uint8_t base_speed, float ratio)
{
    float calibrated_speed = base_speed * ratio;
    return (uint8_t)constrain(calibrated_speed, 0, 255);
}