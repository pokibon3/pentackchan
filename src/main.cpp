//=========================================================
//  main.cpp :    pentackchan-rover with IR Remote + Dabble JoyPad
//  History     : V1.1  2025-06-07 IR Remote + Dabble JoyPad control
//=========================================================
#include <Arduino.h>
#include <DabbleESP32.h>
#include "pentackchan.hpp"
#include "ir_remote_control.hpp"
#include "remote_control.hpp"

//=========================================================
//  Motor Power Calibration Constants
//=========================================================
#define FR_POWER_RATIO  1.10f   // Front Right motor power ratio
#define FL_POWER_RATIO  1.10f   // Front Left motor power ratio  
#define RR_POWER_RATIO  1.00f   // Rear Right motor power ratio
#define RL_POWER_RATIO  1.00f   // Rear Left motor power ratio

//=========================================================
//  JoyPad Parameters
//=========================================================
#define JOYPAD_DEADZONE     20  // Deadzone for joystick
#define JOYPAD_MAX_VALUE   100  // Maximum joystick value
#define SPEED_FACTOR       2.55f // Conversion factor (255/100)

//=========================================================
//  Control Mode
//=========================================================
enum ControlMode {
    CONTROL_NONE = 0,
    CONTROL_IR,
    CONTROL_DABBLE
};

//=========================================================
//  prototypes definition
//=========================================================
static uint8_t remote_control(void);
static void dabble_joypad_control(void);
static uint8_t apply_motor_calibration(uint8_t base_speed, float ratio);
static void execute_motor_command(uint8_t cmd);

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
    Dabble.begin("RumiCar_Rover");  // Bluetooth device name
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
            if (millis() - last_activity_time > 500) {
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
    int x_axis = GamePad.getXaxisData();  // -100 to +100 (left/right)
    int y_axis = GamePad.getYaxisData();  // -100 to +100 (forward/backward)
    
    // Check for emergency stop button
    if (GamePad.isPressed(0)) {
        remote_stop(SPEED_MAX);
        Serial.println("# Emergency Stop!");
        return;
    }
    
    // Check for preset movement buttons
    if (GamePad.isPressed(1)) {         // Triangle - Forward
        execute_motor_command('F');
        return;
    } else if (GamePad.isPressed(2)) {  // Square - Left
        execute_motor_command('<');
        return;
    } else if (GamePad.isPressed(3)) {  // Circle - Right  
        execute_motor_command('>');
        return;
    } else if (GamePad.isPressed(4)) {  // Cross - Backward
        execute_motor_command('B');
        return;
    } else if (GamePad.isPressed(5)) {  // Start - Stop
        execute_motor_command('Q');
        return;
    } else if (GamePad.isPressed(6)) {  // Select - Turn Left
        execute_motor_command('A');
        return;
    } else if (GamePad.isPressed(7)) {  // Turn Right  
        execute_motor_command('C');
        return;
    }
    
    // Apply deadzone to joystick
    if (abs(x_axis) < JOYPAD_DEADZONE) x_axis = 0;
    if (abs(y_axis) < JOYPAD_DEADZONE) y_axis = 0;
    
    // If no significant joystick movement, stop
    if (x_axis == 0 && y_axis == 0) {
        remote_stop(SPEED_MAX);
        return;
    }
    
    // Calculate motor speeds based on joystick input
    // Tank drive: left stick Y = forward/backward, left stick X = turn
    float left_speed = y_axis + x_axis;
    float right_speed = y_axis - x_axis;
    
    // Limit speeds to valid range
    left_speed = constrain(left_speed, -JOYPAD_MAX_VALUE, JOYPAD_MAX_VALUE);
    right_speed = constrain(right_speed, -JOYPAD_MAX_VALUE, JOYPAD_MAX_VALUE);
    
    // Convert to PWM values (0-255)
    uint8_t left_pwm = abs(left_speed) * SPEED_FACTOR;
    uint8_t right_pwm = abs(right_speed) * SPEED_FACTOR;
    
    // Apply motor calibration
    uint8_t fl_speed = apply_motor_calibration(left_pwm, FL_POWER_RATIO);
    uint8_t fr_speed = apply_motor_calibration(right_pwm, FR_POWER_RATIO);
    uint8_t rl_speed = apply_motor_calibration(left_pwm, RL_POWER_RATIO);
    uint8_t rr_speed = apply_motor_calibration(right_pwm, RR_POWER_RATIO);
    
    // Control motors based on direction
    if (left_speed > JOYPAD_DEADZONE) {
        // Left side forward
        ledcWrite(TIRE_FL1, fl_speed);
        ledcWrite(TIRE_FL2, 0);
        ledcWrite(TIRE_RL1, rl_speed);
        ledcWrite(TIRE_RL2, 0);
    } else if (left_speed < -JOYPAD_DEADZONE) {
        // Left side backward
        ledcWrite(TIRE_FL1, 0);
        ledcWrite(TIRE_FL2, fl_speed);
        ledcWrite(TIRE_RL1, 0);
        ledcWrite(TIRE_RL2, rl_speed);
    } else {
        // Left side stop
        ledcWrite(TIRE_FL1, 0);
        ledcWrite(TIRE_FL2, 0);
        ledcWrite(TIRE_RL1, 0);
        ledcWrite(TIRE_RL2, 0);
    }
    
    if (right_speed > JOYPAD_DEADZONE) {
        // Right side forward
        ledcWrite(TIRE_FR1, fr_speed);
        ledcWrite(TIRE_FR2, 0);
        ledcWrite(TIRE_RR1, rr_speed);
        ledcWrite(TIRE_RR2, 0);
    } else if (right_speed < -JOYPAD_DEADZONE) {
        // Right side backward
        ledcWrite(TIRE_FR1, 0);
        ledcWrite(TIRE_FR2, fr_speed);
        ledcWrite(TIRE_RR1, 0);
        ledcWrite(TIRE_RR2, rr_speed);
    } else {
        // Right side stop
        ledcWrite(TIRE_FR1, 0);
        ledcWrite(TIRE_FR2, 0);
        ledcWrite(TIRE_RR1, 0);
        ledcWrite(TIRE_RR2, 0);
    }
    
    // Debug output (uncomment if needed)
    /*
    Serial.printf("Joy: X=%d, Y=%d | Left=%.0f, Right=%.0f | PWM: FL=%d, FR=%d, RL=%d, RR=%d\n", 
                 x_axis, y_axis, left_speed, right_speed, fl_speed, fr_speed, rl_speed, rr_speed);
    */
}

//=========================================================
//  Apply Motor Calibration
//=========================================================
static uint8_t apply_motor_calibration(uint8_t base_speed, float ratio)
{
    float calibrated_speed = base_speed * ratio;
    return (uint8_t)constrain(calibrated_speed, 0, 255);
}