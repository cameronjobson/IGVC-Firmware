// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <uni.h>
#include "controller_callbacks.h"
#include <Wire.h>
#include <bits/stdc++.h>
#include <ESP32Servo.h>
#include <driver/uart.h> // Critical dependency for ESP32 UART functions
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/custom_transport.h>


#define ONBOARD_LED_PIN 2 // LED pin
#define IN1 34 // right motor pins
#define IN2 35
#define IN3 36 // left motor pins
#define IN4 39            
#define IN5 4 // launch motor pins
#define IN6 5
#define mROS_RX 16
#define mROS_TX 17
//const uint8_t LINE_FOLLOW_PINS[] = {36, 35, 34, 14, 13, 39, 33, 32}; // line sensor pins
//#define FRONT_IR_PIN 25 // ir pins
//#define RIGHT_IR_PIN 26 // ir pins
//#define RIGHT_IR_PIN 26 
//#define APDS9960_INT_PIN 0 // color pins
//#define I2C_SDA_PIN 21
//#define I2C_SCL_PIN 22
#define ANGLE_SERVO_PIN 23 // angle servo pin

const uint8_t TOP_MOTOR_SPEED = 255;
const char* const MODES[] = {"Manual", "Color automation", "Wall automation", "Line automation"};
const short MANUAL = 0;
const uint8_t COLOR_AUTOMATION = 1;
const uint8_t WALL_AUTOMATION = 2;
const uint8_t LINE_AUTOMATION = 3;
int currentMode = 0; // current mode for robot
extern ControllerPtr myControllers[BP32_MAX_GAMEPADS]; // controller

//-----------------------------------------------------------------------------------------------//
//-----------------------------------------<< HELPERS >>-----------------------------------------//
//-----------------------------------------------------------------------------------------------//

/*
 * Cleans terminal to debug easier.
 */
void cleanTerminal() {
    for (int i = 0; i < 25; i++) {
        Console.println();
    }
}

/*
 * Alternative way to clean easier.
 * Removes current line.
 */
void cleanTerminalAlt() {
    Console.print('\r');
    Console.print("\x1B[2K");
}

/*
 * Sets cursor back to beginning for next clear call.
 */
void writeTerminalAlt() {
    Console.write('\r');
}

/*
 * Sets current mode based on controller input.
 * @param ctl pointer to controller
 */
void setMode(ControllerPtr ctl) {
    if (ctl->b()) {
        currentMode = MANUAL; // Manual mode
    } else if (ctl->a()) {
        currentMode = COLOR_AUTOMATION; // Color mode
    } else if (ctl->x()) {
        currentMode = WALL_AUTOMATION; // Wall mode
    } else if (ctl->y()) {
        currentMode = LINE_AUTOMATION; // Line mode
    }
}

//-----------------------------------------------------------------------------------------------//
//--------------------------------------<< MICRO-ROS Serial>>-------------------------------------//
//-----------------------------------------------------------------------------------------------//

bool my_open(struct uxrCustomTransport * transport)
{
    return true;
}

bool my_close(struct uxrCustomTransport * transport)
{
    return true;
}

size_t my_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err)
{
    uart_port_t port = *(uart_port_t *)transport->args;
    int written = uart_write_bytes(port, buf, len);
    return (written < 0) ? 0 : (size_t)written;
}

size_t my_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err)
{
    uart_port_t port = *(uart_port_t *)transport->args;
    int read = uart_read_bytes(port, buf, len, timeout / portTICK_PERIOD_MS);
    return (read < 0) ? 0 : (size_t)read;
}


//-----------------------------------------------------------------------------------------------//
//--------------------------------------<< MOTOR MOVEMENT >>-------------------------------------//
//-----------------------------------------------------------------------------------------------//

const uint16_t MAX_JOYSTICK_INPUT = 512;

void moveMotorsHelper(int leftSpeedForward, int leftSpeedBackward, int rightSpeedForward, int rightSpeedBackward);

/*
 * Sets up the motor pins as outputs.
 */
void motorSetup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(IN5, OUTPUT);
    pinMode(IN6, OUTPUT);
}

/*
 * Handles the movement of the robot.
 * Left joystick controls left motor and right joystick controls right motor.
 * @param ctl pointer to controller
 */
void moveMotorsA(ControllerPtr ctl) {
    int lY = ctl->axisY(); // Left joystick Y axis
    int aLY = abs(lY) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // Adjusted Left joystick Y axis for speed input [0, TOP_MOTOR_SPEED]
    int rY = ctl->axisRY();// Right joystick Y axis
    int aRY = abs(rY) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // Adjusted Right joystick Y axis for speed input [0, TOP_MOTOR_SPEED]

    bool lForward = (lY <= -200); // Determine if left joystick is moving forward
    bool lBackward = (lY >= 200); // Determine if left joystick is moving backward
    bool rForward = (rY <= -200);
    bool rBackward = (rY >= 200);

    if (lForward) { // if left joystick is forward, move motor forward at adjusted speed
        analogWrite(IN1, aLY);
        analogWrite(IN2, 0);
    } else if (lBackward) { // else, move motor backwards at adjusted speed
        analogWrite(IN1, 0);
        analogWrite(IN2, aLY);
    } else {
        analogWrite(IN1, 0);
        analogWrite(IN2, 0);
    }

    if (rForward) { // if right joystick is forward, move motor forward at adjusted speed
        analogWrite(IN3, aRY);
        analogWrite(IN4, 0);
    } else if (rBackward) { // else, move motor backwards at adjusted speed
        analogWrite(IN3, 0);
        analogWrite(IN4, aRY);
    } else {
        analogWrite(IN3, 0);
        analogWrite(IN4, 0);
    }
}

/*
 * Handles the movement of the robot differently than main.
 * Left joystick controls both motors and the right joystick turns it to one direction.
 * @param ctl pointer to controller
 */
void moveMotorsB(ControllerPtr ctl) {
    int lY = ctl->axisY(); // left joystick y-value
    int aLY = abs(lY) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // adjusted value
    int rX = ctl->axisRX(); // right joystick x-value
    int aRX = abs(rX) * TOP_MOTOR_SPEED / MAX_JOYSTICK_INPUT; // adjusted value
    bool lT = ctl->l1();
    bool rT = ctl->r1(); // not used most of the time. Ask mentor if this will affect performance

    int motorAdjustment = max(0, aLY - aRX); // adjusts motors for either side
    bool forward = (lY <= -200);
    bool backward = (lY >= 200);
    bool left = (rX <= -30);
    bool right = (rX >= 30);

    if (forward) {
        if (left) {
            moveMotorsHelper(motorAdjustment,0, aLY, 0);
        } else if (right) {
            moveMotorsHelper(aLY, 0, motorAdjustment, 0);
        } else {
            moveMotorsHelper(aLY, 0, aLY, 0);
        }
    } else if (backward) {
        if (left) {
            moveMotorsHelper(0, motorAdjustment, 0, aLY);
        } else if (right) {
            moveMotorsHelper(0, aLY, 0, motorAdjustment);
        } else {
            moveMotorsHelper(0, aLY, 0, aLY);
        }
    } else { // left joystick netural (within -200, 200)
        if (lT) { // spin counter-clockwise
            moveMotorsHelper(0, TOP_MOTOR_SPEED, TOP_MOTOR_SPEED, 0);
        } else if (rT) { // spin clockwise
            moveMotorsHelper(TOP_MOTOR_SPEED, 0, 0, TOP_MOTOR_SPEED);
        } else { // standstill
            moveMotorsHelper(0, 0, 0, 0);
        }
    }
}

/*
 * Prints all button values for the controller.
 * @param ctl pointer to controller
 */
void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "DPAD: %2d A: %2d B: %2d X: %2d Y: %2d LX: %4d LY: %4d RX: %4d RY: %4d L1: %2d R1: %2d L2: %2d R2: %2d\n",
        ctl->dpad(),        // D-pad
        ctl->a(),           // Letter buttons
        ctl->b(),
        ctl->x(),
        ctl->y(),
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->l1(),           // Bumpers
        ctl->r1(),
        ctl->l2(),
        ctl->r2()
    );
}

/*
 * Helper to simplify motor control.
 * Takes in four values: left speed f/b, and right speed f,b
 * @param leftSpeedForward left motor speed forward
 * @param leftSpeedBackward left motor speed backwards
 * @param rightSpeedForward right motor speed forward
 * @param rightSpeedBackward right motor speed backwards
 */
void moveMotorsHelper(int leftSpeedForward, int leftSpeedBackward, int rightSpeedForward, int rightSpeedBackward) {
    analogWrite(IN1, leftSpeedForward);
    analogWrite(IN2, leftSpeedBackward);
    analogWrite(IN3, rightSpeedForward);
    analogWrite(IN4, rightSpeedBackward);
}

//-----------------------------------------------------------------------------------------------//
//--------------------------------------<< MOTOR LAUNCHER >>-------------------------------------//
//-----------------------------------------------------------------------------------------------//

bool launchMotor = false;
bool launchMotorRamped = false;

void rampUpLaunchMotor();
void rampDownLaunchMotor();
void moveLaunchMotorHelper(int speedForward, int speedBackward);

/* 
 * Checks to see if launch motors need to be ramped up/down or neither.
 * @param ctl pointer to control
 */
void checkLaunchMotor(ControllerPtr ctl) {
    launchMotor = (ctl->r2()) ? true : false;
    if (launchMotor && !launchMotorRamped) {
        rampUpLaunchMotor();
        launchMotorRamped = true;
    } else if (!launchMotor && launchMotorRamped) {
        rampDownLaunchMotor();
        launchMotorRamped = false;
    }
}

/*
 * Ramp launch motors up so it doens't reach max speed instantly.
 */
void rampUpLaunchMotor() {
    int initialSpeed = 55;
    for (int i = initialSpeed; i <= 200; i++) {
        moveLaunchMotorHelper(i, 0);
        delay(5);
    }
}

/*
 * Ramp down launch motors so it doesn't stop instantly.
 */
void rampDownLaunchMotor() {
    int initialSpeed = 255;
    for (int i = initialSpeed; i >= 55; i--) {
        moveLaunchMotorHelper(i, 0);
        delay(5);
    }
    moveLaunchMotorHelper(0, 0);
}

/*
 * Helper to simplify launch motor control.
 * @param speedForward motor speed forward
 * @param speedBackward motor speed backward
 */
void moveLaunchMotorHelper(int speedForward, int speedBackward) {
    analogWrite(IN5, speedForward);
    analogWrite(IN6, speedBackward);
}

//-----------------------------------------------------------------------------------------------//
//------------------------------------------<< SERVO >>------------------------------------------//
//-----------------------------------------------------------------------------------------------//

const int ANGLE_MIN = 0;
const int ANGLE_MAX = 60;
const int ANGLE_CLOSED = 0;
const int ANGLE_OPEN = 60;
Servo angleServo;
Servo collectionServo;
int angle = 0; // current angle of angleServo

/*
 * Setups up the angle and collection servo.
 */
void servoSetup() {
    angleServo.attach(ANGLE_SERVO_PIN);
    angle = angleServo.read(); // current angle
}


/*
 * Handles the movement of the angle servo.
 * Dpad up and down control the angle of the servo. (N U D R L) -> (0, 1, 2, 4, 8)
 * @param ctl pointer to controller
 */
void moveAngleServo(ControllerPtr ctl, int &angle) {
    uint8_t dpad = ctl->dpad();
    if (dpad == 1) {
        angleServo.write(ANGLE_OPEN);
    } else if (dpad == 2) {
        angleServo.write(ANGLE_CLOSED);
    }
}

// later convert all these into two using a multiplier

//-----------------------------------------------------------------------------------------------//
//------------------------------------------<< SETUP >>------------------------------------------//
//-----------------------------------------------------------------------------------------------//

void setup() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); 
    esp_log_level_set("gpio", ESP_LOG_ERROR); // Suppress info log spam from gpio_isr_service
    uni_bt_allowlist_set_enabled(true);
    Serial.begin(115200);
    pinMode(ONBOARD_LED_PIN, OUTPUT); // Setup LED pin
    motorSetup(); // Setup motor pins
    servoSetup(); // Setup Servos


}

//-----------------------------------------------------------------------------------------------//
//-------------------------------------------<< LOOP >>------------------------------------------//
//-----------------------------------------------------------------------------------------------//

void loop() {
    vTaskDelay(1); // Ensures WDT does not get triggered when no controller is connected
    BP32.update(); // Is this needed inside for loop?
    bool debug = false;
    bool automate = true;

   
    // Loop code will only run if controller is connected.
    for (auto myController : myControllers) { // Only execute code when controller is connected
        if (myController && myController->isConnected() && myController->hasData()) {
            // wifiConnect(); // Handle WiFi connections
            setMode(myController); // Set current mode based on controller input
                                   // If to inefficient, create loops within each case and handle exiting internally
            // zr shooting sequence: spin launch motor, delay, spin the servo to allow ball in.
            // zl intake sequence: spins motor to intake balls
            // d pad to pivot up and down shooter
            //cleanTerminal();
            digitalWrite(ONBOARD_LED_PIN, HIGH); // Turn on LED when controller is connected
            // if (debug) Console.printf("Current mode: %s ------- ", MODES[currentMode]);
            // switch (currentMode) {
            //     case MANUAL: // Manual mode
            //         if (sampled) resetColorVariables();
            //         if (automate) {
            //             checkLaunchMotor(myController);
            //             // moveMotorsA(myController);
            //             moveMotorsB(myController);
            //             moveAngleServo(myController, angle);
            //         }
            //         if (debug) dumpGamepad(myController);
            //         break;
            //     case COLOR_AUTOMATION: // Color mode
            //         updateColor();
            //         if (automate) colorAutomation(myController);
            //         if (debug) colorDebug();
            //         break;
            //     case WALL_AUTOMATION: // Wall mode
            //         updateIR();
            //         if (automate) wallAutomationB();
            //         if (debug) wallDebugB();
            //         break;
            //     case LINE_AUTOMATION: // Line follow mode
            //         if (!lineCalibrated) startCalibration();
            //         updateLine();
            //         if (automate) lineAutomationB();
            //         if (debug) lineDebug();
            //         break;
            // }

            if (debug && !automate) delay(100);
            digitalWrite(ONBOARD_LED_PIN, LOW); // Turn off LED when done
        }
    }
}