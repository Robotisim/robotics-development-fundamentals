#include "MotorControl.h"

MotorControl::MotorControl(int motorA_pwm, int motorA_in1, int motorA_in2,
                           int motorB_pwm, int motorB_in3, int motorB_in4,
                           int pwm_channelA, int pwm_channelB,
                           int pwm_freq, int pwm_res)
: _motorA_pwm(motorA_pwm), _motorA_in1(motorA_in1), _motorA_in2(motorA_in2),
  _motorB_pwm(motorB_pwm), _motorB_in3(motorB_in3), _motorB_in4(motorB_in4),
  _pwm_channelA(pwm_channelA), _pwm_channelB(pwm_channelB),
  _pwm_frequency(pwm_freq), _pwm_resolution(pwm_res)
{}

void MotorControl::begin() {
    // Set motor direction pins as outputs
    pinMode(_motorA_in1, OUTPUT);
    pinMode(_motorA_in2, OUTPUT);
    pinMode(_motorB_in3, OUTPUT);
    pinMode(_motorB_in4, OUTPUT);

    // Setup LEDC PWM channels
    ledcSetup(_pwm_channelA, _pwm_frequency, _pwm_resolution);
    ledcSetup(_pwm_channelB, _pwm_frequency, _pwm_resolution);

    // Attach LEDC channels to corresponding PWM pins
    ledcAttachPin(_motorA_pwm, _pwm_channelA);
    ledcAttachPin(_motorB_pwm, _pwm_channelB);

    // Ensure motors are stopped initially
    stop();
}

void MotorControl::stop() {
    ledcWrite(_pwm_channelA, 0);
    ledcWrite(_pwm_channelB, 0);
    digitalWrite(_motorA_in1, LOW);
    digitalWrite(_motorA_in2, LOW);
    digitalWrite(_motorB_in3, LOW);
    digitalWrite(_motorB_in4, LOW);
}

void MotorControl::forward(uint8_t speed) {
    // For forward motion, drive both motors forward:
    digitalWrite(_motorA_in1, HIGH);
    digitalWrite(_motorA_in2, LOW);
    digitalWrite(_motorB_in3, HIGH);
    digitalWrite(_motorB_in4, LOW);
    ledcWrite(_pwm_channelA, speed);
    ledcWrite(_pwm_channelB, speed);
}

void MotorControl::backward(uint8_t speed) {
    // For backward motion, reverse the polarity on both motors:
    digitalWrite(_motorA_in1, LOW);
    digitalWrite(_motorA_in2, HIGH);
    digitalWrite(_motorB_in3, LOW);
    digitalWrite(_motorB_in4, HIGH);
    ledcWrite(_pwm_channelA, speed);
    ledcWrite(_pwm_channelB, speed);
}

void MotorControl::turnLeft(uint8_t speed) {
    // For a left turn, stop (or slow) the left motor, run right motor forward:
    digitalWrite(_motorA_in1, LOW);
    digitalWrite(_motorA_in2, LOW);
    digitalWrite(_motorB_in3, HIGH);
    digitalWrite(_motorB_in4, LOW);
    ledcWrite(_pwm_channelA, 0);
    ledcWrite(_pwm_channelB, speed);
}

void MotorControl::turnRight(uint8_t speed) {
    // For a right turn, run left motor forward, stop right motor:
    digitalWrite(_motorA_in1, HIGH);
    digitalWrite(_motorA_in2, LOW);
    digitalWrite(_motorB_in3, LOW);
    digitalWrite(_motorB_in4, LOW);
    ledcWrite(_pwm_channelA, speed);
    ledcWrite(_pwm_channelB, 0);
}
