#include <Arduino.h>
#include <TRSensors.h>
#include <motor.h>
#include <definitions.h>

 
float Kp = 0.2; // Proportional gain
float Ki = 0.01; // Integral gain
float Kd = 0.1; // Derivative gain
 
float previous_error = 0;
float integral = 0;

Motor left_motor = Motor(PIN_DIRECTION_FWD_1, PIN_DIRECTION_REV_1, PIN_DIRECTION_PWM_1);
Motor right_motor = Motor(PIN_DIRECTION_FWD_2, PIN_DIRECTION_REV_2, PIN_DIRECTION_PWM_2);
TRSensors trs = TRSensors();

unsigned int sensorValues[NUM_SENSORS];

void encoder_handler_left_ISR() {
  left_motor.counter++;
}
 
void encoder_handler_right_ISR() {
  right_motor.counter++;
}
 
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_INTERRUPT_LEFT), encoder_handler_left_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_INTERRUPT_RIGHT), encoder_handler_right_ISR, RISING);
  for(int i = 0; i < 400; i++){
    trs.calibrate();
  }
  Serial.println("calibration done");
  delay(1000);
}
 
void loop() {
 
  unsigned int position = trs.readLine(sensorValues);
  int error = 2500 - position;
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  integral += error;
  float derivative = error - previous_error;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;
 
  int base_speed = 150;
  int left_speed = base_speed + correction;
  int right_speed = base_speed - correction;

  left_speed = constrain(left_speed, 0, 100);
  right_speed = constrain(right_speed, 0, 100);
 
  left_motor.set_direction(reverse);
  right_motor.set_direction(reverse);
 
  left_motor.set_speed(left_speed);
  right_motor.set_speed(right_speed);
 
  // Print debug information
  // Serial.print("Position: ");
  // Serial.print(position);
  // Serial.print("\tError: ");
  // Serial.print(error);
  // Serial.print("\tCorrection: ");
  // Serial.println(correction);
 
  delay(10);
  left_motor.counter = 0, right_motor.counter = 0;
}