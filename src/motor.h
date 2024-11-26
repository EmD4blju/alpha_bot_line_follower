#include <Arduino.h>
#include <definitions.h>

enum Motion { forward, reverse, disabled };
class Motor {
  private:
    int PIN_FWD;
    int PIN_REV;
    int PIN_PWM;
  public:
    int counter;
    int current_speed = 0;
    Motor(int PIN_FWD, int PIN_REV, int PIN_PWM) {
      this->counter = 0;
      this->PIN_FWD = PIN_FWD;
      this->PIN_REV = PIN_REV;
      this->PIN_PWM = PIN_PWM;
      pinMode(PIN_FWD, OUTPUT);
      pinMode(PIN_REV, OUTPUT);
      pinMode(PIN_PWM, OUTPUT);
      digitalWrite(PIN_FWD, LOW);
      digitalWrite(PIN_REV, LOW);
      analogWrite(PIN_PWM, 0);
    }
 
    void set_direction(Motion motion) {
      switch (motion) {
        case forward:
          digitalWrite(PIN_REV, LOW);
          delay(DIRECTION_SWITCH_DELAY);
          digitalWrite(PIN_FWD, HIGH);
          break;
        case reverse:
          digitalWrite(PIN_FWD, LOW);
          delay(DIRECTION_SWITCH_DELAY);
          digitalWrite(PIN_REV, HIGH);
          break;
        case disabled:
          digitalWrite(PIN_FWD, LOW);
          digitalWrite(PIN_REV, LOW);
          break;
      }
    }
 
    void set_speed(int x) {
      if (x < 0 || x > 255) {
        current_speed = 0;
        analogWrite(PIN_PWM, 0);
      } else {
        current_speed = x;
        analogWrite(PIN_PWM, x);
      }
    }
 
    void stop() {
      digitalWrite(PIN_FWD, LOW);
      digitalWrite(PIN_REV, LOW);
      analogWrite(PIN_PWM, 0);
    }
};