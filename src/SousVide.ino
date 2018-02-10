#include <Arduino.h>
#include <Wire.h>
#include <queue>

#include "sous_vide.h"

Event checkForEvent() {
  static int sw1_c= 0;
  static int sw2_c= 0;
  static const int kDebounce = 4;

  bool sw1_on = !digitalRead(SW1);
  bool sw2_on = !digitalRead(SW2);

  if (sw1_on && sw1_c <= kDebounce) {
    sw1_c++;
  }
  else if (!sw1_on) {
    if (sw1_c > kDebounce) {
      Event sw1_event;
      sw1_event.type = SW1_PRESS;
      sw1_c = 0;
      return sw1_event;
    }
    sw1_c = 0;
  }

  if (sw2_on && sw2_c <= kDebounce) {
    sw2_c++;
  }
  else if (!sw2_on) {
    if (sw2_c > kDebounce) {
      Event sw2_event;
      sw2_event.type = SW2_PRESS;
      sw2_c = 0;
      return sw2_event;
    }
    sw2_c = 0;
  }

  return Event();
}

void setup() {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  Wire.begin(DAT, CLK);
  pinMode(POT, OUTPUT);
  pinMode(THRM, OUTPUT);

  Wire.begin(DAT, CLK);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setBacklight(HIGH);
  lcd.setCursor(0, 0);
  lcd.createChar(0, degree_char);

  Serial.begin(9600);
}

void loop() {
  static unsigned long last_event_time = 0ul;
  static unsigned long last_print_time = 0ul;
  unsigned long now_ms = millis();
  unsigned long now_s = now_ms / 1000;

  // default constructed to invalid event
  static std::queue<Event> event_queue;

  // fastest loop for reading thermistor, controlling heater, and button checking
  if (now_ms - last_event_time > 5) {
    last_event_time = now_ms;

    Event evt = checkForEvent();
    if (event_queue.empty() && evt.type != EventType::NONE) {
      event_queue.push(evt);
    }

    if (state_g == HEATING) {
      digitalWrite(POT, LOW);
      digitalWrite(THRM, HIGH);
      //delayMicroseconds(100);
      unsigned int thermistor_value = analogRead(A0);
      current_temp_g = analogToTemp(thermistor_value);

      control_heater();

      // Software PWM of relay
      static unsigned long pwm_t0 = 0ul;
      static unsigned long pwm_off_time = 0ul;
      static constexpr double pwm_period_ms = 10000;
      if (now_ms - pwm_t0 >= pwm_period_ms) {
        pwm_t0 = now_ms;
        pwm_off_time = pwm_t0 + duty_cycle_g * pwm_period_ms;
      }
      digitalWrite(RELAY, now_ms <= pwm_off_time);

      if (now_s - start_cooking_time_sec_g >= cooking_duration_sec_g) {
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(RED_LED, LOW);
        lcd.clear();
        state_g = CHANGE_TEMP; // FIXME: change back to FINISHED
      }
    }
    else {
      // make sure relay is off unless in heating state
      digitalWrite(RELAY, LOW);
    }
  }
  // run the display at a mucher slower rate
  if (now_ms - last_print_time > 100) {
    last_print_time = now_ms;
    Event latest_evt;
    if (!event_queue.empty()) {
      latest_evt = event_queue.front();
      event_queue.pop();
    }

    switch (state_g) {
      case CHANGE_TEMP:
        {
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(RED_LED, LOW);

          digitalWrite(POT, HIGH);
          digitalWrite(THRM, LOW);
          static unsigned int potentiometer_value = 0;
          potentiometer_value = 0.7 * potentiometer_value + 0.3 * analogRead(A0);
          double pot_as_temp = potToTemp(potentiometer_value);

          if (latest_evt.type == EventType::SW1_PRESS) {
            lcd.clear();
            state_g = CHANGE_TIME;
            break;
          }

          setpoint_temp_fahrenheit_g = pot_as_temp;

          lcd.setCursor(0, 0);
          lcd.print("Temp: ");
          lcd.setCursor(0, 1);
          lcd.print(formatTemp(setpoint_temp_fahrenheit_g));
          lcd.write((uint8_t)0);
          break;
        }

      case CHANGE_TIME:
        {
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(RED_LED, LOW);

          digitalWrite(POT, HIGH);
          digitalWrite(THRM, LOW);
          static unsigned int potentiometer_value = 0;
          potentiometer_value = 0.7 * potentiometer_value + 0.3 * analogRead(A0);

          if (latest_evt.type == EventType::SW1_PRESS) {
            lcd.clear();
            state_g = HEATING;
            start_cooking_time_sec_g = now_s;
            break;
          }
          else if (latest_evt.type == EventType::SW2_PRESS) {
            lcd.clear();
            state_g = CHANGE_TEMP;
            break;
          }

          cooking_duration_sec_g = potToTime(potentiometer_value);
          lcd.setCursor(0, 0);
          lcd.print("Time: ");
          lcd.setCursor(0, 1);
          lcd.print(formatTime(cooking_duration_sec_g));
          break;
        }

      case HEATING:
        {
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(RED_LED, HIGH);
          if (latest_evt.type == EventType::SW2_PRESS) {
            lcd.clear();
            state_g = PAUSED;
            break;
          }

          auto time_left = cooking_duration_sec_g - (now_s - start_cooking_time_sec_g);
          if (time_left == 0) {
            lcd.clear();
            state_g = FINISHED;
          }

          lcd.setCursor(0, 0);
          lcd.print("Status");
          lcd.setCursor(0, 1);
          lcd.print(formatTemp(current_temp_g));
          lcd.write(0);
          lcd.print(" ");
          lcd.print(formatTemp(setpoint_temp_fahrenheit_g));
          lcd.write(0);
          lcd.print(" ");
          lcd.print(formatTime(time_left));
          break;
        }

      case PAUSED:
        {
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(RED_LED, HIGH);
          if (latest_evt.type == EventType::SW1_PRESS) {
            lcd.clear();
            state_g = HEATING;
            break;
          }
          else if (latest_evt.type == EventType::SW2_PRESS) {
            lcd.clear();
            state_g = FINISHED;
            break;
          }

          lcd.setCursor(0, 0);
          lcd.print("paused.");
          break;
        }

      case FINISHED:
        lcd.setCursor(0, 0);
        lcd.print("finished.");
        // [[fallthrough]]
      default:
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(RED_LED, LOW);
        break;
    }
  }
}

double analogToTemp(unsigned int thermistor_value) {
  static constexpr int ADC_TOP = 1023;
  static constexpr int BETA = 3977;
  static constexpr double LNA = -4.12858298874828;
  static constexpr int R_REF = 1100;
  double temp = (R_REF * thermistor_value) / (ADC_TOP - thermistor_value);
  temp = BETA / (log(temp) - LNA);
  temp = temp - 273.15;

  // FIXME: this is empirically derived from a set of data points
  temp = exp((thermistor_value + 1220) / 385.5);
  return temp;
}

double potToTemp(unsigned int potentiometer_value) {
  static constexpr int minimum_temp = 100; // degrees F
  static constexpr int maximum_temp = 170; // degrees F
  return map(potentiometer_value, 3, 938, minimum_temp, maximum_temp);
}

unsigned int potToTime(unsigned int potentiometer_value) {
  static constexpr unsigned int minimum_time = 30; // X minutes
  static constexpr unsigned int maximum_time = 5 * 60; // X hours
  static constexpr unsigned int intervals = (maximum_time - minimum_time) / 5;
  return (minimum_time + 5 * map(potentiometer_value, 3, 930, 0, intervals)) * 60;
}

String formatTime(unsigned long t_s) {
  unsigned long min = (t_s / 60) % 60;
  unsigned long h = t_s / 3600;
  String hr_str("00");
  if (h < 10) {
    hr_str[1] = String(h)[0];
  }
  else {
    hr_str = String(h);
  }

  String min_str("00");
  if (min < 10) {
    min_str[1] = String(min)[0];
  }
  else {
    min_str = String(min);
  }
  return hr_str + String(":") + min_str;
}

String formatTemp(unsigned int temp_fahrenheit) {
  String temp(temp_fahrenheit);
  if (temp_fahrenheit < 10) {
    temp = String("  ") + temp;
  }
  else if (temp_fahrenheit < 100) {
    temp = String(" ") + temp;
  }

  return temp;
}

void control_heater() {
  static constexpr double kP = 0.18;
  static constexpr double kI = 0.00001;
  static constexpr double kFF = 0.002;
  static double integral = 0;

  double error = setpoint_temp_fahrenheit_g - current_temp_g;

  static double last_error = error;

  integral += error;

  if ((error > 0 && last_error < 0) || (error < 0 && last_error > 0)) {
    integral = 0;
  }

  double derivative = error - last_error;
  duty_cycle_g = kP * error + kI * integral + kFF * setpoint_temp_fahrenheit_g;
#ifdef DEBUG
  Serial.print(integral);
  Serial.print(", ");
  Serial.println(duty_cycle_g);
#endif

  // enforce limit on duty cycle
  duty_cycle_g = fmax(fmin(duty_cycle_g, 1.f), 0.f);

  last_error = error;
}
