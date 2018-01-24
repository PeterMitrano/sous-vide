#include <Arduino.h>
#include <Wire.h>

#include "sous_vide.h"

Event checkForEvent() {
  static int sw1_c= 0;
  static int sw2_c= 0;

  Event event;
  if (!digitalRead(SW1)) {
    sw1_c++;
  }
  else if (digitalRead(SW1)) {
    if (sw1_c > 1) {
      event.valid = true;
      event.type = SW1_PRESS;
    }
    sw1_c = 0;
  }

  if (!digitalRead(SW2)) {
    sw2_c++;
  }
  else if (digitalRead(SW2)) {
    if (sw2_c > 1) {
      event.valid = true;
      event.type = SW2_PRESS;
    }
    sw2_c = 0;
  }
  return event;
}

void setup() {
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
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

  Serial.begin(9600);

  // set initial cooking_duration
  t = millis();
}

void loop() {
  static unsigned long last_event_time = 0ul;
  unsigned long now = millis();
  if (now - last_event_time > 10) {
    last_event_time = now;
    digitalWrite(POT, HIGH);
    digitalWrite(THRM, LOW);
    static unsigned int potentiometer_value = 0;
    potentiometer_value = 0.8 * potentiometer_value + 0.2 * analogRead(A0);
    double pot_as_temp = potToTemp(potentiometer_value);
    digitalWrite(POT, LOW);
    digitalWrite(THRM, HIGH);
    unsigned int thermistor_value = analogRead(A0);
    double current_temp = analogToTemp(thermistor_value);
    digitalWrite(POT, LOW);
    digitalWrite(THRM, LOW);

    Event evt = checkForEvent();

    switch (global_state) {
      case CHANGE_TEMP:
        digitalWrite(RELAY, LOW);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, LOW);

        if (evt.valid && evt.type == EventType::SW1_PRESS) {
          global_state = CHANGE_TIME;
        }

        setpoint_temp_f = pot_as_temp;

        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.setCursor(0, 1);
        lcd.print(String(pot_as_temp) + String("     "));

#ifdef DEBUG
        Serial.print("change temp. ");
        Serial.print("temp = ");
        Serial.println(pot_as_temp);
#endif
        break;

      case CHANGE_TIME:
        digitalWrite(RELAY, LOW);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, LOW);
        if (evt.valid && evt.type == EventType::SW1_PRESS) {
          global_state = HEATING;
          start_cook_time = millis();
        }
        else if (evt.valid && evt.type == EventType::SW2_PRESS) {
          global_state = CHANGE_TEMP;
        }

        cooking_duration = potToTime(potentiometer_value);
        lcd.setCursor(0, 0);
        lcd.print("Time: ");
        lcd.setCursor(0, 1);
        {
          // pad minutes to two characters (ex: 05)
          String min_str("00");
          uint8_t minutes = cooking_duration/60%60;
          if (minutes < 10) {
            min_str[1] = String(minutes)[0];
          }
          else {
            min_str = String(minutes);
          }
          String duration_string = String(cooking_duration/(60*60)) + ":" + min_str + String("       ");
          lcd.print(duration_string);
#ifdef DEBUG
          Serial.print("change time. ");
          Serial.print("time = ");
          Serial.println(duration_string);
#endif
        }
        break;

      case HEATING:
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, HIGH);
        if (evt.valid && evt.type == EventType::SW2_PRESS) {
          global_state = PAUSED;
        }

        lcd.setCursor(0, 0);
        lcd.print("status... ");
        lcd.setCursor(0, 1);
        lcd.print(String(current_temp));
        lcd.print("    ");
        lcd.print(String(millis() - start_cook_time));

        control_heater(current_temp);

        if (millis() - start_cook_time >= cooking_duration) {
          digitalWrite(BLUE_LED, HIGH);
          digitalWrite(RED_LED, LOW);
          global_state = FINISHED;
        }

#ifdef DEBUG
        Serial.println("heating.");
#endif
        break;

      case PAUSED:
        digitalWrite(RELAY, LOW);
        if (evt.valid && evt.type == EventType::SW1_PRESS) {
          global_state = HEATING;
        }
        else if (evt.valid && evt.type == EventType::SW2_PRESS) {
          global_state = FINISHED;
        }

        lcd.setCursor(0, 0);
        lcd.clear();
        lcd.print("paused.");

#ifdef DEBUG
        Serial.println("paused.");
#endif
        break;

      case FINISHED:
#ifdef DEBUG
        Serial.println("finished.");
#endif
        // [[fallthrough]]
      default:
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, LOW);
        digitalWrite(RELAY, LOW);
        break;
    }
  }

  // Software PWM of relay
  static unsigned long pwm_t0 = 0ul;
  static constexpr double pwm_period_ms = 10000;
  digitalWrite(RELAY, (now - pwm_t0 < duty_cycle * pwm_period_ms));
  if (now > pwm_t0 && now - pwm_t0 > pwm_period_ms) {
    pwm_t0 = now;
  }
  //static unsigned long tt = millis();
  //static bool on = true;
  //if (millis() - tt > 1000) {
    //on = !on;
    //tt = millis();
  //}
  //Serial.print("RELAY: ");
  //Serial.println(on);
  //digitalWrite(RELAY, on);
}

/**
 * @return temp in fahrenheit
 */
double analogToTemp(unsigned int thermistor_value) {
  static constexpr int ADC_TOP = 1023;
  static constexpr int BETA = 3977;
  static constexpr double LNA = -4.12858298874828;
  static constexpr int R_REF = 1100;
  double temp = (R_REF * thermistor_value) / (ADC_TOP - thermistor_value);
  temp = BETA / (log(temp) - LNA);
  temp = temp - 273.15;
  return temp;
}

/**
 * @return temp in fahrenheit
 */
double potToTemp(unsigned int potentiometer_value) {
  static constexpr int minimum_temp = 100; // degrees F
  static constexpr int maximum_temp = 170; // degrees F
  Serial.println(potentiometer_value);
  return map(potentiometer_value, 3, 938, minimum_temp, maximum_temp);
}

/**
 * @return time in seconds
 */
unsigned int potToTime(unsigned int potentiometer_value) {
  static constexpr unsigned int minimum_time = 5; // X minutes
  static constexpr unsigned int maximum_time = 4 * 60; // X hours
  static constexpr unsigned int intervals = (maximum_time - minimum_time) / 5;
  Serial.println(potentiometer_value);
  return (minimum_time + 5 * map(potentiometer_value, 3, 930, 0, intervals)) * 60;
}

/**
 * For a given desired teperature, there is a duty cycle that acheives it.
 * We should use a reasonable PWM period fast enough to minimize fluctuations.
 * PID will be used to guide the duty cycle.
 *
 */
void control_heater(double current_temp) {
  static constexpr double kP = 0.0;
  static constexpr double kI = 0.0;
  static constexpr double kD = 0.0;
  static double integral = 0;

  double error = setpoint_temp_f - current_temp;

  static double last_error = error;

  integral += error;

  double derivative = error - last_error;
  duty_cycle = kP * error + kI * integral + kD * derivative;

  // enforce limit on duty cycle
  duty_cycle = fmax(fmin(duty_cycle, 1), 0);

  last_error = error;
}
