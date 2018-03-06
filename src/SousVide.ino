#include <Arduino.h>
#include <Wire.h>
#include <queue>

#include "sous_vide.h"
#include "server.h"

// global variables
int current_temp_g = 0;
int setpoint_temp_fahrenheit_g = 0;
int time_left_sec_g = 0;
std::deque<int> temps = {};
unsigned long start_cooking_time_sec_g = 0ul;
unsigned int cooking_duration_sec_g = 0;
double duty_cycle_g = 0;
unsigned int state_g = SCAN_SSID;
Adafruit_LiquidCrystal lcd(0);
byte degree_char[8] = {0b00110, 0b01001, 0b01001, 0b00110, 0b00000, 0b00000, 0b00000, 0b00000};

ESP8266WebServer server(80);

Event checkForEvent() {
  static int sw1_c= 0;
  static int sw2_c= 0;
  static const int kDebounce = 3;
  static const int kLongPress = 9;

  bool sw1_on = !digitalRead(SW1);
  bool sw2_on = !digitalRead(SW2);

  if (sw1_on && sw1_c <= kLongPress) {
    sw1_c++;
  }
  else if (!sw1_on) {
    if (sw1_c > kLongPress) {
      Event sw1_event;
      sw1_event.type = SW1_LONG_PRESS;
      sw1_c = 0;
      return sw1_event;
    }
    else if (sw1_c > kDebounce) {
      Event sw1_event;
      sw1_event.type = SW1_PRESS;
      sw1_c = 0;
      return sw1_event;
    }
    sw1_c = 0;
  }

  if (sw2_on && sw2_c <= kLongPress) {
    sw2_c++;
  }
  else if (!sw2_on) {
    if (sw2_c > kLongPress) {
      Event sw2_event;
      sw2_event.type = SW2_LONG_PRESS;
      sw2_c = 0;
      return sw2_event;
    }
    else if (sw2_c > kDebounce) {
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

  lcd.begin(16, 2);
  lcd.setBacklight(HIGH);
  lcd.createChar(0, degree_char);
  lcd.clear();

  Serial.begin(9600);

  WiFi.begin();

  lcd.setCursor(0, 0);
  lcd.print("Scanning...");

  // Try to connect to the last seen network. If it works, skip the wifi setup steps;
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  unsigned long start_wifi_ms = millis();
  while (!WiFi.isConnected() && millis() - start_wifi_ms < 8000) {
    delay(100);
  }

  lcd.clear();

  if (WiFi.isConnected()) {
    MDNS.begin("sous-vide");
    state_g = CHANGE_TEMP;
  }

	WiFi.mode(WIFI_STA);
	WiFi.disconnect();

  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();
}

void loop() {
  static unsigned long last_event_time = 0ul;
  static unsigned long last_print_time = 0ul;
  static unsigned long last_log_time_s = 0ul;
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

    if (state_g == HEATING || state_g == PRE_HEATING) {
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

    }
    else {
      // make sure relay is off unless in heating state
      digitalWrite(RELAY, LOW);
    }
  }
  // run the display at a mucher slower rate
  if (now_ms - last_print_time > 100) {
    server.handleClient();

    last_print_time = now_ms;
    Event latest_evt;
    if (!event_queue.empty()) {
      latest_evt = event_queue.front();
      event_queue.pop();
    }

    static int num_ssid = 0;
    static String ssid;
    switch (state_g) {
      case SCAN_SSID:
        {
          lcd.setCursor(0, 0);
          lcd.print("Select a network:");
          lcd.setCursor(0, 1);
          lcd.print("Scanning...");
          num_ssid = WiFi.scanNetworks();
          state_g = SET_SSID;
          break;
        }
      case SET_SSID:
        {
          digitalWrite(POT, HIGH);
          digitalWrite(THRM, LOW);
          static unsigned int potentiometer_value = 0;
          potentiometer_value = 0.7 * potentiometer_value + 0.3 * analogRead(A0);
          int ssid_idx = map(potentiometer_value, 0, 1024, 0, num_ssid);

          ssid = WiFi.SSID(ssid_idx);
          char ssid_buff[16];
          snprintf(ssid_buff, 16, "%1i %-14s", ssid_idx, ssid.c_str());
          lcd.setCursor(0, 1);
          lcd.print(ssid_buff);

          if (latest_evt.type == EventType::SW1_PRESS) {
            lcd.clear();
            state_g = SET_PASSWORD;
            break;
          }
          else if (latest_evt.type == EventType::SW2_PRESS) {
            lcd.clear();
            state_g = SCAN_SSID;
            break;
          }
          break;
        }
      case SET_PASSWORD:
        {
          static int char_idx = 0;
          static char password[17];

          digitalWrite(POT, HIGH);
          digitalWrite(THRM, LOW);
          static unsigned int potentiometer_value = 0;
          potentiometer_value = 0.1 * potentiometer_value + 0.9 * analogRead(A0);
          char c = (char)map(potentiometer_value, 0, 1024, 20, 117);
          password[char_idx] = c;

          lcd.setCursor(0, 0);
          lcd.print("Enter Password");
          lcd.setCursor(0, 1);
          char password_buff[17];
          snprintf(password_buff, 17, "%-16s", password); // TODO: this is undefined behavior technically
          lcd.print(password_buff);

          if (latest_evt.type == EventType::SW1_PRESS) {
            if (char_idx < 16) {
              password[char_idx + 1] = '\0';
            }
            WiFi.begin(ssid.c_str(), (char*)password);
            WiFi.reconnect();
            lcd.clear();
            state_g = WAITING_FOR_CONNECTION;
            break;
          }
          else if (latest_evt.type == EventType::SW2_LONG_PRESS) {
            password[char_idx] = '\0';
            --char_idx;
            break;
          }
          else if (latest_evt.type == EventType::SW2_PRESS) {
            ++char_idx;
            break;
          }

          break;
        }
      case WAITING_FOR_CONNECTION:
        {
          lcd.setCursor(0, 0);
          lcd.print("Connecting...");

          if (WiFi.isConnected()) {
            String ip = WiFi.localIP().toString();

            MDNS.begin("sous-vide");

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Connected");
            lcd.setCursor(0, 1);
            lcd.print(ip);

            delay(1000);

            lcd.clear();
            state_g = CHANGE_TEMP;
          }
          break;
        }
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
            state_g = PRE_HEATING;
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

      case PRE_HEATING:
        {
          static bool at_temp = false;
          if (current_temp_g >= setpoint_temp_fahrenheit_g || at_temp) {
            at_temp = true;
            lcd.clear();
            start_cooking_time_sec_g = now_s;
            state_g = HEATING;
            break;
          }
          else {
            static unsigned long last_blink_ms = now_ms;
            static bool blink_state = true;

            if (now_ms - last_blink_ms > 1000) {
              blink_state = !blink_state;
              last_blink_ms = now_ms;
            }

            digitalWrite(GREEN_LED, LOW);
            digitalWrite(RED_LED, blink_state);


            lcd.setCursor(0, 0);
            lcd.print("Pre-Heating");
          }

          lcd.setCursor(0, 1);
          lcd.print(formatTemp(current_temp_g));
          lcd.write(0);
          lcd.print(" ");
          lcd.print(formatTemp(setpoint_temp_fahrenheit_g));

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

          time_left_sec_g = cooking_duration_sec_g - (now_s - start_cooking_time_sec_g);
          if (time_left_sec_g == 0) {
            lcd.clear();
            state_g = FINISHED;
            break;
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
          lcd.print(formatTime(time_left_sec_g));
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
        lcd.print("Finished.");
        lcd.setCursor(0, 1);
        lcd.print("Enjoy!");
        // [[fallthrough]]
      default:
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(RED_LED, LOW);
        break;
    }
  }

  static unsigned int head = 0u;
  if (now_s - last_log_time_s > 30) {
    last_log_time_s = now_s;
    if (state_g == PRE_HEATING || state_g == HEATING) {
      temps.push_back(current_temp_g);
      if (temps.size() > 1000) {
        temps.pop_front();
      }
    }
  }
}


double analogToTemp(const unsigned int thermistor_value) {
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

double potToTemp(const unsigned int potentiometer_value) {
  static constexpr int minimum_temp = 120; // degrees F
  static constexpr int maximum_temp = 190; // degrees F
  return map(potentiometer_value, 3, 938, minimum_temp, maximum_temp);
}

unsigned int potToTime(const unsigned int potentiometer_value) {
  static constexpr unsigned int minimum_time = 30; // X minutes
  static constexpr unsigned int maximum_time = 24 * 60; // X hours
  static constexpr unsigned int minimum_pot = 3;
  static constexpr unsigned int maximum_pot = 930;
  const unsigned int constrained_pot_value = max(min(potentiometer_value, maximum_pot), minimum_pot);
  const double a = (maximum_time - minimum_time) / (pow(maximum_pot,2) - 2.0*minimum_pot*maximum_pot);
  const double b = -2*a*minimum_pot;
  int time_minutes = (a*pow(constrained_pot_value, 2) + b*constrained_pot_value + minimum_time);

  if (time_minutes > 12*60) {
    time_minutes -= time_minutes % 60;
  }
  else if (time_minutes > 6*60) {
    time_minutes -= time_minutes % 30;
  }
  else if (time_minutes > 60) {
    time_minutes -= time_minutes % 15;
  }

  return time_minutes * 60;
}

String formatTime(const unsigned long t_s) {
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

String formatTemp(const unsigned int temp_fahrenheit) {
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
  static constexpr double kI = 0.0001;
  static constexpr double kFF = 0.0018;
  static double integral = 0;

  double error = setpoint_temp_fahrenheit_g - current_temp_g;

  static double last_error = error;

  if (error > -2) {
    integral += error;

    if ((error > 0 && last_error < 0) || (error < 0 && last_error > 0)) {
      integral = 0;
    }

    double derivative = error - last_error;
    duty_cycle_g = kP * error + kI * integral + kFF * setpoint_temp_fahrenheit_g;

    // enforce limit on duty cycle
    duty_cycle_g = fmax(fmin(duty_cycle_g, 1.f), 0.f);

    last_error = error;

#ifdef DEBUG
  Serial.print(integral);
  Serial.print(", ");
  Serial.println(duty_cycle_g);
#endif
  }
  else {
    duty_cycle_g = 0;
  }
}
