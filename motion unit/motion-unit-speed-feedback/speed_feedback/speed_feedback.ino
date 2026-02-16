#define SHUNT_FEEDBACK_PIN A0
#define M1_ENABLE 3
#define M1_A 4
#define M1_B 5
#define M1_R 5.7    //armature resistance of the motor
#define M1_kb 0.02  // motor back emf constant (V per rpm), as increased, RPM is decreased
#define M2_ENABLE 11
#define M2_A 12
#define M2_B 13
#define M2_R 5.7                        //armature resistance of the motor
#define M2_kb 0.02                      // motor back emf constant (V per rpm), as increased, RPM is decreased
#define MOTOR_IN_SERIES_RESISTANCE 0.5  // The resitance in series with the H bridge. (i.e. ~shunt resistor)

uint8_t line_follower_analog_pins[7] = { A1, A2, A3, A4, A5, A6, A7 };

float BATTERY_VOLTAGE = 0;  //measured only once at the start-up
void setup() {
  Serial.begin(9600);

  for (uint8_t i = 0; i < 7; i++) {
    pinMode(line_follower_analog_pins[i], INPUT);
  }
  pinMode(SHUNT_FEEDBACK_PIN, INPUT);
  pinMode(M1_ENABLE, OUTPUT);
  pinMode(M1_A, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(M2_ENABLE, OUTPUT);
  pinMode(M2_A, OUTPUT);
  pinMode(M2_B, OUTPUT);

  digitalWrite(M1_ENABLE, LOW);  // ensure the motors are turned off
  digitalWrite(M2_ENABLE, LOW);  // ensure the motors are turned off

  delay(5);
  BATTERY_VOLTAGE = return_shunt_voltage_measurement();  //assumes
  Serial.println("Battery Voltage: ~" + String(BATTERY_VOLTAGE) + "V");
  delay(5000);
}

float base_speed = 65;

void loop() {
  test_go_straight();
  test_go_right();
}

void test_go_right() {
  //Go right
  unsigned long start_time = millis();


  //turn right until current line is lost
  update_black_detections(750);
  while(get_number_of_black_detections() > 0) {
    drive_motors_at_constant_RPM(-50, 50);
    update_black_detections(750);
  }

  //turn right until next line is found
  while (get_number_of_black_detections() == 0) {
    drive_motors_at_constant_RPM(-50, 50);
    update_black_detections(750);
  }

  //One of the sensors is found a black line. Turn right (open loop) a little bit so that line is in the middle
  drive_motors_at_constant_RPM(-50, 50);
  delay(125);
  halt_motors();

  delay(500);
}

void test_go_straight() {  //Go straight

  unsigned long start_time = millis();
  while (true) {
    update_black_detections(750);

    if(millis()-start_time > 500){
      if (get_number_of_black_detections() == 0) break;
    }

    float line_pos = get_straight_line_position();

    float proportional = 4;
    float del_speed = proportional * line_pos;
    float right_speed = base_speed + del_speed;
    float left_speed = base_speed - del_speed;
    drive_motors_at_constant_RPM(left_speed, right_speed);
  }
  delay(175);
  halt_motors();
  delay(500);
}

uint8_t is_black[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // D1 to D8 where D8 is not utilized.

uint8_t update_black_detections(int black_threshold) {
  //uint8_t line_follower_analog_pins[7] = {A1, A2, A3, A4, A5, A6, A7}; // A1-> D1, A2->D2 (left) A7->D7 (right)Define the analog input pins
  //D1 is the Leftmost, D8 id the rightmost sensor

  for (uint8_t i = 0; i < 7; i++) {
    int analog_value = analogRead(line_follower_analog_pins[i]);
    uint8_t digital_val = 0;
    if (analog_value > black_threshold) digital_val = 1;
    is_black[i] = digital_val;
  }
}
void print_is_black_array() {
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(String(is_black[i]) + ", ");
  }
  Serial.println();
}

uint8_t get_number_of_black_detections() {
  uint8_t number_of_blacks = 0;
  for (uint8_t i = 1; i < 8; i++) {
    if (is_black[i] == 1) number_of_blacks += 1;
  }
  return number_of_blacks;
}

float get_straight_line_position() {
  static float line_position = 0;

  float line_pos_values[8] = { -4, -3, -2, -1, 1, 2, 3, 4 };

  float line_position_candidate = 0;
  uint8_t number_of_additions = 0;
  for (uint8_t i = 1; i < 8; i++) {
    line_position_candidate += line_pos_values[i] * is_black[i];
    if (is_black[i]) number_of_additions += 1;
  }

  if (number_of_additions > 0) {  // t
    line_position = line_position_candidate / number_of_additions;
  }


  return line_position;
}
void move_test() {
  for (uint8_t i = 0; i < 4; i++) {
    unsigned long start_time = millis();
    while (millis() - start_time < 400) {
      drive_motors_at_constant_RPM(125, 125);
    }
    delay(2100);

    start_time = millis();
    while (millis() - start_time < 400) {
      drive_motors_at_constant_RPM(0, 0);
    }
    delay(600);

    start_time = millis();
    while (millis() - start_time < 400) {
      drive_motors_at_constant_RPM(-75, 75);
    }

    start_time = millis();
    while (millis() - start_time < 400) {
      drive_motors_at_constant_RPM(0, 0);
    }
    delay(600);
  }

  delay(2500);
}
float return_shunt_voltage_measurement() {
  int reading = analogRead(A0);
  float voltage = 0.00989736 * (reading);  //V
  return voltage;
}

void halt_motors() {
  digitalWrite(M1_ENABLE, LOW);
  digitalWrite(M2_ENABLE, LOW);
}

void drive_motors_at_constant_RPM(float DESIRED_M1_RPM, float DESIRED_M2_RPM) {
  uint8_t TIMEOUT_MS = 20;
  uint8_t APPLY_FULL_PERIOD = 5;  //how many miliseconds the full on/off is applied;
  int M1_approximated_rpm = 0;
  int M2_approximated_rpm = 0;

  // Set the directions of the motors
  if (DESIRED_M1_RPM > 0) {
    digitalWrite(M1_A, LOW);
    digitalWrite(M1_B, HIGH);
  } else {
    digitalWrite(M1_A, HIGH);
    digitalWrite(M1_B, LOW);
  }
  if (DESIRED_M2_RPM > 0) {
    digitalWrite(M2_A, HIGH);
    digitalWrite(M2_B, LOW);
  } else {
    digitalWrite(M2_A, LOW);
    digitalWrite(M2_B, HIGH);
  }

  DESIRED_M1_RPM = abs(DESIRED_M1_RPM);  //since direcion information is already utilized, only the magnitude is required.
  DESIRED_M2_RPM = abs(DESIRED_M2_RPM);  //since direcion information is already utilized, only the magnitude is required.

  //try to converge to the desired RPMs
  float M1_rpm = 0;            // in repeats  per minute
  float M1_current_drawn = 0;  //in Amps
  float M1_back_emf = 0;       // in Volts
  float M2_rpm = 0;            // in repeats  per minute
  float M2_current_drawn = 0;  //in Amp
  float M2_back_emf = 0;       // in Volts

  unsigned long started_at = millis();
  while (millis() - started_at < TIMEOUT_MS) {
    //turn off the motors
    digitalWrite(M1_ENABLE, LOW);
    digitalWrite(M2_ENABLE, LOW);
    delayMicroseconds(500);                                                   //let the transient finish
    float voltage_measurement_both_off = return_shunt_voltage_measurement();  //voltage when both motors are off
    digitalWrite(M1_ENABLE, HIGH);
    delayMicroseconds(500);                                             // Wait for M1 transients
    float voltage_measurement_M1 = return_shunt_voltage_measurement();  //voltage when M1 is on, M2 is off
    digitalWrite(M1_ENABLE, LOW);
    digitalWrite(M2_ENABLE, HIGH);
    delayMicroseconds(500);                                             // Wait for M2 transients
    float voltage_measurement_M2 = return_shunt_voltage_measurement();  //voltage when M1 is off, M2 is on

    M1_current_drawn = 2 * (voltage_measurement_both_off - voltage_measurement_M1);  //the voltage drop is over the 0.5Ohm resistor
    M2_current_drawn = 2 * (voltage_measurement_both_off - voltage_measurement_M2);

    M1_back_emf = voltage_measurement_both_off - (M1_R * M1_current_drawn);
    M2_back_emf = voltage_measurement_both_off - (M2_R * M2_current_drawn);

    M1_rpm = M1_back_emf / M1_kb;
    M2_rpm = M2_back_emf / M2_kb;

    //if RPM < DESIRED_RPM, set motor on, otherwise set it low.
    if (M1_rpm < DESIRED_M1_RPM) {
      digitalWrite(M1_ENABLE, HIGH);
    } else {
      digitalWrite(M1_ENABLE, LOW);
    }
    if (M2_rpm < DESIRED_M2_RPM) {
      digitalWrite(M2_ENABLE, HIGH);
    } else {
      digitalWrite(M2_ENABLE, LOW);
    }
    delay(APPLY_FULL_PERIOD);  //keep motors full on/off this many miliseconds. Due to its inertia, the speed change will be differantial.
  }

  //assume motor draws 0.6A when loaded. the steady state PWM is approximated as
  float M1_back_emf_at_desired_rpm = M1_kb * DESIRED_M1_RPM;
  float M2_back_emf_at_desired_rpm = M2_kb * DESIRED_M2_RPM;

  float M1_voltage_applied_when_on = BATTERY_VOLTAGE - (MOTOR_IN_SERIES_RESISTANCE + M1_R) * M1_current_drawn;  //note that M2 current also effect the voltage drop on series resistance, yet it is ignored for simplicity
  float M2_voltage_applied_when_on = BATTERY_VOLTAGE - (MOTOR_IN_SERIES_RESISTANCE + M2_R) * M2_current_drawn;  //note that M2 current also effect the voltage drop on series resistance, yet it is ignored for simplicity

  //If this state is steady-state, below pwm values will more-or-less keep the motors at this state.
  int M1_PWM = constrain(int(255 * (M1_back_emf_at_desired_rpm / M1_voltage_applied_when_on)), 0, 255);
  int M2_PWM = constrain(int(255 * (M2_back_emf_at_desired_rpm / M2_voltage_applied_when_on)), 0, 255);

  analogWrite(M1_ENABLE, M1_PWM);
  analogWrite(M2_ENABLE, M2_PWM);
  delay(25);
}
