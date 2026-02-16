uint8_t analog_pins[6] = { A0, A1, A2, A3, A4, A5 };  // A0-> 2 (left) A5->7 (right)Define the analog input pins
#define rightMotor1 13                                //right motor 240RPM
#define rightMotor2 12
#define right_pwm_normalizer 1
#define rightMotorPWM 11
#define leftMotor1 8  //right motor 320RPM
#define leftMotor2 7
#define leftMotorPWM 10
#define left_pwm_normalizer 0.75
/*  digitalWrite(rightMotor1,HIGH);
    digitalWrite(rightMotor2, LOW); right motor forward

    digitalWrite(leftMotor1,LOW);
    digitalWrite(leftMotor2, HIGH); left motor forward
*/

void setup() {
  Serial.begin(9600);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

  for (uint8_t i = 0; i < 6; i++) {
    pinMode(analog_pins[i], INPUT);
  }
}

unsigned int threshold = 710;  // Threshold for detecting black
void loop() {
  update_black_detections(750);
  Serial.println(get_line_pos());

  uint8_t base_pwm = 127;

  float line_position = get_line_pos();
  if (line_position == -999) {
    Serial.println("No line is found");
    drive_left_motor_at(0, 10, 10);
    drive_right_motor_at(0, 10, 10);

  } else if (line_position > 1.5) {
    drive_left_motor_at(70, 10, 10);
    drive_right_motor_at(-70, 10, 10);
    // drive_left_motor_at(0, 1, 255);
    // drive_right_motor_at(0, 1, 255);

  } else if (line_position < -1.5) {
    drive_left_motor_at(-70, 10, 10);
    drive_right_motor_at(70, 10, 10);
    // drive_left_motor_at(0, 1, 255);
    // drive_right_motor_at(0, 1, 255);
  } else {
    drive_left_motor_at(70, 10, 10);
    drive_right_motor_at(70, 10, 10);
  }
}

uint8_t is_black[6] = { 0, 0, 0, 0, 0, 0 };
uint8_t update_black_detections(int black_threshold) {
  //uint8_t analog_pins[6] = {A0, A1, A2, A3, A4, A5}; // A0-> 2 (left) A5->7 (right)Define the analog input pins
  //right -> 2, left ->7

  for (uint8_t i = 0; i < 6; i++) {
    int analog_value = analogRead(analog_pins[i]);
    uint8_t digital_val = 0;
    if (analog_value > black_threshold) digital_val = 1;
    is_black[i] = digital_val;
  }
}
float get_line_pos() {

  int sensor_coefficients[6] = { -3, -2, -1, 1, 2, 3 };
  float pos_value = 0;
  uint8_t black_counter = 0;
  for (int i = 0; i < 6; i++) {
    if (is_black[i]) {
      pos_value = pos_value + sensor_coefficients[i];
      black_counter = black_counter + 1;
    }
  }
  if (black_counter != 0) {
    pos_value = pos_value / black_counter;
  } else {
    pos_value = -999;  //no line is found
  }

  return pos_value;
}

void test_print_is_black_array() {
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(String(is_black[i]) + " ");
  }
  Serial.println("");
}

void drive_right_motor_at(int pwm_val, int update_period_ms, uint8_t delta_pwm_per_period) {
  static unsigned long last_time_update = 0;
  static uint8_t actual_pwm = 0;
  if (millis() - last_time_update < update_period_ms) return;
  int delta_actual_pwm = pwm_val - actual_pwm;

  if (delta_actual_pwm < -delta_pwm_per_period) {
    delta_actual_pwm = -delta_pwm_per_period;
  } else if (delta_actual_pwm > delta_pwm_per_period) {
    delta_actual_pwm = delta_pwm_per_period;
  }
  actual_pwm = int((actual_pwm + delta_actual_pwm) * right_pwm_normalizer);

  //------------

  if (actual_pwm < 0) {  //go reverse
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    analogWrite(rightMotorPWM, actual_pwm);
  } else {
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, actual_pwm);
  }
}

void drive_left_motor_at(int pwm_val, int update_period_ms, uint8_t delta_pwm_per_period) {
  static unsigned long last_time_update = 0;
  static uint8_t actual_pwm = 0;
  if (millis() - last_time_update < update_period_ms) return;
  int delta_actual_pwm = pwm_val - actual_pwm;

  if (delta_actual_pwm < -delta_pwm_per_period) {
    delta_actual_pwm = -delta_pwm_per_period;
  } else if (delta_actual_pwm > delta_pwm_per_period) {
    delta_actual_pwm = delta_pwm_per_period;
  }
  actual_pwm = int((actual_pwm + delta_actual_pwm) * left_pwm_normalizer);

  //-------------
  if (actual_pwm < 0) {  //go reverse
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, -actual_pwm);
  } else {
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    analogWrite(leftMotorPWM, actual_pwm);
  }
}
