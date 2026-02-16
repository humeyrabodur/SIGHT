uint8_t analog_pins[6] = { A0, A1, A2, A3, A4, A5 };  // A0-> 2 (left) A5->7 (right)Define the analog input pins, 4 D1, 5 D7
#define rightMotor1 4                                 //right motor 240RPM
#define rightMotor2 12
#define right_pwm_normalizer 1
#define rightMotorPWM 11
#define leftMotor1 8  //right motor 320RPM
#define leftMotor2 7
#define leftMotorPWM 10
#define left_pwm_normalizer 0.75
#define LED_PIN 13
#define LISTEN 2
#define ISRESETTED 13
float last_position = 0;
uint8_t base_pwm = 90;
int intersection = 0;
uint8_t is_black[6] = { 0, 0, 0, 0, 0, 0 };
int stop = 0;
int receivedValue = 0;
// Function prototype
uint8_t update_black_detections(int black_threshold);
void test_print_is_black_array();
float get_line_pos();
void rotate_cw();
void rotate_ccw();
void move(int line_position);
void drive_right_motor_at(int pwm_val, int update_period_ms, uint8_t delta_pwm_per_period);
void drive_left_motor_at(int pwm_val, int update_period_ms, uint8_t delta_pwm_per_period);
void stop_mu();

/*  digitalWrite(rightMotor1,HIGH);
    digitalWrite(rightMotor2, LOW); right motor forward

    digitalWrite(leftMotor1,LOW);
    digitalWrite(leftMotor2, HIGH); left motor forward
*/
void setup() {
  Serial.begin(9600);
  compass.init();
  Serial.println("resetted!!!");
  pinMode(ISRESETTED, OUTPUT);
  digitalWrite(ISRESETTED, HIGH);
  delay(300);
  digitalWrite(ISRESETTED, LOW);
  pinMode(LED_PIN, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(LISTEN, INPUT);

  for (uint8_t i = 0; i < 6; i++) {
    pinMode(analog_pins[i], INPUT);
  }
}

unsigned int threshold = 900;  // Threshold for detecting black
uint8_t indicator = 0;


void loop() {


  update_black_detections(threshold);
  //Serial.println(get_line_pos());
  test_print_is_black_array();
  float line_position = get_line_pos();  // This will be deleted
  move(line_position);
}


void move(int line_position) {
  if (line_position == -999) {
    Serial.println("No line is found");

    if (last_position > 0) {
      rotate_cw();
    } else {
      rotate_ccw();
    }

  }
    else if (line_position > 0) {
      int right_pwm = int(base_pwm * right_pwm_normalizer);
      int left_pwm = int((base_pwm + 1.5 * line_position * line_position) * left_pwm_normalizer);
      drive_left_motor_at(left_pwm, 25, 1);
      drive_right_motor_at(right_pwm, 25, 1);
      last_position = line_position;
    }
    else {
      line_position = -line_position;
      int right_pwm = int((base_pwm + 1.5 * line_position * line_position) * right_pwm_normalizer);
      int left_pwm = int(base_pwm * left_pwm_normalizer);
      drive_left_motor_at(left_pwm, 25, 1);
      drive_right_motor_at(right_pwm, 25, 1);
      last_position = -line_position;
    }
  }




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
    int sensor_coefficients[6] = { 3, 2, 1, -1, -2, -3 };
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
      pos_value = -999;
    }
    //no line is found

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
    actual_pwm = actual_pwm + delta_actual_pwm;

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
    actual_pwm = actual_pwm + delta_actual_pwm;

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

  void rotate_cw() {
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    analogWrite(leftMotorPWM, base_pwm);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    analogWrite(rightMotorPWM, base_pwm);
  }
  void rotate_ccw() {
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, base_pwm);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, base_pwm);
  }

  void stop_mu() {

    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);

    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
  }