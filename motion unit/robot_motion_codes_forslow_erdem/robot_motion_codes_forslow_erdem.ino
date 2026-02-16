#define BATTERY_VOLTAGE 7.4  //in volts

#define BLACK_THRESHOLD 850  //if analogRead() value is greater than this, black is detected

#define WHEEL_RADIUS 0.03        //in meters
#define MOTOR_RATED_RPM 60       // turns per minute
#define MOTOR_RATED_VOLTAGE 6.0  //rated voltage of the motor
#define MOTOR1_PWM 3
#define MOTOR1_A 5
#define MOTOR1_B 4
#define MOTOR2_PWM 11
#define MOTOR2_A 9
#define MOTOR2_B 10

//#define LED_PIN 13
#define ISRESETTED 13
#define RFID_READ 2  // aka TALK_PIN
#define FOLLOWING_THE_LINE 1
#define AT_CROSS_SECTION 2


unsigned int threshold = 900;  // Threshold for detecting black
uint8_t indicator = 0;
uint8_t oldData = 0;
uint16_t turn_delay = 1000;
unsigned long oldData_startTime = 0;
int buzzerPin = 8;
int movement_terminated = 0;


uint8_t line_follower_analog_pins[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };  // A0-> 1 (left) A7->8 (right)Define the analog input pins

void setup() {
  // Pins D3 and D11 - 8 kHz
  TCCR2B = 0b00000010;  // x8
  TCCR2A = 0b00000011;  // fast pwm

  delay(2500);
  Serial.begin(9600);
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(line_follower_analog_pins[i], INPUT);
  }
  pinMode(ISRESETTED, OUTPUT);
  digitalWrite(ISRESETTED, HIGH);
  delay(2000);
  digitalWrite(ISRESETTED, LOW);
  // pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_A, OUTPUT);
  pinMode(MOTOR1_B, OUTPUT);

  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);
  pinMode(RFID_READ, INPUT);
  pinMode(buzzerPin, OUTPUT);
  // digitalWrite(MOTOR1_A, HIGH);
  // digitalWrite(MOTOR1_B, LOW);
  // analogWrite(MOTOR1_PWM, 255);

  // digitalWrite(MOTOR2_A, LOW);
  // digitalWrite(MOTOR2_B, HIGH);
  // analogWrite(MOTOR2_PWM, 255);
}

void loop() {
  //Serial.println(digitalRead(RFID_READ));
  move_forward_until_line_crossing(5000);
  delay(2500);
  move_forward_until_RFID_read();
  delay(1000);

  //   //step forward until RFID is read

  //   while (true) {
  //     uint8_t pin_val = digitalRead(RFID_READ);
  //     if(pin_val == 1)break;
  //     open_loop_go_forward(0.1);
  //     delay(2500);
  //   }
  //   set_motor_speeds(0.0, 0.0);
  //   delay(750);

  //   //set_motor_speeds(-0.1, 0.1);



  //   // if (digitalRead(RFID_READ) == 1) {
  //   //   Serial.println("Read an RFID, stopping.");
  //   //   set_motor_speeds(0, 0);
  //   //   delay(700);
  //   //   // Serial.println("Read an RFID, stopping.");
  //   // }

  //   unsigned long currentTime = millis();
  //   if (currentTime - oldData_startTime > 3000) { oldData = 0; }
  //   //Checking if there is any new data from the RFID reader
  //   if (Serial.available() > 0) {  // Check if data is available to read

  //     uint8_t receivedNum = Serial.read();

  //     if (receivedNum != oldData) {
  //       // Print the received character
  //       // uint8_t decision_unit_command = int(receivedNum);
  //       // Serial.println(decision_unit_command);
  //       uint8_t x = int(receivedNum) / 16;
  //       uint8_t y = int(receivedNum) % 16;
  //       Serial.print("Received Command: ");
  //       Serial.println(x);
  //       // Serial.println(y);

  //       int decision_unit_command = x;


  //       if ((decision_unit_command >= 0 || decision_unit_command < 6)) {
  //         digitalWrite(buzzerPin, HIGH);  // Turn the buzzer on
  //         delay(500);                     // Wait for 1 second (1000 milliseconds)
  //         digitalWrite(buzzerPin, LOW);   // Turn the buzzer off
  //         delay(500);
  //       }

  //       if (decision_unit_command == 0) {
  //         set_motor_speeds(0, 0);
  //       } else if (decision_unit_command == 1) {
  //         open_loop_go_forward(0.23);
  //       } else if (decision_unit_command == 2) {
  //         // open_loop_turn_right(74.0);
  //         turn_right_closed_loop();
  //       } else if (decision_unit_command == 3) {
  //         // open_loop_turn_left(74.0);
  //         turn_left_closed_loop();
  //       } else if (decision_unit_command == 4) {
  //         // open_loop_turn_right(140.0);
  //         turn_right_closed_loop();
  //         turn_right_closed_loop();
  //       } else if (decision_unit_command == 5) {

  //         movement_terminated = 1;
  //         set_motor_speeds(0, 0);
  //       }
  //       // STOP FOR BU COMM AND THEN GO FORWARD
  //       else if (decision_unit_command == 6) {
  //         set_motor_speeds(0, 0);
  //         delay(5000);
  //         open_loop_go_forward(0.23);
  //       }
  //       // STOP FOR BU COMM AND THEN TURN RIGHT
  //       else if (decision_unit_command == 7) {
  //         set_motor_speeds(0, 0);
  //         delay(5000);
  //         // open_loop_turn_right(74.0);
  //         turn_right_closed_loop();
  //       }
  //       // STOP FOR BU COMM AND THEN TURN LEFT
  //       else if (decision_unit_command == 8) {
  //         set_motor_speeds(0, 0);
  //         delay(5000);
  //         // open_loop_turn_left(74.0);
  //         turn_left_closed_loop();
  //       }
  //       // STOP FOR BU COMM AND THEN TURN AROUND
  //       else if (decision_unit_command == 9) {
  //         set_motor_speeds(0, 0);
  //         delay(5000);
  //         // open_loop_turn_right(140.0);
  //         turn_right_closed_loop();
  //         turn_right_closed_loop();
  //       }
  //     }
  //   }

  //   if (movement_terminated != 1) {

  //     move_forward_until_line_crossing(5000);
  //     delay(1000);

  //     open_loop_go_forward(0.15);
  //     delay(1000);
  //   }
}


uint8_t is_black[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // if i'th sensor is black, set to 1. otherwise 0

uint8_t update_black_detections() {
  for (uint8_t i = 0; i < 8; i++) {
    int analog_value = analogRead(line_follower_analog_pins[i]);
    uint8_t digital_val = 0;
    if (analog_value > BLACK_THRESHOLD) digital_val = 1;
    is_black[i] = digital_val;
  }
}

void test_print_is_black_array() {
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(String(is_black[i]) + " ");
  }
  Serial.println("");
}

void set_motor_speeds(float M1_speed, float M2_speed) {
  // M1 and M2 speeds are in meters per second and negative value means reverse direction.
  float max_speed = (BATTERY_VOLTAGE / MOTOR_RATED_VOLTAGE) * (MOTOR_RATED_RPM / 60.0) * (6.28 * WHEEL_RADIUS);  //in meters per second, 0.23m/s for recent motor.

  //set the direction
  if (M1_speed > 0) {
    digitalWrite(MOTOR1_A, LOW);
    digitalWrite(MOTOR1_B, HIGH);
  } else {
    digitalWrite(MOTOR1_A, HIGH);
    digitalWrite(MOTOR1_B, LOW);
  }

  if (M2_speed > 0) {
    digitalWrite(MOTOR2_A, HIGH);
    digitalWrite(MOTOR2_B, LOW);
  } else {
    digitalWrite(MOTOR2_A, LOW);
    digitalWrite(MOTOR2_B, HIGH);
  }

  //consider the magnitudes of the speeds since the directions are set.
  M1_speed = abs(M1_speed);
  M2_speed = abs(M2_speed);
  if (M1_speed > max_speed) M1_speed = max_speed;
  if (M2_speed > max_speed) M2_speed = max_speed;

  uint16_t M1_pwm = uint8_t(255.0 * (M1_speed / max_speed));
  uint16_t M2_pwm = uint8_t(255.0 * (M2_speed / max_speed));
  Serial.println(String(M1_speed) + " " + String(M1_pwm) + " " + String(M2_speed) + " " + String(M2_pwm));

  analogWrite(MOTOR1_PWM, M1_pwm);
  analogWrite(MOTOR2_PWM, M2_pwm);
}

void open_loop_turn_right(float angle) {
  //it is found emprically that with below configuration, robot complates 10 complete turns in 40.87 seconds -> 4.87 seconds per 360 degree -> 0.0135 seconds per 1 degree.
  //By using this function, you can turn the robot in both left or right directions. Just insert negative angle to perform left turn.

  if (angle > 360) angle = 360;
  else if (angle < -360) angle = -360;

  uint16_t delay_duration = uint16_t(abs(angle) * 13.5);

  if (angle > 0) {
    set_motor_speeds(0.20, -0.20);
  } else {
    set_motor_speeds(-0.20, 0.20);
  }

  delay(delay_duration);
  set_motor_speeds(0, 0);
}

void open_loop_turn_left(float angle) {
  if (angle > 360) angle = 360;
  else if (angle < -360) angle = -360;

  uint16_t delay_duration = uint16_t(abs(angle) * 13.5);

  if (angle > 0) {
    set_motor_speeds(-0.20, 0.20);
  } else {
    set_motor_speeds(0.20, -0.20);
  }

  delay(delay_duration);
  set_motor_speeds(0, 0);
}

void open_loop_go_forward(float distance) {
  //This function drives the wheels at 0.17m/s speed. thus each milisecond, it travels 0.00017m/ms
  //By using this function, you can go forward ot backward. Just insert negative distance to perform left turn. The distance is given in meters

  //put an upper bound for the distance
  if (distance > 0.25) {
    distance = 0.25;
  } else if (distance < -0.25) {
    distance = -0.25;
  }

  uint16_t delay_duration = uint16_t(abs(distance) / 0.00017);

  if (distance > 0) {  //go forward
    set_motor_speeds(0.19, 0.19);
  } else {
    set_motor_speeds(-0.19, -0.19);
  }

  delay(delay_duration);
  set_motor_speeds(0, 0);
}

void move_forward_until_line_crossing(unsigned long timeout_ms) {
  //TODO: this function should return why it is returned, i.e. line is lost, crossing is found etc.
  float line_position = -99;                 //-99 -> means no line is found.
  static unsigned long line_found_when = 0;  //When the last time a black is detected

  unsigned long start_time = millis();

  while (true) {
    if (millis() - start_time > timeout_ms) break;  //exit the loop after a certain timeout(i.e.~open loop approximated movement time)

    update_black_detections();
    uint8_t number_of_blacks = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (is_black[i] == 1) number_of_blacks += 1;
    }

    if (number_of_blacks > 4) {  //line crossing is found, STOP
      //HARD BREAK for a short time! Then stop the motors.
      set_motor_speeds(-0.05, -0.05);
      delay(75);
      set_motor_speeds(0.0, 0.0);
      break;
    } else if (number_of_blacks == 0) {  //line is lost, STOP if it is not found for a while using the last line position
      if ((millis() - line_found_when > 750) && line_position != -99) {
        set_motor_speeds(0.0, 0.0);
        line_position = -99;
        break;
      } else if ((millis() - line_found_when < 750) && line_position != -99) {
        //TODO: Should try to find line. Using line_position may be beneficial. Note that it is 'STATIC'
      } else if (line_position == -99) {
        break;
      }
    }

    //=======================================================================================0
    //Keep going forward with feedback.
    if (number_of_blacks > 0) {  // Atleast one sensor detects black.
      line_found_when = millis();
      line_position = 0;
      for (uint8_t i = 0; i < 8; i++) {
        if (is_black[i] == 1) line_position += (i - (3.5));
      }
      line_position = line_position / float(number_of_blacks);  // Ensure that this is a float division
    } else {
      //black is not detected, using previous 'line_position' for a certain duration.
    }

    float BASE_SPEED = 0.20;                 //speed that each motor should go with when the line is centered in m/s
    float del_speed = line_position * 0.03;  // m/s
    float M1_speed = BASE_SPEED - del_speed;
    float M2_speed = BASE_SPEED + del_speed;
    if (M1_speed < 0.14) M1_speed = 0.14;
    if (M2_speed < 0.14) M2_speed = 0.14;

    set_motor_speeds(M1_speed, M2_speed);
  }
}

void move_forward_until_RFID_read() {
  while (true) {
    uint8_t pin_val = digitalRead(RFID_READ);
    if (pin_val == 1) break;
    set_motor_speeds(0.19, 0.19);
  }
  set_motor_speeds(0, 0);
}

void rotate_cw() {
  set_motor_speeds(0.20, -0.20);
}
void rotate_ccw() {
  set_motor_speeds(-0.20, 0.20);
}

void turn_right_closed_loop() {

  while (true) {
    update_black_detections();
    uint8_t number_of_blacks = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (is_black[i] == 0) {
        number_of_blacks = number_of_blacks + 1;
      }
    }
    if (number_of_blacks != 0) break;
    rotate_cw();
  }

  while (!(is_black[0])) {

    // Serial.println(get_line_pos());
    update_black_detections();
    rotate_cw();
  }

  while (!(is_black[1])) {

    // Serial.println(get_line_pos());
    update_black_detections();
    rotate_cw();
  }

  while (!(is_black[2])) {

    // Serial.println(get_line_pos());
    update_black_detections();
    rotate_cw();
  }

  while (!(is_black[3])) {

    // Serial.println(get_line_pos());
    update_black_detections();
    rotate_cw();
  }
}


void turn_left_closed_loop() {

  while (true) {
    update_black_detections();
    uint8_t number_of_blacks = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (is_black[i] == 0) {
        number_of_blacks = number_of_blacks + 1;
      }
    }
    if (number_of_blacks != 0) break;
    rotate_ccw();
  }

  while (!(is_black[7])) {

    // Serial.println(get_line_pos());
    update_black_detections();
    rotate_ccw();
  }

  while (!(is_black[6])) {

    // Serial.println(get_line_pos());
    update_black_detections();
    rotate_ccw();
  }

  while (!(is_black[5])) {

    // Serial.println(get_line_pos());
    update_black_detections();
    rotate_ccw();
  }

  while (!(is_black[4])) {

    // Serial.println(get_line_pos());
    update_black_detections();
    rotate_ccw();
  }
}
