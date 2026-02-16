#define BLACK_THRESHOLD 850  //if analogRead() value is greater than this, black is detected

#define MOTOR1_PWM 3
#define MOTOR1_A 4
#define MOTOR1_B 5
#define MOTOR2_PWM 11
#define MOTOR2_A 10
#define MOTOR2_B 9

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

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_A, OUTPUT);
  pinMode(MOTOR1_B, OUTPUT);

  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);

  // digitalWrite(MOTOR1_A, HIGH);
  // digitalWrite(MOTOR1_B, LOW);
  // analogWrite(MOTOR1_PWM, 255);

  // digitalWrite(MOTOR2_A, LOW);
  // digitalWrite(MOTOR2_B, HIGH);
  // analogWrite(MOTOR2_PWM, 255);
}

void loop() {
  // update_black_detections();
  // test_print_is_black_array();

  // move_forward_until_line_crossing(5000);
  // delay(1000);
  // open_loop_go_forward_smooth(325);
  // delay(1000);
  // open_loop_turn_right_smooth(210);
  // delay(2500);
  move_forward_until_line_crossing_hard(5000);
  delay(1000);
  open_loop_turn_right_smooth(275);
  delay(2500);
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

void move_forward_until_line_crossing_hard(unsigned long timeout_ms) {
  //TODO: this function should return why it is returned, i.e. line is lost, crossing is found etc.
  static float line_position = -99;          //-99 -> means no line is found.
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
      delay(165);
      digitalWrite(MOTOR1_A, LOW);
      digitalWrite(MOTOR1_B, HIGH);
      digitalWrite(MOTOR2_A, HIGH);
      digitalWrite(MOTOR2_B, LOW);
      analogWrite(MOTOR1_PWM, 255);
      analogWrite(MOTOR2_PWM, 255);
      delay(75);
      analogWrite(MOTOR1_PWM, 0);
      analogWrite(MOTOR2_PWM, 0);
      //TODO: maybe go forward a little so that RFID reader is read.
      break;
    } else if (number_of_blacks == 0) {  //line is lost, STOP if it is not found for a while using the last line position
      if (millis() - line_found_when > 500 && line_position != -99) {
        analogWrite(MOTOR1_PWM, 0);
        analogWrite(MOTOR2_PWM, 0);
        line_position = -99;
        break;
      } else {
        //continue moving forward using the last position
      }

      //TODO: Should try to find line. Using line_position may be beneficial. Note that it is 'STATIC'
    }
    //=======================================================================================0
    //number of blacks found is less than 2 and atleast 1 black is found, means the bug is stil on a straight line. Keep going forward with feedback.
    // move forward with feedback
    line_found_when = millis();

    line_position = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (is_black[i] == 1) line_position += (i - (3.5));
    }
    line_position = line_position / float(number_of_blacks);  // Ensure that this is a float division

    if (line_position > 1) {
      //Go straight

      //set directions so that it goes forward
      digitalWrite(MOTOR1_A, HIGH);
      digitalWrite(MOTOR1_B, LOW);
      digitalWrite(MOTOR2_A, LOW);
      digitalWrite(MOTOR2_B, HIGH);

      //set PWM values
      analogWrite(MOTOR1_PWM, 200);
      analogWrite(MOTOR2_PWM, 245);
      delay(25);

    } else if (line_position < -1) {
      //Go straight

      //set directions so that it goes forward
      digitalWrite(MOTOR1_A, HIGH);
      digitalWrite(MOTOR1_B, LOW);
      digitalWrite(MOTOR2_A, LOW);
      digitalWrite(MOTOR2_B, HIGH);

      //set PWM values
      analogWrite(MOTOR1_PWM, 245);
      analogWrite(MOTOR2_PWM, 200);
      delay(25);
    } else {
      //Go straight

      //set directions so that it goes forward
      digitalWrite(MOTOR1_A, HIGH);
      digitalWrite(MOTOR1_B, LOW);
      digitalWrite(MOTOR2_A, LOW);
      digitalWrite(MOTOR2_B, HIGH);

      //set PWM values
      analogWrite(MOTOR1_PWM, 230);
      analogWrite(MOTOR2_PWM, 230);
      delay(25);
    }
  }
}

void move_forward_until_line_crossing(unsigned long timeout_ms) {
  //TODO: this function should return why it is returned, i.e. line is lost, crossing is found etc.
  static float line_position = -99;          //-99 -> means no line is found.
  static unsigned long line_found_when = 0;  //When the last time a black is detected

  unsigned long start_time = millis();
  while (true) {
    if (millis() - start_time > timeout_ms) break;  //exit the loop after a certain timeout(i.e.~open loop approximated movement time)

    update_black_detections();
    uint8_t number_of_blacks = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (is_black[i] == 1) number_of_blacks += 1;
    }
    if (number_of_blacks > 3) {  //line crossing is found, STOP
      //HARD BREAK for a short time! Then stop the motors.
      digitalWrite(MOTOR1_A, LOW);
      digitalWrite(MOTOR1_B, HIGH);
      digitalWrite(MOTOR2_A, HIGH);
      digitalWrite(MOTOR2_B, LOW);
      analogWrite(MOTOR1_PWM, 255);
      analogWrite(MOTOR2_PWM, 255);
      delay(75);
      analogWrite(MOTOR1_PWM, 0);
      analogWrite(MOTOR2_PWM, 0);
      //TODO: maybe go forward a little so that RFID reader is read.
      break;
    } else if (number_of_blacks == 0) {  //line is lost, STOP if it is not found for a while using the last line position
      if (millis() - line_found_when > 500 && line_position != -99) {
        analogWrite(MOTOR1_PWM, 0);
        analogWrite(MOTOR2_PWM, 0);
        line_position = -99;
        break;
      } else {
        //continue moving forward using the last position
      }

      //TODO: Should try to find line. Using line_position may be beneficial. Note that it is 'STATIC'
    }
    //=======================================================================================0
    //number of blacks found is less than 2 and atleast 1 black is found, means the bug is stil on a straight line. Keep going forward with feedback.
    // move forward with feedback
    line_found_when = millis();

    line_position = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (is_black[i] == 1) line_position += (i - (3.5));
    }
    line_position = line_position / float(number_of_blacks);  // Ensure that this is a float division
    uint8_t BASE_PWM = 205;                                   // This is an emprical value which is found to be fine. Neither halts nor accelerates too fast.

    uint8_t K_p = 6;
    float del_PWM = K_p * line_position;  //line_pos is bound to [-3.5, 3.5]

    int motor1_PWM = BASE_PWM - int(del_PWM);
    int motor2_PWM = BASE_PWM + int(del_PWM);

    // int motor1_PWM = BASE_PWM;
    // int motor2_PWM = BASE_PWM;
    // if (del_PWM < 0) {
    //   motor1_PWM += -del_PWM;
    // } else {
    //   motor2_PWM += del_PWM;
    // }

    //ensure that PWM values are bounded to [0,255];
    if (motor1_PWM > 255) motor1_PWM = 255;
    else if (motor1_PWM < 0) motor1_PWM = 0;
    if (motor2_PWM > 255) motor2_PWM = 255;
    else if (motor2_PWM < 0) motor2_PWM = 0;

    Serial.println(String(motor1_PWM) + " , " + String(motor2_PWM));

    //set directions so that it goes forward
    digitalWrite(MOTOR1_A, HIGH);
    digitalWrite(MOTOR1_B, LOW);
    digitalWrite(MOTOR2_A, LOW);
    digitalWrite(MOTOR2_B, HIGH);

    //set PWM values
    analogWrite(MOTOR1_PWM, motor1_PWM);
    analogWrite(MOTOR2_PWM, motor2_PWM);
  }
}

void open_loop_go_forward(uint16_t duration) {
  //Set directions so that bug tries to go forward
  digitalWrite(MOTOR1_A, HIGH);
  digitalWrite(MOTOR1_B, LOW);
  digitalWrite(MOTOR2_A, LOW);
  digitalWrite(MOTOR2_B, HIGH);

  //set PWM values
  analogWrite(MOTOR1_PWM, 240);
  analogWrite(MOTOR2_PWM, 240);

  delay(duration);

  //stop motors
  analogWrite(MOTOR1_PWM, 0);
  analogWrite(MOTOR2_PWM, 0);
}

void open_loop_turn_right(uint16_t duration) {

  digitalWrite(MOTOR1_A, HIGH);
  digitalWrite(MOTOR1_B, LOW);
  digitalWrite(MOTOR2_A, HIGH);
  digitalWrite(MOTOR2_B, LOW);

  //set PWM values
  analogWrite(MOTOR1_PWM, 240);
  analogWrite(MOTOR2_PWM, 240);

  delay(duration);

  //stop motors
  analogWrite(MOTOR1_PWM, 0);
  analogWrite(MOTOR2_PWM, 0);
}

void open_loop_turn_right_smooth(uint16_t duration) {

  digitalWrite(MOTOR1_A, HIGH);
  digitalWrite(MOTOR1_B, LOW);
  digitalWrite(MOTOR2_A, HIGH);
  digitalWrite(MOTOR2_B, LOW);


  //set PWM values
  analogWrite(MOTOR1_PWM, 254);
  analogWrite(MOTOR2_PWM, 254);
  delay(30);

  //set PWM values
  analogWrite(MOTOR1_PWM, 220);
  analogWrite(MOTOR2_PWM, 220);

  delay(duration);

  //stop motors
  analogWrite(MOTOR1_PWM, 0);
  analogWrite(MOTOR2_PWM, 0);
}

void open_loop_go_forward_smooth(uint16_t duration) {
  //Set directions so that bug tries to go forward
  digitalWrite(MOTOR1_A, HIGH);
  digitalWrite(MOTOR1_B, LOW);
  digitalWrite(MOTOR2_A, LOW);
  digitalWrite(MOTOR2_B, HIGH);

  analogWrite(MOTOR1_PWM, 254);
  analogWrite(MOTOR2_PWM, 254);
  delay(30);

  //set PWM values
  analogWrite(MOTOR1_PWM, 240);
  analogWrite(MOTOR2_PWM, 240);

  delay(duration);

  //stop motors
  analogWrite(MOTOR1_PWM, 0);
  analogWrite(MOTOR2_PWM, 0);
}

void closed_loop_turn_right(uint8_t number_of_pulses, uint16_t pulse_duration) {
  for (uint8_t i = 0; i < number_of_pulses; i++) {
    //Turn right in open-control manner.
    digitalWrite(MOTOR1_A, HIGH);
    digitalWrite(MOTOR1_B, LOW);
    digitalWrite(MOTOR2_A, HIGH);
    digitalWrite(MOTOR2_B, LOW);

    //set PWM values
    analogWrite(MOTOR1_PWM, 240);
    analogWrite(MOTOR2_PWM, 240);

    delay(pulse_duration);

    //stop motors
    analogWrite(MOTOR1_PWM, 0);
    analogWrite(MOTOR2_PWM, 0);

    delay(1000);  //wait until the motors are stopped

    //check if black line is found, if found break. Otherwise keep continue until number of burst are complated.
    update_black_detections();
    uint8_t number_of_blacks = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (is_black[i] == 1) number_of_blacks += 1;
    }
    if (number_of_blacks > 0) break;
  }
}

void closed_loop_turn_right_v2(unsigned long timeout_ms) {

  //Turn right configuration
  digitalWrite(MOTOR1_A, HIGH);
  digitalWrite(MOTOR1_B, LOW);
  digitalWrite(MOTOR2_A, HIGH);
  digitalWrite(MOTOR2_B, LOW);


  //escape from the current line
  analogWrite(MOTOR1_PWM, 190);
  analogWrite(MOTOR2_PWM, 190);

  uint8_t number_of_blacks = 0;
  unsigned long start_time = millis();
  while (millis() - start_time < timeout_ms) {
    update_black_detections();
    for (uint8_t i = 0; i < 8; i++) {
      if (is_black[i] == 1) number_of_blacks += 1;
    }
    if (number_of_blacks == 0) break;
  }
  delay(15);  //to avoid glitching in 0&1's at the edge

  //find the next line
  while (millis() - start_time < timeout_ms) {
    update_black_detections();
    for (uint8_t i = 0; i < 8; i++) {
      if (is_black[i] == 1) number_of_blacks += 1;
    }
    if (number_of_blacks > 0) break;
  }

  delay(15);  //to ensure the line is centered

  //Apply HARD BREAK!, Turn left configuration
  digitalWrite(MOTOR1_A, LOW);
  digitalWrite(MOTOR1_B, HIGH);
  digitalWrite(MOTOR2_A, LOW);
  digitalWrite(MOTOR2_B, HIGH);
  analogWrite(MOTOR1_PWM, 240);
  analogWrite(MOTOR2_PWM, 240);
  delay(30);

  //Stop the motors
  analogWrite(MOTOR1_PWM, 0);
  analogWrite(MOTOR2_PWM, 0);
}


void test_print_is_black_array() {
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(String(is_black[i]) + " ");
  }
  Serial.println("");
}
