uint8_t analog_pins[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };  // A0-> 2 (left) A5->7 (right)Define the analog input pins, 4 D1, 5 D7
#define rightMotor1 D4                                 //right motor 240RPM
#define rightMotor2 D12
#define right_pwm_normalizer D1
#define rightMotorPWM D10
#define leftMotor1 D8  //right motor 320RPM
#define leftMotor2 D7
#define leftMotorPWM D11
#define left_pwm_normalizer 0.9
#define LED_PIN D13
#define ISRESETTED 13
#define RFID_READ D2
#define FOLLOWING_THE_LINE 1
#define AT_CROSS_SECTION 2


float last_position = 0;
uint8_t base_pwm = 90;
int intersection = 0;
uint8_t is_black[8] = { 0, 0, 0, 0, 0, 0, 0, 0};
int stop = 0;
int receivedValue = 0;
int state = 1;


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
  Serial.println("resetted!!!");
  pinMode(ISRESETTED, OUTPUT);
  digitalWrite(ISRESETTED, HIGH);
  delay(2000);
  digitalWrite(ISRESETTED, LOW);
  pinMode(LED_PIN, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(RFID_READ, INPUT);


  for (uint8_t i = 0; i < 8; i++) {
    pinMode(analog_pins[i], INPUT);
  }
}

unsigned int threshold = 900;  // Threshold for detecting black
uint8_t indicator = 0;
uint8_t oldData = 0;
uint16_t turn_delay = 1000;
unsigned long oldData_startTime = 0;


void loop() {
  
  update_black_detections(threshold);
  
  // if(check_for_cross_section()){
  //   state = AT_CROSS_SECTION;
  // }
  // else{
  //   state = FOLLOWING_THE_LINE;
  // }

  if(digitalRead(RFID_READ)==1){
    Serial.println("Read an RFID, stopping.1");
    stop_RFID();
    Serial.println("Read an RFID, stopping.");
  }

  unsigned long currentTime = millis();
  if (currentTime - oldData_startTime > 3000){oldData = 0;}
  //Checking if there is any new data from the RFID reader
  if (Serial.available() > 0) { // Check if data is available to read

    uint8_t receivedNum = Serial.read(); // Read the incoming byte
    if (receivedNum != oldData){
    Serial.print("Received: ");
    Serial.println(receivedNum); // Print the received character
    uint8_t x = int(receivedNum) / 16;
    uint8_t y = int(receivedNum) % 16;
    Serial.println(x);
    Serial.println(y);

    Serial.println("An RFID card has been read");
    if(x==1 && y==1){
      Serial.println("Starting rotating CW");
      turn_left();
    }

    if(x==1 && y==2){
      turn_left();
      Serial.println("Starting rotating CW");

    }

    if(x==1 && y==3){
      Serial.println("Starting rotating CW");
      turn_left();
    }

    if(x==2 && y==1){
      turn_left();
      Serial.println("Starting rotating CW");

    }

    if(x==2 && y==2){
      turn_left();
      Serial.println("Starting rotating CW");

    }

    if(x==2 && y==3){
      turn_left();
      Serial.println("Starting rotating CW");

    }

    oldData = receivedNum;
    oldData_startTime = millis();
    }
  }

  // if(state == AT_CROSS_SECTION)
  // {
  //   move_straight_ahead();
  //   //stop_RFID();
  // }
  // else{
  
  float line_position = get_line_pos();  // This will be deleted
  move(line_position);
  
  // }

}



void move(int line_position) {
  if (line_position == -999) {
    // Serial.println("No line is found");

    if (last_position > 0) {
      rotate_cw();
    } else {
      rotate_ccw();
    }

  }
    else if (line_position > 0) {
      int right_pwm = int(base_pwm * right_pwm_normalizer);
      int left_pwm = int((base_pwm + 1.5 * line_position * line_position) * left_pwm_normalizer);
      drive_left_motor_at(left_pwm, 30, 1);
      drive_right_motor_at(right_pwm, 30, 1);
      last_position = line_position;
    }
    else {
      line_position = -line_position;
      int right_pwm = int((base_pwm + 1.5 * line_position * line_position) * right_pwm_normalizer);
      int left_pwm = int(base_pwm * left_pwm_normalizer);
      drive_left_motor_at(left_pwm, 30, 1);
      drive_right_motor_at(right_pwm, 30, 1);
      last_position = -line_position;
    }
  }




  uint8_t update_black_detections(int black_threshold) {
    //uint8_t analog_pins[8] = {A0, A1, A2, A3, A4, A5, A6, A7}; // A0-> 2 (left) A7->7 (right)Define the analog input pins
    //right -> 2, left ->7

    for (uint8_t i = 0; i < 8; i++) {
      int analog_value = analogRead(analog_pins[i]);
      uint8_t digital_val = 0;
      if (analog_value > black_threshold) digital_val = 1;
      is_black[i] = digital_val;
    }
  }
  float get_line_pos() {
    int sensor_coefficients[8] = { 4, 3, 2, 1, -1, -2, -3, -4 };
    float pos_value = 0;
    uint8_t black_counter = 0;

    for (int i = 0; i < 8; i++) {
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
    for (uint8_t i = 0; i < 8; i++) {
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
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      analogWrite(rightMotorPWM, actual_pwm);
    } else {
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
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
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      analogWrite(leftMotorPWM, -actual_pwm);
    } else {
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
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

void rotate_cw_rfid() {
    unsigned long start_time = millis();
      while(millis()-start_time < turn_delay){
      digitalWrite(leftMotor1, LOW);
      
      // dsip(digitalRead(leftMotor1));
      digitalWrite(leftMotor2, HIGH);
      analogWrite(leftMotorPWM, base_pwm);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      analogWrite(rightMotorPWM, base_pwm);
    }
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

  void stop_RFID() {

    unsigned long start_time = millis();
    while(millis()-start_time<turn_delay){

      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, LOW);

      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, LOW);

    }

  }

  void turn_left(){
    
    while(is_black[0] || is_black[1] || is_black[2] || is_black[3] || is_black[4] || is_black[5] || is_black[6] || is_black[7]){
      
      Serial.println(get_line_pos());
      update_black_detections(threshold);
      rotate_cw();   
      
    }

    while(!(is_black[0])){
      
      Serial.println(get_line_pos());
      update_black_detections(threshold);
      rotate_cw(); 

    }

    while(!(is_black[1])){
      
      Serial.println(get_line_pos());
      update_black_detections(threshold);
      rotate_cw(); 

    }

    while(!(is_black[2])){
      
      Serial.println(get_line_pos());
      update_black_detections(threshold);
      rotate_cw(); 

    }

    while(!(is_black[3])){
      
      Serial.println(get_line_pos());
      update_black_detections(threshold);
      rotate_cw(); 

    }

  }

  bool check_for_cross_section(){
    uint8_t black_counter = 0;
    for(uint8_t i = 0; i<8; i++){
      if(is_black[i] == 1){
        black_counter++;
      }
    }
    if(black_counter>3){
      return 1;
    }
    else{
      return 0;
    }
    
  }

  void move_straight_ahead(){
     
    //drive_left_motor_at(100*right_pwm_normalizer, 30, 1);
    //drive_right_motor_at(100*left_pwm_normalizer, 30, 1);
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      analogWrite(leftMotorPWM, 100*left_pwm_normalizer);

      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      analogWrite(rightMotorPWM, 100*right_pwm_normalizer);

  }

  