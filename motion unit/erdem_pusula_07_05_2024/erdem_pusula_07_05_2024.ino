#include <QMC5883LCompass.h>

#define SHUNT_FEEDBACK_PIN A0

#define rightMotor1 2  //right motor 240RPM
#define rightMotor2 3
#define right_pwm_normalizer 1
#define rightMotorPWM 9
#define right_R 4.65   //armature resistance of the motor
#define right_kb 0.02  // motor back emf constant (V pser rpm), as increased, RPM is decreased

#define leftMotor1 4  //right motor 320RPM
#define leftMotor2 5
#define leftMotorPWM 10
#define left_R 4.65   //armature resistance of the motor
#define left_kb 0.02  // motor back emf constant (V per rpm), as increased, RPM is decreased

#define MOTOR_IN_SERIES_RESISTANCE 0.5  // The resitance in series with the H bridge. (i.e. ~shunt resistor)

#define left_pwm_normalizer 1
#define threshold = 3;
uint8_t base_pwm = 100;
QMC5883LCompass compass;
float reference;
float return_angle();
void rotate_ccw();
void rotate_cw();


float BATTERY_VOLTAGE = 0;  //measured only once at the start-up

float right_angles[4] = { 0, 90, 180, 270 };
void calibrate_right_angles() {
  //vibrate
  rotate_ccw();
  delay(50);
  rotate_cw();
  delay(50);
  stopmotor();
  delay(3500);
  right_angles[0] = return_angle();

  //vibrate
  rotate_ccw();
  delay(50);
  rotate_cw();
  delay(50);
  stopmotor();
  delay(3500);
  right_angles[1] = return_angle();

  //vibrate
  rotate_ccw();
  delay(50);
  rotate_cw();
  delay(50);
  stopmotor();
  delay(3500);
  right_angles[2] = return_angle();

  //vibrate
  rotate_ccw();
  delay(50);
  rotate_cw();
  delay(50);
  stopmotor();
  delay(3500);
  right_angles[3] = return_angle();

  //vibrate
  rotate_ccw();
  delay(50);
  rotate_cw();
  delay(50);
  stopmotor();
  delay(3500);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  compass.init();
  pinMode(SHUNT_FEEDBACK_PIN, INPUT);

  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

  calibrate_right_angles();

  BATTERY_VOLTAGE = return_shunt_voltage_measurement();  //assumes
  delay(3000);
  reference = return_angle();

  Serial.println("Battery Voltage: ~" + String(BATTERY_VOLTAGE) + "V");
}

float desired_rpm = 150;
uint8_t min_pwm = 70;
uint8_t max_pwm = 150;

float del_rpm_bound = 0.25 * desired_rpm;
float final_x_pos = 0;
float final_y_pos = 0;
float percentage_kp = 0.025 * desired_rpm;
int left_pwm = 0;
int right_pwm = 0;
int pwm_increment = 3;
int pwm_decrement = 3;

float M_PER_SEC_RPM = (0.2198) / 60.0;
uint16_t iteration_duration_ms = 15;
float iteration_duration = iteration_duration_ms / 1000.0;

uint8_t counter = 0;
void loop() {
  //align with 0 angle
  reference = right_angles[counter];
  counter = counter + 1;
  if (counter > 3) {
    counter = 0;
  }
  delay(1500);
  align_with(reference, 3);
  delay(1500);

  left_pwm = min_pwm;
  right_pwm = min_pwm;

  float distance_mag = 0;
  float distance_x = 0;
  float distance_y = 0;
  unsigned long start_time = millis();
  while (millis() - start_time < 7500) {
    // if deviation is positive slow down left motor.
    float *motor_speeds = return_motor_speeds(left_pwm, right_pwm);
    Serial.println("Left: " + String(motor_speeds[0]) + " RPM, Right: " + String(motor_speeds[1]) + " RPM" + " Left PWM: " + String(left_pwm) + " Right PWM: " + String(right_pwm));

    float angle_deviation = return_angle_deviation(reference);
    float radian_deviation = angle_deviation * (0.017453);
    float common_speed = (motor_speeds[0] + motor_speeds[1]) / 2;
    distance_mag = distance_mag + common_speed * M_PER_SEC_RPM * iteration_duration;
    distance_x = distance_mag * sin(radian_deviation);  //term with sine
    distance_y = distance_mag * cos(radian_deviation);  //term with cosine
    Serial.println("Distance: " + String(distance_mag) + ", Distance X: " + String(distance_x) + ", Distance Y: " + String(distance_y));
    if (distance_y > 0.5) break;

    float del_rpm = percentage_kp * angle_deviation;
    if (del_rpm < -del_rpm_bound) {
      del_rpm = -del_rpm_bound;
    } else if (del_rpm > del_rpm_bound) {
      del_rpm = del_rpm_bound;
    }
    float left_desired_speed = desired_rpm - del_rpm;
    float right_desired_speed = desired_rpm + del_rpm;

    if (motor_speeds[0] < left_desired_speed) {
      left_pwm = left_pwm + pwm_increment;
      if (left_pwm > max_pwm) left_pwm = max_pwm;
    } else {
      left_pwm = left_pwm - pwm_decrement;
      if (left_pwm < min_pwm) left_pwm = min_pwm;
    }
    if (motor_speeds[1] < right_desired_speed) {
      right_pwm = right_pwm + pwm_increment;
      if (right_pwm > max_pwm) right_pwm = max_pwm;
    } else {
      right_pwm = right_pwm - pwm_decrement;
      if (right_pwm < min_pwm) right_pwm = min_pwm;
    }


    free(motor_speeds);  //free the allocated memory;
    delay(iteration_duration_ms);
  }

  //slow down the motors
  stopmotor();
  delay(1000);






  // put your main code here, to run repeatedly:

  //float angle = return_angle();
  //Serial.println(angle);

  // float  desired_angle=(angle+90)%360;
  //Serial.println(deviation);
  //move(reference);
  //rotate_ccw();

  //align_with(0, 4);


  //drive_motors_at_constant_RPM(100,100);
}

float return_shunt_voltage_measurement() {
  int reading = analogRead(A0);
  float voltage = 0.00989736 * (reading);  //V
  return voltage;
}

float return_angle() {
  //https://stackoverflow.com/questions/57120466/how-to-calculate-azimuth-from-x-y-z-values-from-magnetometer
  int MEAN_COUNT = 10;

  int x = 0;
  int y = 0;
  int z = 0;

  // Read compass values
  compass.read();
  // Precompute the tilt compensation parameters to improve readability.
  float phi = 0;    //TODO: needs gyro to dynamically calculate
  float theta = 0;  //TODO: needs gyro to dynamically calculate

  float bearing = 0;
  for (uint8_t i = 0; i < MEAN_COUNT; i++) {
    x = compass.getX();
    y = compass.getY();
    z = compass.getZ();


    // Precompute cos and sin of pitch and roll angles to make the calculation a little more efficient.
    float sinPhi = sin(phi);
    float cosPhi = cos(phi);
    float sinTheta = sin(theta);
    float cosTheta = cos(theta);

    // Calculate the tilt compensated bearing, and convert to degrees.
    bearing += (360 * atan2(x * cosTheta + y * sinTheta * sinPhi + z * sinTheta * cosPhi, z * sinPhi - y * cosPhi)) / (2 * PI);
  }
  bearing = bearing / MEAN_COUNT;

  // Ensure the calculated bearing is in the 0..359 degree range.
  if (bearing < 0)
    bearing += 360.0f;
  else if (bearing > 360) {
    bearing -= 360.0f;
  }

  return bearing;
}

float return_angle_deviation(float desired_angle) {
  float angle_now = return_angle();
  // Calculate the deviation
  float angle_deviation = desired_angle - angle_now;

  // Normalize the deviation to be within the range [-180, 180)
  if (angle_deviation >= 180) {
    angle_deviation = 360 - angle_deviation;
  } else if (angle_deviation < -180) {
    angle_deviation += 360;
  }
  return angle_deviation;
}

void drive_motors_at_constant_RPM(float DESIRED_LEFT_RPM, float DESIRED_RIGHT_RPM) {
  uint8_t TIMEOUT_MS = 20;
  uint8_t APPLY_FULL_PERIOD = 5;  //how many miliseconds the full on/off is applied;
  int left_approximated_rpm = 0;
  int right_approximated_rpm = 0;

  // Set the directions of the motors
  if (DESIRED_LEFT_RPM < 0) {
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
  } else {
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
  }
  if (DESIRED_RIGHT_RPM < 0) {
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
  } else {
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
  }

  DESIRED_LEFT_RPM = abs(DESIRED_LEFT_RPM);    //since direcion information is already utilized, only the magnitude is required.
  DESIRED_RIGHT_RPM = abs(DESIRED_RIGHT_RPM);  //since direcion information is already utilized, only the magnitude is required.

  //try to converge to the desired RPMs
  float left_rpm = 0;             // in repeats  per minute
  float left_current_drawn = 0;   //in Amps
  float left_back_emf = 0;        // in Volts
  float right_rpm = 0;            // in repeats  per minute
  float right_current_drawn = 0;  //in Amp
  float right_back_emf = 0;       // in Volts

  unsigned long started_at = millis();
  while (millis() - started_at < TIMEOUT_MS) {
    //turn off the motors
    digitalWrite(leftMotorPWM, LOW);
    digitalWrite(rightMotorPWM, LOW);
    delayMicroseconds(500);                                                   //let the transient finish
    float voltage_measurement_both_off = return_shunt_voltage_measurement();  //voltage when both motors are off
    digitalWrite(leftMotorPWM, HIGH);
    delayMicroseconds(500);                                               // Wait for left transients
    float voltage_measurement_left = return_shunt_voltage_measurement();  //voltage when left is on, right is off
    digitalWrite(leftMotorPWM, LOW);
    digitalWrite(rightMotorPWM, HIGH);
    delayMicroseconds(500);                                                // Wait for right transients
    float voltage_measurement_right = return_shunt_voltage_measurement();  //voltage when left is off, right is on

    left_current_drawn = 2 * (voltage_measurement_both_off - voltage_measurement_left);  //the voltage drop is over the 0.5Ohm resistor
    right_current_drawn = 2 * (voltage_measurement_both_off - voltage_measurement_right);

    left_back_emf = voltage_measurement_both_off - (left_R * left_current_drawn);
    right_back_emf = voltage_measurement_both_off - (right_R * right_current_drawn);

    left_rpm = left_back_emf / left_kb;
    right_rpm = right_back_emf / right_kb;

    //if RPM < DESIRED_RPM, set motor on, otherwise set it low.
    if (left_rpm < DESIRED_LEFT_RPM) {
      digitalWrite(leftMotorPWM, HIGH);
    } else {
      digitalWrite(leftMotorPWM, LOW);
    }
    if (right_rpm < DESIRED_RIGHT_RPM) {
      digitalWrite(rightMotorPWM, HIGH);
    } else {
      digitalWrite(rightMotorPWM, LOW);
    }
    delay(APPLY_FULL_PERIOD);  //keep motors full on/off this many miliseconds. Due to its inertia, the speed change will be differantial.
  }

  //assume motor draws 0.6A when loaded. the steady state PWM is approximated as
  float left_back_emf_at_desired_rpm = left_kb * DESIRED_LEFT_RPM;
  float right_back_emf_at_desired_rpm = right_kb * DESIRED_RIGHT_RPM;

  float left_voltage_applied_when_on = BATTERY_VOLTAGE - (MOTOR_IN_SERIES_RESISTANCE + left_R) * left_current_drawn;     //note that right current also effect the voltage drop on series resistance, yet it is ignored for simplicity
  float right_voltage_applied_when_on = BATTERY_VOLTAGE - (MOTOR_IN_SERIES_RESISTANCE + right_R) * right_current_drawn;  //note that right current also effect the voltage drop on series resistance, yet it is ignored for simplicity

  //If this state is steady-state, below pwm values will more-or-less keep the motors at this state.
  int left_PWM = constrain(int(255 * (left_back_emf_at_desired_rpm / left_voltage_applied_when_on)), 0, 255);
  int right_PWM = constrain(int(255 * (right_back_emf_at_desired_rpm / right_voltage_applied_when_on)), 0, 255);

  analogWrite(leftMotorPWM, left_PWM);
  analogWrite(rightMotorPWM, right_PWM);
  delay(25);
}

void align_with(float desired_angle, int angle_margin) {
  float angle_error = return_angle_deviation(desired_angle);
  float pulse_duration = 20 + abs(angle_error) * 2;
  if (pulse_duration > 75) pulse_duration = 75;
  if (angle_error <= -angle_margin) {
    while (true) {
      float zaa2 = return_angle_deviation(desired_angle);
      if (zaa2 > 0) {
        break;
      }
      stopmotor();
      delay(25);
      rotate_cw();
      delay(pulse_duration);
      stopmotor();
      delay(25);
    }
  } else if (angle_error >= angle_margin) {
    while (true) {
      float zaa = return_angle_deviation(desired_angle);
      if (zaa < 0) {
        break;
      }
      stopmotor();
      delay(25);
      rotate_ccw();
      delay(pulse_duration);
      stopmotor();
      delay(50);
    }
  }
}

void move(float desired_angle, int angle_margin, float kp) {

  float angle_error = return_angle_deviation(desired_angle);

  if (angle_error <= -angle_margin) {

    while (true) {
      float zaa2 = return_angle_deviation(desired_angle);
      Serial.println("yiğit" + String(zaa2));
      if (zaa2 > 0) {
        break;
      }
      stopmotor();
      delay(25);
      rotate_cw();
      delay(40);
      stopmotor();
    }

  } else if (-angle_margin < angle_error && angle_error < angle_margin) {

    angle_error = -angle_error;
    int right_pwm = int((base_pwm + kp * angle_error));
    int left_pwm = int((base_pwm + kp * angle_error));
    drive_motors_at_constant_RPM(left_pwm, right_pwm);

    Serial.println("berfin" + String(angle_error));
  } else if (angle_error >= angle_margin) {
    while (true) {
      float zaa = return_angle_deviation(desired_angle);
      if (zaa < 0) {
        break;
      }
      Serial.println("erdem " + String(zaa));
      stopmotor();
      delay(25);
      rotate_ccw();
      delay(40);
      stopmotor();
    }
  }
}

float *return_motor_speeds(uint8_t left_motor_exit_PWM, uint8_t right_motor_exit_PWM) {
  //turn off the motors
  digitalWrite(leftMotorPWM, LOW);
  digitalWrite(rightMotorPWM, LOW);
  delayMicroseconds(1000);                                                  //let the transient finish
  float voltage_measurement_both_off = return_shunt_voltage_measurement();  //voltage when both motors are off
  digitalWrite(leftMotorPWM, HIGH);
  delayMicroseconds(1000);                                              // Wait for left transients
  float voltage_measurement_left = return_shunt_voltage_measurement();  //voltage when left is on, right is off
  digitalWrite(leftMotorPWM, LOW);
  digitalWrite(rightMotorPWM, HIGH);
  delayMicroseconds(1000);                                               // Wait for right transients
  float voltage_measurement_right = return_shunt_voltage_measurement();  //voltage when left is off, right is on

  // Set the directions of the motors and the default speeds
  if (leftMotorPWM < 0) {
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
  } else {
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
  }
  if (rightMotorPWM < 0) {
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
  } else {
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
  }
  analogWrite(leftMotorPWM, left_motor_exit_PWM);
  analogWrite(rightMotorPWM, right_motor_exit_PWM);

  float left_current_drawn = 2 * (voltage_measurement_both_off - voltage_measurement_left);  //the voltage drop is over the 0.5Ohm resistor
  float right_current_drawn = 2 * (voltage_measurement_both_off - voltage_measurement_right);

  float left_back_emf = voltage_measurement_both_off - (left_R * left_current_drawn);
  float right_back_emf = voltage_measurement_both_off - (right_R * right_current_drawn);

  float left_rpm = left_back_emf / left_kb;
  float right_rpm = right_back_emf / right_kb;

  float *rpm_vals = malloc(2 * sizeof(float));
  if (rpm_vals == NULL) {
    // Handle memory allocation failure
    return NULL;
  }
  rpm_vals[0] = left_rpm;
  rpm_vals[1] = right_rpm;

  return rpm_vals;
}


void drive_right_motor_at(int pwm_val, int update_period_ms, uint8_t delta_pwm_per_period) {
  static unsigned long last_time_update = 0;
  static uint16_t actual_pwm = 0;
  if (millis() - last_time_update < update_period_ms) return;
  last_time_update = millis();

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
  static uint16_t actual_pwm = 0;
  if (millis() - last_time_update < update_period_ms) return;
  last_time_update = millis();

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

void rotate_ccw() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, base_pwm);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, base_pwm);
}
void rotate_cw() {
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, base_pwm);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, base_pwm);
}
void stopmotor() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, base_pwm);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, base_pwm);
}
