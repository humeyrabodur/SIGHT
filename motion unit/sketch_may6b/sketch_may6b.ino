#include <QMC5883LCompass.h>

#define SHUNT_FEEDBACK_PIN A0

#define rightMotor1 2  //right motor 240RPM
#define rightMotor2 3
#define right_pwm_normalizer 1
#define rightMotorPWM 9
#define M2_R 4.7    //armature resistance of the motor
#define M2_kb 0.02  // motor back emf constant (V pser rpm), as increased, RPM is decreased

#define leftMotor1 4  //right motor 320RPM
#define leftMotor2 5
#define leftMotorPWM 10
#define M1_R 5.7    //armature resistance of the motor
#define M1_kb 0.02  // motor back emf constant (V per rpm), as increased, RPM is decreased

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
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  compass.init();
  pinMode(SHUNT_FEEDBACK_PIN, INPUT);

  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  delay(50);
  reference = return_angle();

  delay(5);
  BATTERY_VOLTAGE = return_shunt_voltage_measurement();  //assumes
  Serial.println("Battery Voltage: ~" + String(BATTERY_VOLTAGE) + "V");
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:

  //float angle = return_angle();
  //Serial.println(angle);

  // float  desired_angle=(angle+90)%360;
  //Serial.println(deviation);
  //move(reference);
  //rotate_ccw();

  align_with(reference, 4);
  unsigned long start_time = millis();
  while (millis() - start_time < 1000) {
    move(reference, 20, 4);
  }
  stopmotor();
  delay(2000);

  reference = int(reference+90)%360;


  //drive_motors_at_constant_RPM(100,100);
}

float return_shunt_voltage_measurement() {
  int reading = analogRead(A0);
  float voltage = 0.00989736 * (reading);  //V
  return voltage;
}

float return_angle() {
  //https://stackoverflow.com/questions/57120466/how-to-calculate-azimuth-from-x-y-z-values-from-magnetometer
  int MEAN_COUNT = 5;

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

void drive_motors_at_constant_RPM(float DESIRED_LEFT_RPM, float DESIRED_RIGHT_RPM) {
  uint8_t TIMEOUT_MS = 20;
  uint8_t APPLY_FULL_PERIOD = 5;  //how many miliseconds the full on/off is applied;
  int M1_approximated_rpm = 0;
  int M2_approximated_rpm = 0;

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
  float M1_rpm = 0;            // in repeats  per minute
  float M1_current_drawn = 0;  //in Amps
  float M1_back_emf = 0;       // in Volts
  float M2_rpm = 0;            // in repeats  per minute
  float M2_current_drawn = 0;  //in Amp
  float M2_back_emf = 0;       // in Volts

  unsigned long started_at = millis();
  while (millis() - started_at < TIMEOUT_MS) {
    //turn off the motors
    digitalWrite(leftMotorPWM, LOW);
    digitalWrite(rightMotorPWM, LOW);
    delayMicroseconds(500);                                                   //let the transient finish
    float voltage_measurement_both_off = return_shunt_voltage_measurement();  //voltage when both motors are off
    digitalWrite(leftMotorPWM, HIGH);
    delayMicroseconds(500);                                             // Wait for M1 transients
    float voltage_measurement_M1 = return_shunt_voltage_measurement();  //voltage when M1 is on, M2 is off
    digitalWrite(leftMotorPWM, LOW);
    digitalWrite(rightMotorPWM, HIGH);
    delayMicroseconds(500);                                             // Wait for M2 transients
    float voltage_measurement_M2 = return_shunt_voltage_measurement();  //voltage when M1 is off, M2 is on

    M1_current_drawn = 2 * (voltage_measurement_both_off - voltage_measurement_M1);  //the voltage drop is over the 0.5Ohm resistor
    M2_current_drawn = 2 * (voltage_measurement_both_off - voltage_measurement_M2);

    M1_back_emf = voltage_measurement_both_off - (M1_R * M1_current_drawn);
    M2_back_emf = voltage_measurement_both_off - (M2_R * M2_current_drawn);

    M1_rpm = M1_back_emf / M1_kb;
    M2_rpm = M2_back_emf / M2_kb;

    //if RPM < DESIRED_RPM, set motor on, otherwise set it low.
    if (M1_rpm < DESIRED_LEFT_RPM) {
      digitalWrite(leftMotorPWM, HIGH);
    } else {
      digitalWrite(leftMotorPWM, LOW);
    }
    if (M2_rpm < DESIRED_RIGHT_RPM) {
      digitalWrite(rightMotorPWM, HIGH);
    } else {
      digitalWrite(rightMotorPWM, LOW);
    }
    delay(APPLY_FULL_PERIOD);  //keep motors full on/off this many miliseconds. Due to its inertia, the speed change will be differantial.
  }

  //assume motor draws 0.6A when loaded. the steady state PWM is approximated as
  float M1_back_emf_at_desired_rpm = M1_kb * DESIRED_LEFT_RPM;
  float M2_back_emf_at_desired_rpm = M2_kb * DESIRED_RIGHT_RPM;

  float M1_voltage_applied_when_on = BATTERY_VOLTAGE - (MOTOR_IN_SERIES_RESISTANCE + M1_R) * M1_current_drawn;  //note that M2 current also effect the voltage drop on series resistance, yet it is ignored for simplicity
  float M2_voltage_applied_when_on = BATTERY_VOLTAGE - (MOTOR_IN_SERIES_RESISTANCE + M2_R) * M2_current_drawn;  //note that M2 current also effect the voltage drop on series resistance, yet it is ignored for simplicity

  //If this state is steady-state, below pwm values will more-or-less keep the motors at this state.
  int M1_PWM = constrain(int(255 * (M1_back_emf_at_desired_rpm / M1_voltage_applied_when_on)), 0, 255);
  int M2_PWM = constrain(int(255 * (M2_back_emf_at_desired_rpm / M2_voltage_applied_when_on)), 0, 255);

  analogWrite(leftMotorPWM, M1_PWM);
  analogWrite(rightMotorPWM, M2_PWM);
  delay(25);
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

void align_with(float desired_angle, int angle_margin) {
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
      delay(30);
      stopmotor();
    }
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
      delay(30);
      stopmotor();
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
