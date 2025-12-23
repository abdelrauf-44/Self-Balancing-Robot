#include <Wire.h>
#include "MPU6050.h"

// Motor control pins
// #define dir_l_1 ❓
// #define dir_l_2 ❓
// #define pwm_l ❓

// #define dir_r_1 ❓
// #define dir_r_2 ❓
// #define pwm_r ❓

// MPU6050
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Timing
unsigned long prev_time;
float delta_time;

// Orientation
float pitch_acc, roll_acc;
float pitch_gyro = 0, roll_gyro = 0;
float pitch = 0, roll = 0;
int basespeed = 0;
// PID variables
float Setpoint = 2.55;  // Balanced position
float Input = 0;
float Output = 0;
float Kp = 27.60, Ki = 380, Kd = 0.67;
float Propotional, Integral, Derivative, SUM =0; 
float integral = 0,error = 0 , last_error = 0;

void setup() {
  Wire.begin(8, 9);
  Serial.begin(9600);

  mpu.initialize();
  // if (!mpu.testConnection()) {
  //   Serial.println("MPU6050 connection failed");
  //   while (1);
  // }

  // Optional calibration offsets
  mpu.setXAccelOffset(-5820);
  mpu.setYAccelOffset(-6204);
  mpu.setZAccelOffset(8072);
  mpu.setXGyroOffset(-7);
  mpu.setYGyroOffset(-253);
  mpu.setZGyroOffset(-18);

  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(10, OUTPUT);

  prev_time = millis();
}

void loop() {
  updateIMU();
  computePID();
  adjustBalance();
  error = Setpoint - Input;
}

void updateIMU() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float axf = (float)ax;
  float ayf = (float)ay;
  float azf = (float)az;

  pitch_acc = atan2(axf, sqrt(ayf * ayf + azf * azf)) * 180 / PI;  //pitch angle calculated from accel
  roll_acc = atan2(ayf, sqrt(axf * axf + azf * azf)) * 180 / PI;   //roll angle calculated from accel

  delta_time = (millis() - prev_time) / 1000.0; //delta time
  prev_time = millis();

  float gx_f = gx / 131.0; //gyro scaling
  float gy_f = gy / 131.0; //gyro scaling

  roll_gyro = gx_f * delta_time;  //roll angle from gyro
  pitch_gyro = gy_f * delta_time; //pitch angle from gyro

  pitch = 0.96 * (pitch - pitch_gyro) + 0.04 * pitch_acc;   //complementary filter for pitch
  roll = 0.96 * (roll + roll_gyro) + 0.04 * roll_acc;  //complementary filter for roll

  Input = pitch;
  Serial.print(" Input:");
  Serial.print(Input);
  Serial.print("       speed:");
  Serial.print(basespeed);
  Serial.print("       Error:");
  Serial.println(error);
}

void computePID() {
    Propotional = error * Kp;
  
    Integral += error*delta_time*Ki;
  	
    Integral = constrain(Integral, -140, 140);
  
    Derivative = ((error- last_error)/delta_time )* Kd;
  
    SUM = Propotional + Integral + Derivative;
  
  	Output = SUM;
  
    last_error = error;
}

void adjustBalance() {
  int speedCorrection = (int)Output;
  basespeed = constrain(abs(speedCorrection),0,255);
  int speedL = basespeed;
  int speedR = basespeed;

  // Setpoint control can be updated for motion (e.g., move forward: Setpoint = -5)

  // Apply motor direction and PWM
    if (Output > 0.0) {
    digitalWrite(1, HIGH);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
    analogWrite(0, speedR);
    analogWrite(10, speedL);
  }else {
    digitalWrite(1, LOW);
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
    analogWrite(0, speedR);
    analogWrite(10, speedL);
}
}