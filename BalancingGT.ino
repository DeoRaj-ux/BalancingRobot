#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

int joystickForwards = 512;
int joystickLeft = 512;
int JOYSTICK_FORWARDS_PIN = A3;
int JOYSTICK_LEFT_PIN = A4;
bool joystickPressed = false;

const int angle_list_number = 5;
double angle_list[angle_list_number];
double targetStableAngle = 0;
double sensorAngle;
double targetAngle = 0;
double calibratedStableAngle;
double movementAnglePatch;
double turningDifferential;

double accFilterNew;
double accFilterOld = 0;

// right DC Motor H-Bridge driver control pins
double pwmRight = 12;
int in1 = 9;
int in2 = 8;
// left DC Motor H-Bridge driver control pins
double pwmLeft = 2;
int in3 = 5;
int in4 = 4;

//initial motor speeds
int Mspeed = 0;
int rightMotorSpeed = 0;
int leftMotorSpeed = 0;

//Proportional-Derivative(PD) controller variables
float kp = 0;
float ki = 0;
float kd = 0;
float proportionalValue;
float derivativeValue;
float feedback;
unsigned long timer;
float time_pre, time_step;
float pre_error = 0;
float sumError;
float diff_error = 0;
float error;


#define BNO055_SAMPLERATE_DELAY_MS (10) //sensor samples every 10ms
Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
/************************************************************************************************************************************************
 * *********************************************************************************************************************************************/
void setup() {
  pinMode(pwmRight, OUTPUT);
  pinMode(pwmLeft, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(13, OUTPUT);
  // Initially motor are disabled
  analogWrite(pwmRight, 0);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Motor left
  analogWrite(pwmLeft, 0);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  Serial.begin(115200);
  Serial.println("Start!!!");
  /* Initialise the sensor */
  bno.begin();
  //bno.setMode(bno.OPERATION_MODE_NDOF);
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(100);
  //Calibration data for BNO055 sensor
  adafruit_bno055_offsets_t calibData;

  calibData.accel_offset_x = -22;
  calibData.accel_offset_y = -12;
  calibData.accel_offset_z = -34;
  calibData.gyro_offset_x = -2;
  calibData.gyro_offset_y = -1;
  calibData.gyro_offset_z = 1;
  calibData.mag_offset_x = 680;
  calibData.mag_offset_y = 271;
  calibData.mag_offset_z = -833;
  calibData.accel_radius = 1000;
  calibData.mag_radius = 1111;

  bno.setSensorOffsets(calibData);

  int8_t temp = bno.getTemp();
  bno.setExtCrystalUse(true);
  delay(1000);
  for (int i = 0; i < 5; i++)
  {
    Serial.println("Ready...");
    delay(500);
  }
}
/************************************************************************************************************************************************
 * *********************************************************************************************************************************************/
void loop() {
  collectSensorInput();
  if (joystickPressed)
  {
    moveTargetAngle();
  }
  else
  {
    calibrateStableAngle();
  }
  correctUsingWheels();
}

/************************************************************************************************************************************************
 * *********************************************************************************************************************************************/
void collectSensorInput()
{
  joystickForwards = map(analogRead(JOYSTICK_FORWARDS_PIN), 0, 255, -127, 128); //collect joystick forwards
  joystickLeft = map(analogRead(JOYSTICK_LEFT_PIN), 0, 255, -127, 128); // collect joystick left

  // Read quaternion, convert to Euler using Adafruit library
  imu::Quaternion q = bno.getQuat();
  q.normalize();
  imu::Vector<3> euler = q.toEuler();
  euler.x() *= -180 / M_PI;
  euler.y() *= -180 / M_PI;
  euler.z() *= -180 / M_PI;
  if (euler.x() < 0)
    euler.x() += 360;
  /*
    Serial.print(F("HeadingRollPitch quat+library: "));
    Serial.print(euler.x());  // heading, nose-right is positive
    Serial.print(F(" "));
    Serial.print(euler.y());  // roll, rightwing-up is positive
    Serial.print(F(" "));
    Serial.print(euler.z());  // pitch, nose-down is positive
    Serial.println(F(""));//*/


  for (int i = 0; i < angle_list_number; i++)
  {
//    angle_list[i] = angle_list[i + 1];
    angle_list[i] = euler.z();
  }
  
  for (int i = 0; i < angle_list_number; i++)
  {
    sensorAngle += angle_list[i];
  }
  sensorAngle /= angle_list_number; //collect sensor angle

   //collect acceleration in x,y,z
  double accAngle = -atan2(acc.y() / 9.8, acc.z() / 9.8) * 57.29746936176986;
  accFilterNew = 0.9 * accFilterOld + 0.1 * accAngle;
  Serial.println(accFilterNew);

  proportionalValue = analogRead(A0); //collect controller constants (KP and KD)
  derivativeValue = analogRead(A1);
  kp = map(proportionalValue, 0, 1023, 0, 255);
  kd = map(derivativeValue, 0, 1023, 0, 100) / 10.;
  ki = 0;
  ///*
  Serial.print("Proportional(KP)=");
  Serial.print(kp);
  Serial.print(", ");
  Serial.print("Derivative(KD)=");
  Serial.print(kd);
  Serial.print(", ");//*/
}
/************************************************************************************************************************************************
 * *********************************************************************************************************************************************/

void calibrateStableAngle()
{
  if (abs(sensorAngle - targetStableAngle) < 0.3)
  {
    if (acc.y() > 0)
    {
      calibratedStableAngle += accFilterNew; //if error is greater than 0.3, and acceleration
    }                                        // on Pitch direction is greater than 0, set calibration angle
  }
}
/************************************************************************************************************************************************
 * *********************************************************************************************************************************************/

void moveTargetAngle()
{
  if (joystickForwards > 3)
  {
    movementAnglePatch = map(joystickForwards, 3, 128, 0, 3);
    error = movementAnglePatch;
    PD_feedback();

  }
  else if (joystickForwards < -3)
  {
    movementAnglePatch = map(joystickForwards, -3, -127, 0, -3);
    error = movementAnglePatch;
    PD_feedback();
  }
  else
  {
    error = 0;
    PD_feedback();
  }

  if (joystickLeft > 3)
  {
    error = map(joystickLeft, 3, 128, 0, 3);
    turningDifferential =  PD_feedback();
  }
  else if (joystickForwards < -3)
  {
    error = map(joystickForwards, -3, -127, 0, -3);
    turningDifferential = PD_feedback();
  }
  else
  {
    error = 0;
    PD_feedback();
  }
  /************************************************************************************************************************************************
   * *********************************************************************************************************************************************/
 
}

 void correctUsingWheels()
  {
    if ((sensorAngle - targetAngle) < 0.3)
    {
      wheelSpeed();
      PD_feedback();
      rightMotorSpeed = abs(feedback) - turningDifferential;
      leftMotorSpeed = abs(feedback) + turningDifferential;

      // Set Motor A forward

      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);

      // Set Motor B forward

      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);

      analogWrite(pwmRight, rightMotorSpeed);
      analogWrite(pwmLeft, leftMotorSpeed);


    }
    if ((sensorAngle - targetAngle) > -0.3)
    {
      wheelSpeed();
      PD_feedback();
      rightMotorSpeed = abs(feedback) + turningDifferential;
      leftMotorSpeed = abs(feedback) - turningDifferential;
      // Set Motor A backward

      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);

      // Set Motor B backward

      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);

      analogWrite(pwmRight, rightMotorSpeed);
      analogWrite(pwmLeft, leftMotorSpeed);
    }
  }
/************************************************************************************************************************************************
 * *********************************************************************************************************************************************/
float wheelSpeed()
{
  float targetAngle = calibratedStableAngle + movementAnglePatch;
  error = sensorAngle - targetAngle;
}
/************************************************************************************************************************************************
 * *********************************************************************************************************************************************/

float PD_feedback()
{
  timer = millis();
  int SampleTime = 1000;
  int NewSampleTime;
  double SampleTimeInSec = ((double)SampleTime) / 1000.;
  kp = kp;
  ki = ki * SampleTimeInSec;
  kd = kd / SampleTimeInSec;
  if (NewSampleTime > 0)
  {
    double ratio  = (double)NewSampleTime
                    / (double)SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long)NewSampleTime;
  }
  //timer = millis();
  time_step = (timer - time_pre);
  if (timer >= SampleTime)
  {
    sumError += error;
    diff_error = (error - pre_error);
  }
  pre_error = error;
  time_pre = timer;

  float P = kp * error;
  float I = ki * sumError;
  float D = kd * diff_error;
  float feedback = P + I + D;

  if (feedback >= 255)
  {
    feedback = 255;
  }
  else if (feedback <= -255)
  {
    feedback = -255;
  }//*/

  return feedback;
}
/************************************************************************************************************************************************
 * *********************************************************************************************************************************************/
