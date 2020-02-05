#include <PID_v1.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

const int angle_list_number = 5;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//define control pins for motors and initialise motor speed
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Motor A
double pwmR = 12;
int in1 = 9;
int in2 = 8;

// Motor B
double pwmL = 2;
int in3 = 5;
int in4 = 4;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Variables for joystick
int joyStickVertXinput = A2; 
int joyStickHorzYinput = A3; 
int Mspeed = 0;
int joyStickposVert = 512;
int joyStickposHorz = 512;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//declare and initialise some variables to determine the current angle and also to run PD control
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double Kp = 0;
double Ki = 0;
double Kd = 0;
double proportionalValue;
double derivativeValue;
double feedback;
unsigned long timer;
double time_pre, time_step;
double pre_error = 0;
double sumError;
double angle_list[angle_list_number];
double diff_error = 0;
double Setpoint, Input, Output;
int max_control = 255;
int min_control = -255;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
const int sampleRate =10;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define BNO055_SAMPLERATE_DELAY_MS (100) //sensor samples every 100ms

Adafruit_BNO055 bno = Adafruit_BNO055();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//select pinmode, check sensor, calibrate, and define the offset
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(sampleRate);
  myPID.SetOutputLimits(min_control, max_control);
  for (int i = 0; i < angle_list_number; i++)
    angle_list[i] = 0.0;
  pinMode(pwmR, OUTPUT);
  pinMode(pwmL, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(13, OUTPUT);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Start with motors disabled and direction forward
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Motor A
  analogWrite(pwmR, 0);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Motor B
  analogWrite(pwmL, 0);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

  adafruit_bno055_offsets_t calibData;

  calibData.accel_offset_x = -10;
  calibData.accel_offset_y = -17;
  calibData.accel_offset_z = -6;
  calibData.gyro_offset_x = -2;
  calibData.gyro_offset_y = 0;
  calibData.gyro_offset_z = 1;
  calibData.mag_offset_x = 66;
  calibData.mag_offset_y = 232;
  calibData.mag_offset_z = -718;
  calibData.accel_radius = 1000;
  calibData.mag_radius = 768;


  bno.setSensorOffsets(calibData);

  int8_t temp = bno.getTemp();
  bno.setExtCrystalUse(true);
  Serial.println("Hold the device upright to set offset/setpoint position.");
  delay(1000);
  for (int i = 0; i < 5; i++)
  {
    Serial.println("Ready...");
    delay(200);
  }
  //*
  int time2 = millis();

  //target this position as this desired balance point after robot startup 2 seconds later
  while ((millis() - time2) < 2000)
  {
    Setpoint = getError();
  }
  
  Serial.print("your setpoint is set at =");
  Serial.println(Setpoint);
  delay(2000);
  digitalWrite(13, HIGH);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//calculate the error between the current angle and target angle repeatedly and use PD control algorithm to feedback signal that control motors
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  double proportionalValue = analogRead(A0);
  double derivativeValue = analogRead(A1);
  joyStickposVert = analogRead(joyStickVertXinput);
  joystickposHorz = analogRead(joyStickHorzYinput);
  Kp = map(proportionalValue, 0, 1023, 0, 255);
  Kd = map(derivativeValue, 0, 1023, 0, 100) / 10.;
  Ki = 0;
  ///*
  Serial.print("Proportional(KP)=");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print("Derivative(KD)=");
  Serial.print(Kd);
  Serial.print(", ");//*/
  double error = getError()-Setpoint;
  ///*
  Serial.print("Error= ");
  Serial.print(error);
  Serial.print(", ");//*/
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.Compute();
  double feedback = Output;
 
  if (abs(error) > 15)
  {
    while (true)
    {
      // Set Motor A off

      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);

      // Set Motor B off

      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      analogWrite(pwmR, 0);
      analogWrite(pwmL, 0);
    }
  }
  motorControl(feedback);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//computes the PD feedback in motorControl function set up the condition based on control signal from controller
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void motorControl(double feedback)
{
  Mspeed1 = int(feedback);
  ///*
  Serial.print("Motor Speed=");
  Serial.println(Mspeed); //*/
  if (Mspeed <= -5)
  {
    // Set Motor A backward

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    // Set Motor B backward

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(pwmR, abs(Mspeed));
    analogWrite(pwmL, abs(Mspeed));
  }
  else if (Mspeed < 5 && Mspeed >-5)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  if(Mspeed >= 5)
  {
    // Set Motor A forward

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    // Set Motor B forward

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(pwmR, abs(Mspeed));
    analogWrite(pwmL, abs(Mspeed));
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read the raw sensor value from IMU, filter the received data using complementary filter, then, average the data and return this data
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double getError()
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /*
    //Display the doubleing point data
    Serial.print("Yaw: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tPitch: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tRoll: ");
    Serial.print(event.orientation.z, 4);
    Serial.println(""); //*/

  for (int i = 0; i < angle_list_number; i++)
  {
    angle_list[i] = angle_list[i + 1];
  }
  angle_list[angle_list_number - 1] = event.orientation.z; // event.orientation.z Produces the Kalman filtered angle and it is stored in the array angle_list
  double meanErrorAngle;
  meanErrorAngle = 0.0;

  for (int i = 0; i < angle_list_number; i++)
  {
   meanErrorAngle += angle_list[i];
  }
  meanErrorAngle /= angle_list_number;
  Input=-meanErrorAngle;
  //meanErrorAngle -= setpoint;
  //delay(BNO055_SAMPLERATE_DELAY_MS);
  return meanErrorAngle;
}
