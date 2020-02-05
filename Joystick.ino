//Credit to Mr. Bill, DB workshop
//Right Motor control pins
int pwmRight = 12;
int in1 = 9;
int in2 = 8;

// Left Motor control pins

int pwmLeft = 2;
int in3 = 5;
int in4 = 4;

// Joystick Input

int joystickVertical = A0;   
int joystickHorizontal = A1; 

// Initial speed of motors are set at zero

int rightMotorSpeed = 0;
int leftMotorSpeed = 0;

// Initial Joystick value set at middle position which is about 512

int joystickVerticalPosition = 512;
int joystickHorizontalPosition = 512;  


void setup()

{
  pinMode(pwmRight, OUTPUT);
  pinMode(pwmLeft, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
   
  // Start with motors disabled and direction forward
  // Right Motor
  digitalWrite(pwmRight, LOW);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Left Motor
  digitalWrite(pwmLeft, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.begin(9600);
}

void loop() {
  joystickVerticalPosition = analogRead(joystickVertical); 
  Serial.print(joystickVerticalPosition);
  Serial.print(" , ");
  joystickHorizontalPosition = analogRead(joystickHorizontal);
  Serial.println(joystickHorizontalPosition);
  if (joystickVerticalPosition < 460)
  {
    // In this case both the Motor are set Backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    joystickVerticalPosition = joystickVerticalPosition - 460; 
    joystickVerticalPosition = joystickVerticalPosition * -1;  

    rightMotorSpeed = map(joystickVerticalPosition, 0, 460, 0, 255);
    leftMotorSpeed = map(joystickVerticalPosition, 0, 460, 0, 255);

  }
  else if (joystickVerticalPosition > 564)
  {
    // Motor are set in forward direction

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    rightMotorSpeed = map(joystickVerticalPosition, 564, 1023, 0, 255);
    leftMotorSpeed = map(joystickVerticalPosition, 564, 1023, 0, 255); 

  }
  else
  {
    // brake

    rightMotorSpeed = 0;
    leftMotorSpeed = 0; 

  }
  
  //Left and right turn

  if (joystickHorizontalPosition < 460)
  {
    // Left turn

    joystickHorizontalPosition = joystickHorizontalPosition - 460;
    joystickHorizontalPosition = joystickHorizontalPosition * -1;  

    joystickHorizontalPosition = map(joystickHorizontalPosition, 0, 460, 0, 255);
        
    rightMotorSpeed = rightMotorSpeed + joystickHorizontalPosition;
    leftMotorSpeed = leftMotorSpeed - joystickHorizontalPosition;

    // Setting imits

    if (rightMotorSpeed < 0)rightMotorSpeed = 0;
    if (leftMotorSpeed > 255)leftMotorSpeed = 255;

  }
  else if (joystickHorizontalPosition > 564)
  {//Right turn
    joystickHorizontalPosition = map(joystickHorizontalPosition, 564, 1023, 0, 255);  

    rightMotorSpeed = rightMotorSpeed - joystickHorizontalPosition;
    leftMotorSpeed = leftMotorSpeed + joystickHorizontalPosition;

    // Don't exceed range of 0-255 for motor speeds

    if (rightMotorSpeed > 255)rightMotorSpeed = 255;
    if (leftMotorSpeed < 0)leftMotorSpeed = 0;      
  }
  // Adjust to prevent "buzzing" at very low speed
  if (rightMotorSpeed < 8)rightMotorSpeed = 0;
  if (leftMotorSpeed < 8)leftMotorSpeed = 0;

  // Enable the pins

  analogWrite(pwmRight, rightMotorSpeed);
  analogWrite(pwmLeft, leftMotorSpeed);

}
