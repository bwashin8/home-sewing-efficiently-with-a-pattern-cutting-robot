#include <Servo.h> // library for Servo Motor

// IR Sensors
int sensor1 = 12;  // Left sensor connected to digital pin 12
int sensor2 = 4;   // Middle sensor connected to digital pin 4
int sensor3 = 13;  // Right sensor connected to digital pin 13

int sensor[3] = {0, 0, 0};  // Initial Values of Sensors in an integer array

// Motor Variables/ Digital Pins Connections
int ENA = 5; 
int ENB = 6;
int motorInput1 = 7; 
int motorInput2 = 8;
int motorInput3 = 9;
int motorInput4 = 11;

Servo armServo;  //Servo Motor used for arm mechanism
int initial_speed = 160; //Initial Speed of Motor

// PID Constants
float Kp = 65;
float Ki = 0;
float Kd = 45;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;


void setup()
{
  //setting sensors as inputs
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);

//setting motors as outputs
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  armServo.attach(3);  //setting pin 3 for servo
  
  Serial.begin(9600);  //setting serial monitor at a default baund rate of 9600
  delay(500); // delay for half a second
  Serial.println("Started !!");
  delay(1000); // delay for one second
  armServo.write(80); // blade is lowered at the start of operation
  delay(3000); // delay for three seconds
  read_sensor_values(); // read the input values of all IR modules

}
void loop()
{

  read_sensor_values(); // read the input values of all IR modules
  
  if (error == 1 && error != 0) // this executes when there is a left turn detected. 
  {     
       //raises arm and the robot stops to prepare to turn
       stop_bot();
       armServo.write(105);
       delay(3000);
       
  do 
    {
      // the car reverses until it detects the point of intersection
      read_sensor_values();
      analogWrite(ENA, 115); //Left Motor Speed
      analogWrite(ENB, 115); //Right Motor Speed
      reverse();
      
    } 
    while (error != 1 && error != 2); //do this while no line is detected to the left or right
  
 
    stop_bot();
    delay(3000);
    forward();// goes forward a little bit to conpensate for the sharpness of the turn
    delay(420);

  // turn left until a straight line is detected
    do 
    {
      //
      read_sensor_values();
      analogWrite(ENA, 170); //Left Motor Speed
      analogWrite(ENB, 170); //Right Motor Speed
      left(); 
      
      
    } 
    while (error != 0); // do this while no line is detected by the middle sensor
    
      stop_bot();
      delay(3000);
      
  // reverses until it sensor the t-intersection
  do 
    {
      
      read_sensor_values();
      analogWrite(ENA, 105); //Left Motor Speed
      analogWrite(ENB, 105); //Right Motor Speed
      reverse();

    } 
    while (sensor[2] == 0); // reverse while no line is detected to the right
    
    // arm is let down and robot goes straight 
    stop_bot();
    delay(3000);
    armServo.write(80);
    delay(3000);
   

  } 

 else if (error == 2 && error != 0) // this executes when there is a right turn detected. 
  {     
       //raises arm and the robot stops to prepare to turn
       stop_bot();
       armServo.write(105);
       delay(3000);
       
  do 
    {
      // the car reverses until it detects the point of intersection
      read_sensor_values();
      analogWrite(ENA, 115); //Left Motor Speed
      analogWrite(ENB, 115); //Right Motor Speed
      reverse();
      
    } 
    while (error != 1 && error != 2); //do this while no line is detected to the left or right
  
 
    stop_bot();
    delay(3000);
    forward();// goes forward a little bit to conpensate for the sharpness of the turn
    delay(420);

  // turn left until a straight line is detected
    do 
    {
      //
      read_sensor_values();
      analogWrite(ENA, 170); //Left Motor Speed
      analogWrite(ENB, 170); //Right Motor Speed
      right(); 
      
      
    } 
    while (error != 0); // do this while no line is detected by the middle sensor
    
      stop_bot();
      delay(3000);
      
  // reverses until it sensor the t-intersection
  do 
    {
      
      read_sensor_values();
      analogWrite(ENA, 105); //Left Motor Speed
      analogWrite(ENB, 105); //Right Motor Speed
      reverse();

    } 
    while (sensor[2] == 0); // reverse while no line is detected to the right
    
    // arm is let down and robot goes straight 
    stop_bot();
    delay(3000);
    armServo.write(80);
    delay(3000);
   

  } 
 
  else // only carry out the PID control when the robot moves forward since that is when the robot is cutting
  {
    calc_pid(); // calls calc_pid function that calculates error
    motor_control(); // calls motor_control function 
  }
 
}

void read_sensor_values()
{
  sensor[0] = digitalRead(sensor1); //store input value of sensor 1 into the first index of the sensor array
  sensor[1] = digitalRead(sensor2); //store input value of sensor 2 into the second index of the sensor array
  sensor[2] = digitalRead(sensor3); //store input value of sensor 3 into the last index of the sensor array

    Serial.print(sensor[0]); // print value of IR sensor at first index
    Serial.print("\t");
    Serial.print(sensor[1]); // print value of IR sensor at second index
    Serial.print("\t");
    Serial.print(sensor[2]); // print value of IR sensor last index

       if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0)) // no line dectected
       {
         error = -1;
       }
       else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0)) // line has been detected on left but there is a straight line ahead therefore go straight instead of turning
       {
         error = 0;
       }
       else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)) // line has been detected in either direction, but go straight instead
       {
         error = 0;
       }
       else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1)) // lline has been detected on right but there is a straight line ahead therefore go straight instead of turning
       {
         error = 0;
       }
       else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0)) // line is straight ahead
       {
         error = 0;
       }
       else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0)) // line had been detected on left, turn left
       {
          error = 1;
       }
       else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1)) // line has come to a turn but line to the right detected, still take it as there is a line to the left, turn left
       {
          error = 1;
       }

}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_speed - PID_value;
  int right_motor_speed = initial_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  analogWrite(ENA, left_motor_speed - 50); //Left Motor Speed minus 50 because my robot stears towards the right so it straightens it up 
  analogWrite(ENB, right_motor_speed); //Right Motor Speed

  //moves the robot foward and helps control the robot when it is going forward
  forward();
}


//This function calucates past and present errors while also anticipating future errors to make the robot move as smoothly as possible
void calc_pid()
{
  P = error; 
  I = I + previous_I; 
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}



void forward()
{
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
  Serial.println("go forward!");
}
void reverse()
{
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
  Serial.println("go back!");
}
void right()
{
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
  Serial.println("go right!");
}
void left()
{
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
  Serial.println("go left!");
}

void stop_bot()
{
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
  Serial.println("stop!");
}
