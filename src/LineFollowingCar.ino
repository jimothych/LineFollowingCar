// assigning pins
// wheel pins
const int left2 = 2;
const int left1 = 4;
const int middle = 3;
const int right1 = 5;
const int right2 = 6;

// pins for controlling motor direction
//LEFT
const int in1 = 7; // HIGH
const int in2 = 9; // LOW

//RIGHT
const int in3 = 12; // HIGH
const int in4 = 13; // LOW

// sensor pins
const int sensorPins[5] = {left2, left1, middle, right1, right2};

// obstacle pin
const int obstaclePin = 8;

// PWM pins
const int wheel_left = 10;
const int wheel_right = 11;

//global variables
unsigned long int LAST_HIGH_TIME = 0;  //global ms timestamp for middle sensor
const unsigned long int TIMEOUT_DURATION = 2000; //2 seconds
const int BASE_SPEED_RIGHT = 70; //TODO: test
const int BASE_SPEED_LEFT = 70;
bool IS_BRAKING = false; //global break flag modified by brake()

float error = 0.0;
float prevError = 0.0;
float integral = 0.0;

//tuning tutorial => https://pidexplained.com/how-to-tune-a-pid-controller/
float Kp = 25; //error constant   TODO: test
float Kd = 8.5; //motor derivative   TODO: test
float Ki = 0.0; //integral   TODO: test

void setup(){
  Serial.begin(9600); //serial on port 9600
  for (int i = 0; i < 5; i++){ //init sensors
    pinMode(sensorPins[i], INPUT);
  }
  LAST_HIGH_TIME = millis(); // initialize to current time
 
  pinMode(obstaclePin, INPUT);
  pinMode(wheel_left, OUTPUT);
  pinMode(wheel_right, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

//forward declarations for helper functions
void detectObstacle(int obstacleReading);
void checkMiddleLineSensorActive();
void brake();
float getError();
float calculatePID();

void loop(){
  //if (IS_BRAKING) { return; } //checking global brake flag for early loop exit
 
  //1. checking for obstacle
  int obstacleReading = digitalRead(obstaclePin);
  detectObstacle(obstacleReading);

  //2. checking if middle line detection sensor has been active in the past 2 seconds
  //checkMiddleLineSensorActive();

  //3. calculating PID based on previous error state
  error = getError();
  float correction = calculatePID();

  //4. determine if we need to adjust left or right
  int leftWheelPWM = 0;
  int rightWheelPWM = 0;

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
 
  //6. apply corrections
  if(error > 0) { //car veering to the left
    Serial.println("    adjusting right");
   
    leftWheelPWM = constrain(BASE_SPEED_LEFT + correction, 0, 255);
    rightWheelPWM = constrain(BASE_SPEED_RIGHT - correction, 0, 255);
   
  } else if(error < 0) { //car veering to the right
    Serial.println("    adjusting left");
   
    leftWheelPWM = constrain(BASE_SPEED_LEFT + correction, 0, 255);
    rightWheelPWM = constrain(BASE_SPEED_RIGHT - correction, 0, 255);

  } else { //no deviation
    Serial.println("    no adjustment");
   
    leftWheelPWM = BASE_SPEED_LEFT;
    rightWheelPWM = BASE_SPEED_RIGHT;
  }
 
  Serial.println(leftWheelPWM);
  Serial.println(rightWheelPWM);


  analogWrite(wheel_left,  leftWheelPWM);
  analogWrite(wheel_right, rightWheelPWM);
 
  delay(30);
}

/**
* function to detect and handle if we've hit an obstacle
* @param obstacleReading HIGH or LOW reading from distance sensor at pin 13
*/
void detectObstacle(int obstacleReading) {
  if(obstacleReading == LOW) {
    Serial.println("Obstacle detected. Stopping car.");
    brake();
  }
}

/**
* checking if any of our line detecting sensors are valid, if not we brake
* @param LAST_HIGH_TIME time in ms since last active sensor detection
*/
void checkMiddleLineSensorActive() {
  for(int i=0; i<5; i++) {
    if(digitalRead(sensorPins[i]) == LOW) { LAST_HIGH_TIME = millis(); } // LAST_HIGH_TIME is the time stamp for when the line has been detected
  }
  if(millis() - LAST_HIGH_TIME >= TIMEOUT_DURATION) {
    Serial.println("No sensor activity for 2 seconds. Stopping car.");
    otherbrake();
  }
}

/**
* force brake by writing 0 PWM to all wheel pins in main func
*/
void brake() {
  Serial.println("Braking...");
  analogWrite(wheel_left,  0);
  analogWrite(wheel_right, 0);
  delay(1000);
}

void otherbrake() {
  Serial.println("Braking...");
  analogWrite(wheel_left,  0);
  analogWrite(wheel_right, 0);
  delay(1000);
  IS_BRAKING = true;
  while (true) {
    // Infinite stop
    delay(1000);  // Optional: reduce serial flooding
    Serial.println("System halted.");
  }
}

/**
* function to calculate error value for PID calculation based on total weighted sensor deviation
* @returns prevError (global var) if no deviation found
* @returns newError if new deviation found
*/
float getError() {
  int NUM_SENSORS = 5;
  int weights[NUM_SENSORS] = {-1.825, -1, 0, 1, 1.825}; //5 is the number of sensors we have
  int sum = 0;
  int count = 0;
 
  for(int i = 0; i < NUM_SENSORS; i++) {
    if(digitalRead(sensorPins[i]) == LOW) {
      sum += weights[i];
      count++;
    }
  }
 
  if(count == 0) { return prevError; } //no deviation found
 
  float newError = (float)sum / count; //explicit cast to float
  return newError;
}

/**
* function to calculate PID
* using the global var error
* @note error > 0 means car is veering right. error < 0 means car is veering left.
* @returns correction value to be applied to motors
*/
float calculatePID() {
  integral += error; //integral is a global var, accumulates error over time
  float derivative = error - prevError;
 
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative); //PID calculation formula
 
  prevError = error; //updating state
 
  Serial.print("error: ");
  Serial.println(error);
  Serial.print("correction: ");
  Serial.println(correction);
 
  return correction;
}
