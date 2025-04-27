// assigning pins
const int left2 = 10;
const int left1 = 12;
const int middle = 4;
const int right1 = 8;
const int right2 = 7; 
const int sensorPins[5] = {left2, left1, middle, right1, right2};
const int outerSensors[4] = {left2, left1, right1, right2}; //sensors to the left and right of the middle sensor

const int obstaclePin = 13;
const int wheel_left = 11;
const int wheel_right = 9; 

//gobal variables
unsigned long int LAST_HIGH_TIME = 0;  //global ms timestamp for middle sensor
const unsigned long int TIMEOUT_DURATION = 2000; //2 seconds
const int BASE_SPEED = 80; //TODO: test
bool IS_BRAKING = false; //global break flag modified by brake()

float error = 0.0;
float prevError = 0.0;
float integral = 0.0;
//tuning tutorial => https://pidexplained.com/how-to-tune-a-pid-controller/
float Kp = 20.0; //error constant   TODO: test
float Kd = 5.0; //motor derivative   TODO: test
float Ki = 1.0; //integral   TODO: test

void setup(){
  Serial.begin(9600); //serial on port 9600
  for (int i = 0; i < 5; i++){ //init sensors
    pinMode(sensorPins[i], INPUT);
  }
  LAST_HIGH_TIME = millis(); // initialize to current time
  
  pinMode(obstaclePin, INPUT);
  pinMode(wheel_left, OUTPUT);
  pinMode(wheel_right, OUTPUT);
}

//forward declarations for helper functions
void detectObstacle(int obstacleReading);
void checkMiddleLineSensorActive();
void brake();
float getError();
float calculatePID();

//main func
void loop(){
  if (IS_BRAKING) { return; } //checking global brake flag for early loop exit
  
  //1. checking for obstacle
  // int obstacleReading = digitalRead(obstaclePin);
  // detectObstacle(obstacleReading);

  
  //2. checking if middle line detection sensor has been active in the past 2 seconds
  //checkMiddleLineSensorActive();
  
  //3. calculating PID based on previous error state
  error = getError();
  float correction = calculatePID();
  
  //4. determine if we need to adjust left or right
  //docs for constrain() => https://docs.arduino.cc/language-reference/en/functions/math/constrain/
  //we use constrain to avoid "integral windup" => https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Integral_windup
  int leftWheelPWM = 0;
  int rightWheelPWM = 0;
  
  if(error > 0) { //car veering to the right
    Serial.println("    adjusting left");
    
    leftWheelPWM = constrain(BASE_SPEED - correction, 0, 150);
    rightWheelPWM = constrain(BASE_SPEED + correction, 0, 150); 
    
  } else if(error < 0) { //car veering to the left
    Serial.println("    adjusting right");
    
    leftWheelPWM = constrain(BASE_SPEED + correction, 0, 150);
    rightWheelPWM = constrain(BASE_SPEED - correction, 0, 150); 

  } else { //no deviation
    Serial.println("    no adjustment");
    
    leftWheelPWM = BASE_SPEED;
    rightWheelPWM = BASE_SPEED; 
   
  }
  
  Serial.println(leftWheelPWM);
  Serial.println(rightWheelPWM);
  analogWrite(wheel_left,  leftWheelPWM);
  analogWrite(wheel_right, rightWheelPWM);
  
  //delay(); //for testing purposes!
}
  

/**
* function to detect and handle if we've hit an obstacle
* @param obstacleReading HIGH or LOW reading from distance sensor at pin 13
*/
void detectObstacle(int obstacleReading) {
  if(obstacleReading == HIGH) { //early guarding against obstacle
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
    if(digitalRead(sensorPins[i]) == HIGH) { LAST_HIGH_TIME = millis(); } // if ANY sensor is active, we assign last high time as the current time
  }
  if(millis() - LAST_HIGH_TIME >= TIMEOUT_DURATION) {
    Serial.println("No sensor activity for 2 seconds. Stopping car.");
    brake();
  }
}

/**
* force brake by writing 0 PWM to all wheel pins in main func
*/
void brake() {
  Serial.println("Braking...");
  analogWrite(wheel_left,  0);
  analogWrite(wheel_right, 0);
  IS_BRAKING = true;
}

/**
* function to calculate error value for PID calculation based on total weighted sensor deviation
* @returns prevError (global var) if no deviation found
* @returns newError if new deviation found
*/
float getError() {
  int NUM_SENSORS = 5;
  int weights[NUM_SENSORS] = {-2, -1, 0, 1, 2}; //5 is the number of sensors we have
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
