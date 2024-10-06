#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>
#include "Wire.h"
// --------------------- Pin Definitions --------------------- //
// Servo Pin
const int SERVO_PIN = 2;
// Initial Distance Measurements
double initialFrontDistance = 0.0;
double initialLeftDistance = 0.0;
double initialRightDistance = 0.0;

// Tolerance for Matching Distances (in cm)
const double DISTANCE_TOLERANCE = 15.
// Motor Pins
const int IN1 = 4;
const int IN2 = 3;
const int ENA = 5;

// Ultrasonic Sensor Pins
const int TRIG_FRONT = 46;
const int ECHO_FRONT = 47;

const int TRIG_LEFT = 9;
const int ECHO_LEFT = 8;

const int TRIG_RIGHT = 12;
const int ECHO_RIGHT = 11;

// LED Indicator Pin
const int LED_PIN = 13;

// --------------------- Global Variables --------------------- //

// Servo Control
Servo servo;
int pos = 90; // Start at standard steering angle

// PID Control Variables
double leftDistance, rightDistance, frontDistance;
double PID_error;
double steering_angle_degrees, prev_err = 0;
const double std_steering_angle = 90.0; // standard steering angle
const double Kp = 0.08, Kd = 0.15, Ki = 0.0;
double sum_err = 0;

// Turn Tracking
int no_of_turns = 0;

// FSM States
enum State {
  FOLLOW_LINE,
  TURN_LEFT,
  TURN_RIGHT,
  RETURN_TO_START,
  STOP
};

State currentState = FOLLOW_LINE;

// HMC5883L Compass Variables
double heading = 0.0;

// Objects
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Function Prototypes
void moveForward(int speed);
void stopMotors();
void turnServo(int angle);
double PID_Controller(double err);
long readUltrasonicDistance(int trigPin, int echoPin);
void updateDistances();
void updateHeading();
double normalizeAngle(double angle);
bool hasTurnedLeft(double start, double current, double delta);
bool hasTurnedRight(double start, double current, double delta);

// --------------------- Setup Function --------------------- //
void setup() {
  Serial.begin(115200);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize Servo
  servo.attach(SERVO_PIN);
  turnServo(std_steering_angle);
  Serial.println("Servo Initialized");

  // Initialize Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Initialize Ultrasonic Sensor Pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Initialize HMC5883L Compass
  if (!mag.begin()) {
    Serial.println("Failed to initialize HMC5883L");
    while (1);
  }
  Serial.println("HMC5883L Initialized");
}

// --------------------- Main Loop --------------------- //
void loop() {
  updateDistances();
  updateHeading();
  
  Serial.print("Front Distance: ");
  Serial.print(frontDistance);
  Serial.print(" cm, Left Distance: ");
  Serial.print(leftDistance);
  Serial.print(" cm, Right Distance: ");
  Serial.print(rightDistance);
  Serial.print(" cm, Heading: ");
  Serial.print(heading);
  Serial.print("°, Current State: ");
  Serial.println(currentState);
  
  // FSM Logic
  switch (currentState) {
    case FOLLOW_LINE:
      if (frontDistance < 60) {
        /**if (leftDistance < 30 && rightDistance > 80) {
          currentState = TURN_RIGHT;
          digitalWrite(LED_PIN, HIGH); // Optional: Indicate turning right
        }
        else if (leftDistance > 80 && rightDistance < 30) {
          currentState = TURN_LEFT;
          digitalWrite(LED_PIN, HIGH); // Optional: Indicate turning left
        }*/
        if(leftDistance - rightDistance > 60){
          currentState = TURN_LEFT;
      }
          else{
            currentState = TURN_RIGHT;
          }}
      else {
        // Continue moving forward with PID steering
        moveForward(160);
        // Example PID usage (you might need to adjust based on your line-following mechanism)
        PID_error = leftDistance - rightDistance;
        steering_angle_degrees = (30 * PID_Controller(PID_error)) + std_steering_angle;
        steering_angle_degrees = constrain(steering_angle_degrees, std_steering_angle - 45, std_steering_angle + 45);
        turnServo(steering_angle_degrees);
      }
      break;

    case TURN_LEFT:
      Serial.println("Executing TURN_LEFT");
      digitalWrite(LED_PIN, HIGH);
      // Capture the starting heading
      double startHeadingLeft = heading;
      // Calculate target heading
      double targetHeadingLeft = normalizeAngle(startHeadingLeft - 42.0);
      
      // Start turning left
      moveForward(150);
      turnServo(std_steering_angle - 40);
      
      // Continuously update heading until target is reached
      while (!hasTurnedLeft(startHeadingLeft, heading, 35.0)) {
        updateHeading();
        Serial.print("Current Heading (Left Turn): ");
        Serial.println(heading);
        delay(50); // Small delay to prevent flooding the serial output
      }
      
      // Stop turning
      turnServo(std_steering_angle);
      no_of_turns++;
      currentState = FOLLOW_LINE;
      digitalWrite(LED_PIN, LOW);
      Serial.println("TURN_LEFT Completed");
      break;

    case TURN_RIGHT:
      Serial.println("Executing TURN_RIGHT");
      digitalWrite(LED_PIN, HIGH);
      // Capture the starting heading
      double startHeadingRight = heading;
      // Calculate target heading
      double targetHeadingRight = normalizeAngle(startHeadingRight + 42.0);
      
      // Start turning right
      moveForward(150);
      turnServo(std_steering_angle + 40);
      
      // Continuously update heading until target is reached
      while (!hasTurnedRight(startHeadingRight, heading, 42.0)) {
        updateHeading();
        Serial.print("Current Heading (Right Turn): ");
        Serial.println(heading);
        delay(50); // Small delay to prevent flooding the serial output
      }
      
      // Stop turning
      turnServo(std_steering_angle);
      no_of_turns++;
      currentState = FOLLOW_LINE;
      digitalWrite(LED_PIN, LOW);
      Serial.println("TURN_RIGHT Completed");
      break;

    case RETURN_TO_START: // New State Handling
      Serial.println("Executing RETURN_TO_START");
      // Check if current distances are within tolerance of initial distances
      if ( (abs(frontDistance - initialFrontDistance) <= DISTANCE_TOLERANCE) &&
           (abs(leftDistance - initialLeftDistance) <= DISTANCE_TOLERANCE) &&
           (abs(rightDistance - initialRightDistance) <= DISTANCE_TOLERANCE) ) {
        Serial.println("Returned to Start Position");
        currentState = STOP;
      }
      else {
        // Navigate towards the start position
        // Example Logic: Adjust steering based on difference from initial distances
        double errFront = initialFrontDistance - frontDistance;
        double errLeft = initialLeftDistance - leftDistance;
        double errRight = initialRightDistance - rightDistance;
        PID_error = (errLeft - errRight); // Example PID error
        steering_angle_degrees = (30 * PID_Controller(PID_error)) + std_steering_angle;
        steering_angle_degrees = constrain(steering_angle_degrees, std_steering_angle - 45, std_steering_angle + 45);
        turnServo(steering_angle_degrees);
        moveForward(150);
      }
      break;
    

    case STOP:
      Serial.println("Stopping.");
      stopMotors();
      while (1); // Halt the program
      break;

   
  }
  
  // Optional: Stop after a certain number of turns
 if (no_of_turns >= 12 && currentState != RETURN_TO_START && currentState != STOP) {
  Serial.println("Reached 12 Turns. Initiating Return to Start.");
  resetPID(); // Reset PID before starting return
  currentState = RETURN_TO_START;
}

  delay(100); // Small delay for stability
}

// --------------------- Helper Functions --------------------- //

// Function to move the robot forward at a specified speed
void moveForward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
}

// Function to stop all motors
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

// Function to turn the servo to a specific angle smoothly
void turnServo(int angle) {
  angle = constrain(angle, 0, 180);
  if (pos != angle) {
    while (pos != angle) {
      pos += (pos < angle) ? 1 : -1;
      servo.write(pos);
      delay(5);
    }
  }
}

// Simple PID controller
double PID_Controller(double err) {
  sum_err += err;
  double result = ((Kp * err) + (Ki * sum_err) + (Kd * (err - prev_err)));
  prev_err = err;
  return result;
}

// Function to read distance from an ultrasonic sensor in centimeters
long readUltrasonicDistance(int trigPin, int echoPin) {
  // Clear trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send 10µs pulse to trigPin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read echoPin
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after ~30ms
  
  // Calculate distance (speed of sound is 340 m/s)
  // distance (cm) = duration * 0.034 / 2
  long distance = duration * 0.034 / 2;
  
  // If no echo received, return a large distance
  if (duration == 0) {
    distance = 999;
  }
  
  return distance;
}

// Function to update all distance measurements
void updateDistances() {
  frontDistance = readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
  leftDistance = readUltrasonicDistance(TRIG_LEFT, ECHO_LEFT);
  rightDistance = readUltrasonicDistance(TRIG_RIGHT, ECHO_RIGHT);
}

// Function to update the heading using HMC5883L Compass
void updateHeading() {
  sensors_event_t event;
  mag.getEvent(&event);
  
  heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
  if (heading < 0) {
    heading += 360;
  }
  heading = normalizeAngle(heading);
}

// Function to normalize angles between 0 and 360 degrees
double normalizeAngle(double angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

// Function to check if the robot has turned left by the desired delta
bool hasTurnedLeft(double start, double current, double delta) {
  double target = normalizeAngle(start - delta);
  
  // Determine if the current heading has passed the target heading
  // Considering wrap-around
  if (start >= target) {
    if (current <= target || current >= start) {
      return true;
    }
    else {
      return false;
    }
  }
  else {
    // This case shouldn't occur for left turns, but included for completeness
    return current <= target;
  }
}

// Function to check if the robot has turned right by the desired delta
bool hasTurnedRight(double start, double current, double delta) {
  double target = normalizeAngle(start + delta);
  
  // Determine if the current heading has passed the target heading
  // Considering wrap-around
  if (start <= target) {
    if (current >= target || current <= start) {
      return true;
    }
    else {
      return false;
    }
  }
  else {
    // This case shouldn't occur for right turns, but included for completeness
    return current >= target;
  }
}
// Function to reset PID variables
void resetPID() {
  sum_err = 0;
  prev_err = 0;
}
