// MEC830 Project 2 Version 3 - Krishan Hansji
// Version 3 - Added pole placement and optimized pid control through functions

#include <Encoder.h> // include encoder library

#define directionPin 8 // direction pin for stepper
#define stepPin 9      // step pin for stepper motor
#define stepsPerRevolution 128 // steps/1 revolution

// Define pushbuttons and LEDs for HMI
#define PIDButton 4
#define poleButton 5
#define resetButton 6
#define stopButton 7
#define PIDLED 10
#define poleLED 11

// Initialize encoder
Encoder myEnc(2, 3); // Pins 2 and 3 for encoder channels A and B

// PID parameters
float Kp = 0.35, Ki = 0.01, Kd = 0.05;
float setPoint = 0; // Desired pendulum position
float input = 0, output = 0;
float prevError = 0, integral = 0;
unsigned long prevTime = 0;

// Pole Placement control variables
float polePlacementOutput = 0;

// Button states
int stopCount = 0; // Stop flag
int PIDcount = 1;  // Start in PID mode
int poleCount = 0; // Pole placement mode flag

// System constraints for safety
#define MAX_ANGLE 30 // Maximum angle before stopping
#define MIN_ANGLE -30

void setup() {
  // Set motor control pins to output
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  // Set button pins to HIGH input
  pinMode(PIDButton, INPUT_PULLUP);
  pinMode(poleButton, INPUT_PULLUP);
  pinMode(resetButton, INPUT_PULLUP);
  pinMode(stopButton, INPUT_PULLUP);

  // Set LED pins to output
  pinMode(PIDLED, OUTPUT);
  pinMode(poleLED, OUTPUT);

  // Start serial monitor
  Serial.begin(9600);

  // Reset encoder position
  myEnc.write(0);
}

void loop() {
  input = readPendulumAngle(); // Read encoder position
  

  if (input > MAX_ANGLE || input < MIN_ANGLE || stopCount == 1) { // Stop if pendulum angle exceeds thresholds
    stopMotor();
    Serial.println("Pendulum has fallen. System stopped.");
    return;
  }

  if (digitalRead(stopButton) == LOW) {
    stopMotor(); // Stop the motor
    Serial.println("System stopped. Press reset to resume.");
    stopCount = 1; // Set stop flag
    return;
  }

  if (digitalRead(resetButton) == LOW) {
    stopCount = 0;   // Reset the stop flag
    PIDcount = 1;    // Default to PID mode
    poleCount = 0;   // Reset pole placement mode
    Serial.println("System Reset.");
  }

  if (digitalRead(PIDButton) == LOW) {
    PIDcount = 1;    // Enable PID mode
    poleCount = 0;   // Disable pole placement mode
    Serial.println("PID Control Mode");
  }

  if (digitalRead(poleButton) == LOW) {
    PIDcount = 0;    // Disable PID mode
    poleCount = 1;   // Enable pole placement mode
    Serial.println("Pole Placement Control Mode");
  }

  if (PIDcount == 1) {
    digitalWrite(PIDLED, HIGH);
    digitalWrite(poleLED, LOW);
    // Run PID control
    runPIDControl();
  }

  if (poleCount == 1) {
    // Run Pole Placement control
    digitalWrite(PIDLED, LOW);
    digitalWrite(poleLED, HIGH);
    runPolePlacementControl();
  }
}

float readPendulumAngle() {
  long encoderValue = myEnc.read(); // Read encoder position in ticks
  float angle = encoderValue * (360.0 / 2400); // Convert ticks to degrees (2400 is the dual channel resolution)
  return angle;
}

void runPIDControl() {
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - prevTime) / 1000.0; // Time in seconds
  prevTime = currentTime;

  float error = setPoint - input; // Calculate error
  integral += error * elapsedTime; // Accumulates error over time
  float derivative = (error - prevError) / elapsedTime; // change in error over time
  prevError = error;
  output = Kp * error + Ki * integral + Kd * derivative; // Calculate PID output
  controlMotor(output); //set motor to PID output

  // Debug output
  Serial.print("PID - Angle: ");
  Serial.print(input);
  Serial.print(" | Output: ");
  Serial.println(output);
}

void runPolePlacementControl() {
  // Simplified pole placement
  float K1 = 15, K2 = 5; // pole gains
  float state1 = input;      // Pendulum angle (theta)
  float state2 = angularVelocity(); //pendulum angle speed (theta dot)
  polePlacementOutput = K1 * state1 + K2 * state2; // calcuate required control force based on gains, position and velocity

  controlMotor(polePlacementOutput);
  Serial.print("Pole Placement - Angle: ");
  Serial.print(input);
  Serial.print(" | Output: ");
  Serial.println(polePlacementOutput);
}

float angularVelocity() {
  static float prevAngle = 0; //set previous angle variable
  static unsigned long prevTime = 0; //set previous time variable
  float currentAngle = readPendulumAngle();//get current angle
  unsigned long currentTime = millis(); //get current time
  float angularVelocity = (currentAngle - prevAngle) / ((currentTime - prevTime) / 1000.0); //calculate anglular velocity (change in angle/change in time)
  prevAngle = currentAngle; //set previous angle to current angle
  prevTime = currentTime; //set previous time to current time
  return angularVelocity; //return velocity 
}

void controlMotor(float controlSignal) {
  // Control motor direction
  if (controlSignal > 0) {
    digitalWrite(directionPin, LOW); // Clockwise
  } else {
    digitalWrite(directionPin, HIGH); // Counterclockwise
    controlSignal = -controlSignal; // Use absolute value for speed
  }

  controlSignal = constrain(controlSignal, 0, 8); //limit PID control to 0-8 (for stability)

  int pulseDelay = map(controlSignal, 0, 8, 55, 200); // Map signal to pulse delay

  for (int i = 0; i < stepsPerRevolution; i++) { //rotate the stepper motor
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelay); //pulse delay changes the speed of the motor, higher delay = slower speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay);
  }
}

void stopMotor() { // stop motor
  digitalWrite(directionPin, LOW);
  digitalWrite(stepPin, LOW);
}
