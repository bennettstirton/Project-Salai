#include <Wire.h> // Include Wire Library for I2C Communications
#include <Adafruit_PWMServoDriver.h> // Include Adafruit PWM Library

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
 
#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define FREQUENCY       60
#define SERVOMIN        150 // length?
#define SERVOHALF       375 // length?
#define SERVOMAX        600 // length?

// Define Potentiometer Inputs
int controlA = A0;
int controlB = A1;
int controlC = A2;
int controlD = A3;
int joyStickXpin = A4;
int joyStickYpin = A5;
 
// Define Motor Outputs on PCA9685 board
int motorA = 0;
int motorB = 1;
int motorC = 2;
int motorD = 3;  // int motorE = 4; int motorF = 5;

const int analogInPin = A0;
int sensorValue = 0;
int outputValue = 0;
uint8_t servonum = 0;
uint8_t numberOfServos = 4;

// neutral readings
int X_neutral;
int Y_neutral;

int deadzone = 10; // deadzone

void setup() {
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  Serial.begin(9600);
  pinMode(analogInPin, INPUT);
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  Serial.println("About to calibrate");
  calibrate();
  Serial.println("Calibrated!");
  X_neutral = analogRead(joyStickXpin); // joystick neutrals
  Y_neutral = analogRead(joyStickYpin); // joystick neutrals
  delay(10);
}

void calibrate(){
  int sDelay = 100;
  int lDelay = 150;
  for (servonum = 0; servonum < numberOfServos; servonum++){ // goto 90
    pwm.setPWM(servonum, 0, SERVOHALF);
    delay(50); }
  for (servonum = 0; servonum < numberOfServos; servonum++){ // 45 sweep
    pwm.setPWM(servonum, 0, (SERVOHALF + 90)); delay(sDelay); pwm.setPWM(servonum, 0, (SERVOHALF - 90)); delay(sDelay);
    pwm.setPWM(servonum, 0, 262.5); delay(sDelay); pwm.setPWM(servonum, 0, SERVOHALF); delay(sDelay); }
  for (servonum = 0; servonum < numberOfServos; servonum++){ // 45 sweep
    pwm.setPWM(servonum, 0, 487.5); delay(sDelay); pwm.setPWM(servonum, 0, SERVOHALF); delay(sDelay);
    pwm.setPWM(servonum, 0, 262.5); delay(sDelay); pwm.setPWM(servonum, 0, SERVOHALF); delay(sDelay); }
  for (servonum = 0; servonum < numberOfServos; servonum++){ // goto 90
    pwm.setPWM(servonum, 0, SERVOHALF);
    delay(50); }
}

void moveMotor(int conInA, int conInB, int conInC, int conInD, int joyStickXpin, int joyStickYpin, int motA, int motB, int motC, int motD) {

  int potA, potB, potC, potD, X, Y; // variables for analog readings
  potA = analogRead(conInA);
  potB = analogRead(conInB);
  potC = analogRead(conInC);
  potD = analogRead(conInD);
  X = constrain(analogRead(joyStickXpin), 5, 1000);
  Y = constrain(analogRead(joyStickYpin), 5, 1000);
  
  Serial.print("X: "); Serial.print(X); Serial.print(" Y: "); Serial.println(Y);
  //delay(50);
  

  int pWideA, pWideB, pWideC, pWideD; // Pulse_Wide  
  int pWidthA, pWidthB, pWidthC, pWidthD; // Pulse_Width
     
  int halfFlexion, remappedPotC;
  int F1, F2; // Flexion Variables
  
  // A -> ABDUCTION
  pWideA = map(potA, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pWidthA = int(float(pWideA) / 1000000 * FREQUENCY * 4096);

 

  // JAWS
  if (potC > 511){ //jaw open
    remappedPotC = constrain(potC, 511, 750);
    remappedPotC = map(remappedPotC, 511, 750, 0, 170);
    // Serial.print(" remappedPotC = "); Serial.println(remappedPotC);
    halfFlexion = (remappedPotC / 2); }
  else { //jaw closed
    remappedPotC = 0;
    halfFlexion = 0; }

  // F1 B -> FLEXION_1
  F1 = potB + ((potA-511)/2) + remappedPotC; // Jaw and abduction compensation
  F1 = constrain(F1, 0, 1023);
  pWideB = map(F1, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pWidthB = int(float(pWideB) / 1000000 * FREQUENCY * 4096);
  
  // F2 C -> FLEXION_2
  F2 = potB + ((potA-511)/2) - remappedPotC; // Jaw and abduction compensation
  F2 = constrain(F2, 0, 1023);
  pWideC = map(F2, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pWidthC = int(float(pWideC) / 1000000 * FREQUENCY * 4096);  

  // D -> SHAFT ROTATION
  pWideD = map(potD, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pWidthD = int(float(pWideD) / 1000000 * FREQUENCY * 4096);
  
 
  pwm.setPWM(motD, 0, pWidthD);
  pwm.setPWM(motA, 0, pWidthA);
  pwm.setPWM(motB, 0, pWidthB);
  pwm.setPWM(motC, 0, pWidthC);
  delay(50);
  
}


void loop() {
  moveMotor(controlA, controlB, controlC, controlD, joyStickXpin, joyStickYpin, motorA, motorB, motorC, motorD);
  delay(10);
}