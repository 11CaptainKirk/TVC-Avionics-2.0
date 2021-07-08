#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>

// Setup Servos
int servoYPin = 4;  // Set Servo Pin
Servo ServoY;      // Create Servo Object
int servoZPin = 5;  
Servo ServoZ;      
//

// Setup Piezo Buzzer
int piezoPin = 6;
long prevMillis = 0;
//

// Setup Button
int buttonPin = 2;
int buttonState;
int prevButtonState;
bool systemState = false;
//

// Setup IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55); // Create IMU Object
//

// Setup PID Controllers
double Setpoint;
double InputY, InputZ;
double OutputY, OutputZ;

PID PIDy(&InputY, &OutputY, &Setpoint, 1, 0, 0, DIRECT); // PID Y //TODO: Add variables to PID values.
PID PIDz(&InputZ, &OutputZ, &Setpoint, 1, 0, 0, DIRECT); // PID Z
//

void setup() {

  // Initialize Servos
  ServoY.attach(servoYPin);
  ServoZ.attach(servoZPin);
  //

  // Initialize Serial Readout
  Serial.begin(9600);
  //

  // Initialize Button
  pinMode(buttonPin, INPUT);
  //

  // Initialize IMU
  Serial.println("Orientation Sensor Test");
  Serial.println("");
  if(!bno.begin()){
    Serial.print("Oops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000); //TODO: Why this delay?
  bno.setExtCrystalUse(true);
  //

  // Initialize PID
  InputY = 0;
  InputZ = 0;
  Setpoint = 0;

  PIDy.SetMode(AUTOMATIC); // Turn PID on
  PIDz.SetMode(AUTOMATIC); 
  PIDy.SetOutputLimits(-255,255);
  PIDz.SetOutputLimits(-255,255); // Allow Negative Outputs
  PIDy.SetSampleTime(25);
  PIDz.SetSampleTime(25); // Increase update frequency (default 200)
  //
}

void loop() {

int servoHome = 90; // Set Home Position of servo (degrees)

buttonState = digitalRead(buttonPin);
if((buttonState != prevButtonState) && (buttonState == HIGH)) {
  systemState = !systemState;
  if (systemState == true){
    tone(piezoPin, 3000, 100);
    delay(100);
    tone(piezoPin, 3000, 100);
    delay(100);
    tone(piezoPin, 6000, 800);
  }  // TODO: this is messed up below but it works
  if (systemState == false){
    tone(piezoPin, 8000, 800);
    delay(100);
    tone(piezoPin, 4000, 100);
    delay(100);
    tone(piezoPin, 4000, 100);
  }
}
prevButtonState = buttonState;
Serial.print(systemState);
Serial.print("\t\t");


if(systemState == true){
// Get new sensor event
sensors_event_t event;
bno.getEvent(&event);
//

// Update inputs & compute PID
InputY = event.orientation.y;
InputZ = event.orientation.z;

PIDy.Compute();
PIDz.Compute();
//

// Move Servos based on output
if(OutputY > 10){
  ServoY.write(servoHome+20);
} else if(OutputY < -10){
  ServoY.write(servoHome-20);
} else {
  ServoY.write(servoHome+(OutputY*2));
}

if(OutputZ > 10){
  ServoZ.write(servoHome+20);
} else if(OutputZ < -10){
  ServoZ.write(servoHome-20);
} else {
  ServoZ.write(servoHome+(OutputZ*2));
}
//

// Piezo Buzzer
int interval = 1000;
if((millis()-prevMillis) > interval){
  prevMillis = millis();
  tone(piezoPin, 2000, 100);
} 
//

// Display FP data
Serial.print("X: ");
Serial.print(event.orientation.x,4);
Serial.print("\tY: ");
Serial.print(event.orientation.y,4);
Serial.print("\tZ: ");
Serial.print(event.orientation.z,4);
Serial.print("\t\tOutput Y: ");
Serial.print(OutputY,4);
Serial.print("\t\tOutput Z: ");
Serial.print(event.orientation.z,4);
Serial.println("");

} else {
  delay(25);
}
//  delay(25);


}