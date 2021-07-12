#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <Adafruit_BMP3XX.h>
#include <SD.h>
#include <SPI.h>

// Setup Servos
short servoYPin = 4;  // Set Servo Pin
Servo ServoY;      // Create Servo Object
short servoZPin = 5;  
Servo ServoZ;      
//

// Setup Piezo Buzzer
short piezoPin = 6;
long prevMillis = 0;
//

// Setup Button
short buttonPin = 2;
short buttonState;
short prevButtonState;
bool systemState = false;
//

// Setup Barometer
float seaPressure = 1013.25;
Adafruit_BMP3XX bmp;
//

// Setup IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55); // Create IMU Object
//

File currFlightData;

// Setup PID Controllers
double Setpoint;
double InputY, InputZ;
double OutputY, OutputZ;

PID PIDy(&InputY, &OutputY, &Setpoint, 1, 0, 0, DIRECT); // PID Y //TODO: Add variables to PID values.
PID PIDz(&InputZ, &OutputZ, &Setpoint, 1, 0, 0, DIRECT); // PID Z
//

void setup() {

  pinMode(3, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);
  digitalWrite(3, HIGH);

  // Initialize Servos
  ServoY.attach(servoYPin);
  ServoZ.attach(servoZPin);
  //

  // Initialize Serial Readout
  Serial.begin(9600);
  while (!Serial) {
  ;
  }
  //

  // Initalize SD Card
  Serial.print("SD Card...");
  if (!SD.begin(10)) {
    Serial.println("Initialization failed!");
    while(1);
  }
  Serial.println("Initialization Done.");
  //

  // Initialize Button
  pinMode(buttonPin, INPUT);
  //

  // Initialize IMU
  Serial.println("Orientation Sensor Test");
  Serial.println("");
  if(!bno.begin()){
    Serial.print("** BNO055 not detected **\t\t<<");
    while(1);
  }
  delay(1000); //TODO: Why this delay?
  bno.setExtCrystalUse(true);
  //

  // Initialize Barometer
  if (!bmp.begin_I2C()){
    Serial.println("** BMP388 not detected **\t\t<<");
    while(1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_1_5_HZ);
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

Serial.println("looping");
short servoHome = 90; // Set Home Position of servo (degrees)

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
digitalWrite(A2, HIGH);
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
short interval = 1000;
if((millis()-prevMillis) > interval){
  prevMillis = millis();
  tone(piezoPin, 4000, 100);
} 
//

float Xorient = event.orientation.x;

// Write to SD Card
currFlightData = SD.open("currflight.csv", FILE_WRITE);
if (currFlightData) {
  Serial.print("TEST Writing,");
  Serial.println("This is a test.");
  currFlightData.close();
} else {
  int interval2 = 100;
  if((millis()-prevMillis) > interval2){
  prevMillis = millis();
  tone(piezoPin, 4000, 50);
} 
}
//

// Display FP data       (Parts commented out because they interfere with performance)
Serial.print(millis());

//Serial.print("\t\tX: ");
Serial.print(Xorient,4);
//Serial.print("\tY: ");
Serial.print(InputY,4);
//Serial.print("\tZ: ");
Serial.print(InputZ,4);
//Serial.print("\t\tOutput Y: ");
Serial.print(OutputY,4);
//Serial.print("\t\tOutput Z: ");
Serial.print(OutputZ,4);

//Serial.print("\tTemperature = ");
  Serial.print(bmp.temperature);
  //Serial.print(" *C");

  //Serial.print("\tPressure = ");
  Serial.print(bmp.pressure / 100.0);
  //Serial.print(" hPa");

  //Serial.print("\tApprox. Altitude = ");
  Serial.print(bmp.readAltitude(seaPressure));
  //Serial.print(" m");
Serial.println("");

} else {
  delay(25);
  digitalWrite(A2, LOW);
}
//  delay(25);


}