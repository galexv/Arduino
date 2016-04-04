#include <Wire.h> // I2C library, required for MLX90614
#include <SparkFunMLX90614.h> // SparkFunMLX90614 Arduino library

#include <Adafruit_MotorShield.h>

#include <Servo.h>

/* Constants */

const byte LED_PIN = 13; // LED pin for debugging

// Temperature meaning "no temperature available"
const float NO_SUCH_TEMP=-10000;

const float COLD_SPOT_THRESH=5; // degrees F

const byte ROBOT_SPEED=64; // how fast we want to drive
const int NUMBER_OF_STEPS=10; // how many steps go before turning around
const int STEP_TIME=500; // how long is one step, in ms

const int MARKER_SERVO_PIN=9;
const int MARKER_LIFTED_POS=0; // servo position for the marker lifted
const int MARKER_LOWERED_POS=90; // servo position for the marker lowered

/* Temperature measurements */

IRTherm Therm; // Create an IRTherm object to interact with throughout

// (Calibrated) temperature of a "warm/normal" wall
float Warm_Wall_T=NO_SUCH_TEMP; 

/* Motor control */

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(3);

byte CurrentDir=FORWARD;

/* Turn-around control */
int StepsToGo=NUMBER_OF_STEPS;

/* Marker/servo control */
Servo MarkerServo; // object to control marker servo
bool Marker_is_up=true; // whether marker is lifted or lowered


// If warm-wall T is not calibrated, measure the current temperature and set to it
// If cannot calibrate, return false
bool calibrate_Warm_T()
{
  // Serial.println("Entering calibration"); delay(100);
  // If already calibrated, nothing to do:
  if (Warm_Wall_T != NO_SUCH_TEMP) {
    // Serial.println("Calibration: Already calibrated"); 
    return true;
  }
  
  // Serial.println("Calibration: measuring"); delay(100);
  // Let's measure
  if (!Therm.read()) {
    // Serial.println("Calibration: Could not read"); 
    return false;
  }

  //Serial.println("Calibration: measured OK"); delay(100);
  otemp=Therm.object();
  atemp=Therm.ambient();
  if (otemp>atemp) return false; // object is too warm, don't calibrate
  Warm_Wall_T=otemp; // remember the wall temperature
  return true;
}

// Set speed of both of our motors
void setSpeed(byte speed)
{
  LeftMotor->setSpeed(speed);
  RightMotor->setSpeed(speed);  
}

// Set direction and run both of our motors
void runMotors(byte dir)
{
  LeftMotor->run(dir);
  RightMotor->run(dir);  
}

// Determine if it's time to turn around
bool timeToTurn()
{
  --StepsToGo;
  if (StepsToGo==0) {
    StepsToGo=NUMBER_OF_STEPS;
    return true;
  } else {
    return false;
  }
}

// Lower the marker
void markerDown()
{
  setLED(HIGH);
  MarkerServo.write(MARKER_LOWERED_POS);
  delay(100);
}

 // Lift the marker
 void markerUp()
 {
  setLED(LOW);
  MarkerServo.write(MARKER_LIFTED_POS);
  delay(100);  
 }



// For debugging: LED on and off
void setLED(bool on)
{
  if (on)
    digitalWrite(LED_PIN, HIGH);
  else
    digitalWrite(LED_PIN, LOW);
}


void setup() 
{
  Serial.begin(9600);
  Serial.println("Initializing...");

  Therm.begin();
  Therm.setUnit(TEMP_F); // Set the library's units to Farenheit

  AFMS.begin();  // initialize motor shield

  setSpeed(ROBOT_SPEED); // set our speed
  runMotors(CurrentDir);

  MarkerServo.attach();
  markerUp(); // make sure the marker is lifted
  Serial.println("Initialized OK");
}

void loop()
{
  // Let's calibrate.
  // Serial.println("Calibrating...");
  if (!calibrate_Warm_T()) { 
    delay(100);  
    return;
  }
  // Serial.println("Calibrated.");


  // Let's measure
  // Serial.println("Measuring...");
  if (!Therm.read()) return;  // FIXME: do something better!

  // Serial.print("Measured:");
  float otemp=Therm.object();
  float atemp=Therm.ambient();
  // Serial.println("otemp="+String(otemp,2)+" atemp="+String(atemp,2));

  if (otemp < atemp) {
    // Serial.println("Temp is below ambient");
    if (otemp < Warm_Wall_T - COLD_SPOT_THRESH) {
      // We are in a cold spot!
      // Serial.println("Cold spot");
      if (Marker_is_up) { // marker is up, we should start marking
        runMotors(RELEASE); // stop moving (FIXME: we are coasting still!)
        markerDown();
        runMotors(CurrentDir);
      }
    } else {
      // We are out of a cold spot!
      if (!Marker_is_up) { // marker is down, we should stop marking
        runMotors(RELEASE); // stop moving (FIXME: we are coasting still!)
        markerUp();
        runMotors(CurrentDir);
      }
    }
  }
  // Serial.println("Loop is over");

  // FIXME: check end-of-run/proximity or smth
  if (timeToTurn()) {
    Serial.println("Turning");
    if (CurrentDir==FORWARD) {
      CurrentDir=BACKWARD;
    } else if (CurrentDir==BACKWARD) {
      CurrentDir=FORWARD;
    }
    runMotors(RELEASE);
    delay(500);

  }
  Serial.println("Going to run again");
  runMotors(CurrentDir);
  delay(STEP_TIME); // wait and let the motors go
}

