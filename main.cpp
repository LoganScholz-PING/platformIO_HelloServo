 /********************************************************************************************
  * 
  *                       ***DEFINITION OF SERIAL DATA PACKETS:***
  *  SEND:
  *  '<1>' - btnMTRSTART_Click()
  *          Start motor infinitiely spinning until commanded to stop
  *  '<2>' - btnMTRSTOP_Click()
  *          Stop motor (and also stop any PWM)
  *  '<3>' - btnMTRDIRCCW_Click()
  *          Change motor direction to CCW
  *  '<4>' - btnMTRDIRCW_Click()
  *          Change motor direction to CW
  *  '<5>' - btnSingleRotation_Click()
  *          Perform a single motor rotation
  *  '<6>' - btnPulsePerSec_Click()
  *          Perform a one pulse per second motor test
  *  '<7>' - btn400Pulse_Click()
  *          Pulse the motor pulse pin 800 times (800 pulses = 1 complete revolution)
  *  '<8xxx>' - btnRotateDeg_Click()
  *          Turn the motor xxx degrees (if xxx is not a valid number from 0-999 this 
  *          command will be ignored)
  *  '<9>' - btnLoadCellCalib_Click()
  *          Perform load cell calibration (determine calibration as well as zero factors)
  *  '<0>' - btnLoadCellTare_Click()
  *          Tare the load cell to 0
  *  '<a>' - btnLoadCellRead_Click()
  *          Read the current load cell value and print to serial window
  * 
  ******************************************************************************************/

#include <Arduino.h>
#include <LS7366.h>
#include <Servo.h>
#include "ServoMotorControl.h"
#include "LoadCell.h"
#include "Heartbeat.h"
#include "EEPROM_Arduino.h"
#include "AirPressure.h"

/* === START PROGRAM SPECIFIC DEFINES === */
#define SOP '<' // denotes start of serial data packet
#define EOP '>' // denotes end of serial data packet

// mega interrupts INT0,INT1,INT2,INT3,INT4,INT5
// mega interrupt pins: 2, 3, 2, 20, 19, 18
#define pinOPTICALHOME 20        // PB7
//#define pinOPTICALEMERGENCY 20 // not hooked up yet

#define DEBUG_PIN 42             // PL7
boolean DEBUG = false;
/* === END PROGRAM SPECIFIC DEFINES === */

/* === START PROGRAM FLOW CONTROL VARIABLES === */
char ctrlChar;
char serialData[10];     // serial receive buffer
byte index;              // serialData indexer
boolean started = false; // serial data flow control
boolean ended = false;   // serial data flow control

int START_MOTOR_TEST = 1; 
int STOP_MOTOR_TEST = 0;

long quad;

double start_counts = 0;      // for tracking motor movement
double total_counts = 0;      // for tracking motor movement
boolean motor_moving = false; // for tracking if motor is moving

boolean optical_stop_chk = false; // samples the digital state of the optical stop
volatile boolean opticalHOME_stop_hit = false; // true for 10ms when rising edge detected
volatile boolean opticalEMERGENCY_stop_hit = false;
boolean prev_opt_reading = false; // state control variable for determining rising edge
long opt_debounce_time = 0;       // 10ms debounce timer that starts upon detection of rising edge

TimeSlice StatusSlice(2000);      // heartbeat object
long _heartbeat_interval = 4000;  // 4 second heartbeat
long _load_update_interval = 50;  // read torque load every 50ms
unsigned long hb_timer = 0;
/* === END PROGRAM FLOW CONTROL VARIABLES === */

/* === START ServoMotorControl.cpp EXTERNS === */
extern boolean _motor_enable;
/* === END ServoMotorControl.cpp EXTERNS === */



void checkSerial()
{
  // if serial is available, receive the
  // serial data packet (it better be formatted
  // correctly!!)
  while (Serial.available() > 0)
  {
    char inChar = Serial.read();
    // check if we receive the start character  
    // (SOP) of a serial packet
    if (inChar == SOP)
    {
      index = 0;
      serialData[index] = '\0'; // null character
      started = true;
      ended = false;
    }
    // check if we receive the end character
    // (EOP) of a serial packet
    else if (inChar == EOP)
    {
      ended = true;
      break;
    }
    else
    {
      if (index < 79)
      {
        serialData[index] = inChar;
        ++index;
        serialData[index] = '\0';
      }
    }
  }

  if (started && ended)
  {
    // packet start and end control characters
    // received, begin packet processing
    switch (serialData[0])
    {
      case '1':
        // starts motor free spinning
        runMotorTest(START_MOTOR_TEST);
        break;
      case '2':
        // disables the motor and stops PWM
        runMotorTest(STOP_MOTOR_TEST); 
        break;
      case '3':
        // change motor direction spin to counter-clockwise
        servoMotorDirection(MOTOR_CCW);
        if (DEBUG)
        {
          Serial.println("Motor Direction CCW");
        }       
        break;
      case '4':
        // change motor direction spin to clockwise
        servoMotorDirection(MOTOR_CW);
        if (DEBUG)
        {
          Serial.println("Motor Direction CW");
        }       
        break;
      case '5':
        // note that this function will block for 1 second
        // to calculate quadrature counts / 360 degree
        // rotation
        servoMotorOneRevolution();
        break;
      case '6':
        // Actuate air pressure solenoid for clamps
        actuateSolenoid();
        break;
      case '7':
        // the function below is blocking
        servoMotor800Pulses();
        break;
      case '8':
        { // you have to scope the case statement if you created a variable
        // move motor a certain amount as instructed
        
        // atoi() is a C++ built in function to
        // convert a char* array into an integer
        // (it even understands negatives!)
        int deg = atoi(&serialData[1]); 
        start_counts = (double)servoMotorReadQuadratureCount();
        total_counts = servoMotorMoveDegrees(deg);
        motor_moving = true; // maybe move this to the servoMotorMoveDegrees() function
        break;
        }
      case '9':
        // Calibrate Load Cell
        loadcellDetermineCalibrationFactor();
        break;
      case '0':
        // Tare Load Cell
        loadcellTare();
        break;
      case 'a':
        // Read Load Cell
        loadcellReadCurrentValue();
        break;
      case 'b':
        // read first 80 bytes of EEPROM to serial
        REPORT_EEPROM_CONTENTS();
        break;
      case 'c':
        // zero out the contents of the first 80 bytes of EEPROM
        ZERO_EEPROM_CONTENTS();
        break;
      case 'd':
        // read only contents of EEPROM we care about
        ReadEEPROM(true);
        break;
      case 'e':
        // read and output air pressure
        readAirPressure();
        break;
      default:
        // we received a packet where serialData[0] 
        // is unrecognized
        Serial.print("ERROR Invalid or uninterpretable command received on serial: ");
        Serial.println(serialData[0]);
        break;
    }

    // packet processing completed, reset packet
    // parameters to get ready for next packet
    started = false;
    ended = false;
    index = 0;
    serialData[index] = '\0';
  }
} // end void checkSerial()


void homeOpticalStopInt() // interrupt service routine
{
  opticalHOME_stop_hit = true;
  servoMotorEnable(MOTOR_DISABLED);
} // end void homeOpticalStopInt()

void emergencyOpticalStopInt() // interrupt service routine
{
  opticalEMERGENCY_stop_hit = true;
} // end void emergencyOpticalStopInt()


void ifMovingCheckCountFeedback()
{
  if (DEBUG)
  {
    Serial.println("INSIDE ifMovingCheckCountFeedback()");
  }
  quad = servoMotorReadQuadratureCount();
  if ( (abs(quad-start_counts) >= abs(total_counts)) )
  {   
    servoMotorEnable(MOTOR_DISABLED);
    motor_moving = false;
    if (DEBUG)
    {
      Serial.println("Main loop has stopped motor movement"); 
      Serial.print("total_counts: "); Serial.println(total_counts);
      Serial.print("start_counts: "); Serial.println(start_counts);
      Serial.print("End counts (quad): "); Serial.println(abs(quad));
      Serial.print("Count delta: "); Serial.println(abs(quad-start_counts));
    }
  }
} // end void ifMovingCheckCountFeedback()

void updateEnvironment()
{
  //char* cp; // for tracking state machine current state, not implemented
  //double pos = servoMotorReadRotationAngle(); // doesn't work yet
  float tq = loadcellReadCurrentValue();


  Serial.print("{,");
  Serial.print(":T="); Serial.print(tq);
  //Serial.print(":A="); Serial.print(pos);
  Serial.print(":MTR="); Serial.print(motor_moving);
  Serial.println(",}");
}

void setup() 
{
  Serial.begin(9600);
  delay(250); // let serial settle itself
  
  pinMode(pinOPTICALHOME, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinOPTICALHOME), 
                  homeOpticalStopInt, 
                  RISING);

  /* EMERGENCY stop not implemented yet
  pinMode(pinOPTICALEMERGENCY, INPUT); // not hooked up yet
  attachInterrupt(digitalPinToInterrupt(pinOPTICALEMERGENCY), 
                  emergencyOpticalStopInt, 
                  RISING);
  */
  
  pinMode(DEBUG_PIN, INPUT_PULLUP);

  servoMotorSetup();
  setupAirValve();
  // need to do setupEEPROM() before loadcellSetup()
  // because loadcellSetup() retrieves data from EEPROM
  // like scale_zero_bias and scale_calibration_factor
  setupEEPROM(); 
  loadcellSetup();
  
  // initialize heartbeat to 4 seconds
  StatusSlice.Interval(_heartbeat_interval);
} // end void setup()


void loop() 
{
  // DEBUG_PIN is INPUT_PULLUP (defaults HIGH aka TRUE)
  // If DEBUG_PIN is grounded (LOW aka FALSE) we want
  // the debug output to show in the serial window (DEBUG = TRUE)
  // Since we want to check this every iteration of the main loop,
  // we'll make this operation as performant as possible.
  // DEBUG_PIN is pin 42, which is Register "PORTL", bit 7 (0 indexed)
  // _SFR_MEM8(0x109) returns all 8 bits of register PORTL
  // (note 0x109 is Port L's "PINL" register memory address)
  DEBUG = !(_SFR_MEM8(0x109) & B10000000);
  
  checkSerial();

  if (motor_moving)
  {
    ifMovingCheckCountFeedback(); 
  }

  if ( opticalHOME_stop_hit )
  {
    servoMotorEnable(MOTOR_DISABLED);
    Serial.println("*OPTICAL HOME STOP HIT*");
    opticalHOME_stop_hit = false;
  }

  if ( opticalEMERGENCY_stop_hit )
  {
    servoMotorEnable(MOTOR_DISABLED);
    Serial.println("*OPTICAL EMERGENCY STOP HIT*");
    opticalEMERGENCY_stop_hit = false;
  }

/*
  hb_timer = millis();
  if (StatusSlice.Triggered(hb_timer))
  {
    updateEnvironment(); // output heartbeat to serial
  }
*/
} // end void loop()