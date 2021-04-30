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

/* === START PROGRAM SPECIFIC DEFINES === */
#define SOP '<' // denotes start of serial data packet
#define EOP '>' // denotes end of serial data packet

#define pinOPTICALSTOP 13 // PB7
#define DEBUG_PIN 42      // PL7

boolean DEBUG = false;
/* === END PROGRAM SPECIFIC DEFINES === */

/* === START PROGRAM FLOW CONTROL VARIABLES === */
char ctrlChar;
char serialData[10]; // serial receive buffer
byte index;
boolean started = false;
boolean ended = false;

int START_MOTOR_TEST = 1;
int STOP_MOTOR_TEST = 0;

unsigned long angle_timer = 0;
double angle;
long quad;

double start_counts = 0;
double total_counts = 0;
boolean motor_moving = false;

boolean optical_stop_chk = false; // samples the digital state of the optical stop
boolean optical_stop_hit = false; // true for 10ms when rising edge detected
boolean prev_opt_reading = false; // state control variable for determining rising edge
long opt_debounce_time = 0;       // 10ms debounce timer that starts upon detection of rising edge
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
        // disables the motor
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
        // unused
        if (DEBUG)
        {
          Serial.println("Serial cmd 6 is unused!");
        }
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


// TODO?: THIS SHOULD BE HOOKED TO AN INTERRUPT?
void checkOpticalStop()
{
  // pinOPTICALSTOP is HIGH if beam is broken
  //optical_stop_chk = digitalRead(pinOPTICALSTOP); // slow
  optical_stop_chk = (_SFR_IO8(0X03) & B10000000); // more performant

  // if statement checks for rising edge condition
  // on optical_stop_chk (with a 10ms debounce)
  if ((optical_stop_chk) && 
      (prev_opt_reading != optical_stop_chk) && 
      (millis() - opt_debounce_time >= 10))
  {
    optical_stop_hit = true;
    if (DEBUG)
    {
      Serial.println("*OPTICAL STOP HIT*");
    }
    opt_debounce_time = millis();
  }

  // reset the boolean that tracks optical beam state after
  // 10ms (when optical endstop beam is broken this discrete
  // will stay true for 10ms)
  if (optical_stop_hit && (millis() - opt_debounce_time >= 10))
  {
    optical_stop_hit = false;
    //Serial.println("*OPTICAL STOP BOOLEAN RESET*");
  }

  // if motor is enabled and the optical stop is hit
  // turn off the motor
  if (_motor_enable && optical_stop_hit)
  {
    servoMotorEnable(MOTOR_DISABLED);
    if (DEBUG)
    {
      Serial.println("*STOP MOTOR DUE TO OPTICAL STOP*");
    }
  } 

  prev_opt_reading = optical_stop_chk;
} // void checkOpticalStop()


void ifMovingCheckCountFeedback()
{
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


void setup() 
{
  Serial.begin(9600);
  delay(250); // let serial settle itself
  servoMotorSetup();
  loadcellSetup();

  pinMode(pinOPTICALSTOP, INPUT);
  pinMode(DEBUG_PIN, INPUT_PULLUP);
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

  checkOpticalStop();

  if (motor_moving)
  {
    ifMovingCheckCountFeedback(); 
  }   
} // end void loop()