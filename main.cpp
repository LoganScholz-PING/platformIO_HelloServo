#include <Arduino.h>
#include <LS7366.h>
#include <Servo.h>
#include "ServoMotorControl.h"


/* === START PROGRAM FLOW CONTROL VARIABLES === */
char ctrlChar;
int START_MOTOR_TEST = 1;
int STOP_MOTOR_TEST = 0;

unsigned long ang_timer = 0;
double angle;
/* === END PROGRAM FLOW CONTROL VARIABLES === */

/* === START ServoMotorControl.cpp EXTERNS === */
extern boolean _motor_enable;
/* === END ServoMotorControl.cpp EXTERNS === */

void setup() 
{
  Serial.begin(9600);
  delay(500);

  servoMotorSetup();
}

void loop() 
{
  
  if (Serial.available())
  {
    ctrlChar = Serial.read();

    if (ctrlChar == '1') 
    { 
      runMotorTest(START_MOTOR_TEST); 
    }
    else if (ctrlChar == '2') 
    { 
      runMotorTest(STOP_MOTOR_TEST); 
    }
    else if (ctrlChar == '3') 
    { 
      servoMotorDirection(MOTOR_CCW);
      Serial.println("Motor Direction CCW"); 
    }
    else if (ctrlChar == '4')
    {
      servoMotorDirection(MOTOR_CW);
      Serial.println("Motor Direction CW");
    }
  }


  angle = servoMotorReadRotationAngle();
  /*
  if (_motor_enable == true)
  {
    Serial.print("Ang: "); Serial.println(angle);
  }
  */
  
  if (angle == 0)
  {
    Serial.print("0 degrees hit: ");
    Serial.println(angle);
    runMotorTest(STOP_MOTOR_TEST);
  }

  /*
  // Angle debug print every quarter of a second
  if (millis() - ang_timer >= 250)
  {    
    if (_motor_enable == true) 
    { 
      Serial.println(angle);
    }
    ang_timer = millis();
  }
  */
    
}