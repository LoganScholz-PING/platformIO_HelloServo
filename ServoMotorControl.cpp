#include "ServoMotorControl.h"
#include "printBinary.h"
#include <LS7366.h>
#include <NewTone.h>


/* === START SERVO MOTOR VARIABLE DEFINITIONS === */
boolean _motor_enable = false;
boolean _motor_direction = MOTOR_CCW;

long pulses_per_revolution = 200;
double degrees_per_pulse = (360 / pulses_per_revolution); // 1.8 @ 200ppr
double pulses_per_degree = (pulses_per_revolution / 360);
unsigned long _max_freq = 35000;

float _current_speed = 0; // 0-1 controls percent of _max_freq
/* === END SERVO MOTOR VARIABLE DEFINITIONS === */ 

/* === START QUADRATURE VARIABLE DEFINITIONS === */
long _quad_count = 0;
LS7366 QuadCounter(QUAD_CHIPSELECT);
/* === END QUADRATURE VARIABLE DEFINITIONS === */


void servoMotorSetup()
{
    pinMode(mtrPulsePin, OUTPUT);
    pinMode(mtrDirectionPin, OUTPUT);
    pinMode(mtrEnablePin, OUTPUT);

    digitalWrite(mtrEnablePin, MOTOR_DISABLED);
    digitalWrite(mtrDirectionPin, MOTOR_CCW);

    //digitalWrite(mtrPulsePin, 0);

    servoMotorSetupQuadrature();
}

void servoMotorEnable(byte v)
{
    // Toggle state of the mtrEnablePin
    digitalWrite(mtrEnablePin, v);
    _motor_enable = (v == 0);
    if (_motor_enable == false)
    {
        // Turn off the PWM to the motor
        // When Motor is disabled
        noNewTone(mtrPulsePin);
    }
    //Serial.print("_motor_enable status: "); Serial.println(_motor_enable);
}

void servoMotorFrequency(float f)
{
    if ( f < 0 ) { f = 0; }
    if ( f > 1.0 ) { f = 1.0; }

    _current_speed = f;
    unsigned long x = (f * _max_freq);

    // NewTone(pin, freq, length)
    // if length not present, it lasts forever
    NewTone(mtrPulsePin, x);
}

void servoMotorDirection(boolean v)
{
    // TODO
    _motor_direction = v;
    digitalWrite(mtrDirectionPin, _motor_direction);
}

void servoMotorAdjustSpeed(unsigned long nw)
{
    // TODO
}

void runMotorTest(int cmd)
{
  if (cmd == 1)
  {
    // cmd == 1 == start motor test
    Serial.println("Starting motor test");
    // 0.00571 == 200 pulses per second = 1 revolution per second
    servoMotorFrequency(0.001);   
    servoMotorEnable(MOTOR_ENABLED);
    Serial.print("_motor_enable Status: "); Serial.println(_motor_enable);
  }
  else if (cmd == 0)
  {
    // cmd == 0 == stop motor test 
    Serial.println("Stopping motor test");
    servoMotorEnable(MOTOR_DISABLED);
    Serial.print("_motor_enable Status: "); Serial.println(_motor_enable);
  }
  else
  {
    // unknown value passed to cmd
    Serial.println("(else) Stopping motor test");
    servoMotorEnable(MOTOR_DISABLED);
    Serial.print("_motor_enable Status: "); Serial.println(_motor_enable);
  }
}



void servoMotorSetupQuadrature()
{
    QuadCounter.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
	QuadCounter.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4);
	//	QuadCounter.clear_counter();
	QuadCounter.clear_status_register();
	QuadCounter.write_data_register(4);
}

long servoMotorReadQuadratureCount()
{

    _quad_count = QuadCounter.read_counter();
    
    /*
    // debug print:
    byte* b = (byte*)(&_quad_count);
    Serial.print("readQuad:"); 
    Serial.print(_quad_count); 
    Serial.print(" ");
    print_binary(*(b + 2));
    Serial.print(" ");
    print_binary(*(b + 3));
    Serial.print(" ");
    Serial.println();
    */
    

    return _quad_count;
}

double servoMotorReadRotationAngle()
{
    servoMotorReadQuadratureCount();

    long q = (_quad_count % pulses_per_revolution);
    Serial.print("Read Quad:"); Serial.print(_quad_count); Serial.print(" || readRot Q:"); Serial.println(q);
    
    double ang = (degrees_per_pulse * q);
    //Serial.print("readAng A:"); Serial.println(ang);

    return ang;
} 