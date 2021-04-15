#ifndef SERVOMOTORCONTROL_H
#define SERVOMOTORCONTROL_H

#include <Arduino.h>

/* === START SERVO MOTOR DEFINES === */
#define MOTOR_CCW LOW
#define MOTOR_CW HIGH
#define MOTOR_ENABLED LOW
#define MOTOR_DISABLED HIGH
#define mtrDirectionPin 8
#define mtrPulsePin 9
#define mtrEnablePin 10
/* === END SERVO MOTOR DEFINES === */

/* === START QUADRATURE DEFINES === */
#define QUAD_CHIPSELECT 53
/* === END QUADRATURE DEFINES === */



/***** FUNCTION PROTOTYPES *****/
/* === START SERVO MOTOR FUNCTION PROTOTYPES === */
void servoMotorSetup();
void servoMotorEnable(byte v);
void servoMotorDirection(boolean v);
void servoMotorAdjustSpeed(unsigned long nw);
void servoMotorFrequency(float f);

void runMotorTest(int cmd);
/* === END SERVO MOTOR FUNCTION PROTOTYPES === */

/* === START QUADRATURE FUNCTION PROTOTYPES === */
void servoMotorSetupQuadrature();
long servoMotorReadQuadratureCount();
double servoMotorReadRotationAngle();
/* === END QUADRATURE FUNCTION PROTOTYPES === */

#endif // end SERVOMOTORCONTROL_H guard macro