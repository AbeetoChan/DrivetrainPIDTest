#include <cmath>
#include <math.h>
#include <stdio.h>

#include <v5.h>
#include <v5_vcs.h>

using namespace vex;

inline bool Sign(float x)
{
  if (x > 0)
    return 1;
  if (x < 0)
    return -1;
  return 0;
}

class PIDCalc
{
public:
  const float KP;
  const float KI;
  const float KD;
  const float Thresh;

  float Integral = 0;
  float LastError = 0;
  float PIDValue = 0;

  PIDCalc(float KP, float KI, float KD, float Thresh)
      : KP(KP), KI(KI), KD(KD), Thresh(Thresh)
  {
  }

  void CalculatePID(float expected, float actual)
  {
    float error = actual - expected;

    /* Proportional */ float prop = error * KP;
    /* Integral */ Integral += error;
    if (std::abs(Integral) < Thresh)
    {
      Integral = Sign(Integral) * Thresh;
    }
    /* Derivative */ float derivative = error - LastError;

    PIDValue = prop + Integral + derivative;
  }
};

class PIDdrive
{
public:
  motor_group &LeftMotors;
  motor_group &RightMotors;
  rotation &LeftRotation;
  rotation &RightRotation;
  PIDCalc &motorPID;

  const float WHEEL_DIAMETER = 3.25;
  const float TICKS_PER_INCH = 10.2;

  PIDdrive(
      motor_group &LeftMotors,
      motor_group &RightMotors,
      rotation &LeftRotation,
      rotation &RightRotation,
      PIDCalc &motorPID) : LeftMotors(LeftMotors), RightMotors(RightMotors),
                           LeftRotation(LeftRotation), RightRotation(RightRotation),
                           motorPID(motorPID)
  {
  }

  float GetLeftPosition()
  {
    return LeftRotation.position(rotationUnits::rev);
  }

  float GetRightPosition()
  {
    return RightRotation.position(rotationUnits::rev);
  }

  void ResetSensors()
  {
    LeftRotation.resetPosition();
    RightRotation.resetPosition();
  }

  float CalculateTime(float distanceInInches, float rpm)
  {
    return (distanceInInches / rpm) * (60 / (WHEEL_DIAMETER * M_PI));
  }

  #define GET_TRAVELLED()                       \
  float left_travelled = GetLeftPosition();   \
  float right_travelled = GetRightPosition(); \
  float average = (left_travelled + right_travelled) / 2;

  void DriveForDistance(float distance, float rpm)
  {
    ResetSensors();

    float calculatedTime = CalculateTime(distance, rpm);

    timer totalTimeClock;

    while (totalTimeClock.time(sec) < calculatedTime)
    {
      GET_TRAVELLED()

      motorPID.CalculatePID(rpm, average);

      LeftMotors.spin(directionType::fwd, motorPID.PIDValue, velocityUnits::rpm);
      RightMotors.spin(directionType::fwd, motorPID.PIDValue, velocityUnits::rpm);
      wait(20, msec);
    }

    LeftMotors.stop();
    RightMotors.stop();
  }

  #undef GET_TRAVELLED
};

int main()
{
  motor a = motor(10, ratio18_1, false);
  motor b = motor(9, ratio18_1, true);
  motor c = motor(20, ratio18_1, false);
  motor_group leftMotors = motor_group(a, b, c);

  motor d = motor(11, ratio18_1, false);
  motor e = motor(12, ratio18_1, false);
  motor f = motor(13, ratio18_1, false);
  motor_group rightMotors = motor_group(d, e, f);

  rotation leftRotation = rotation(15);
  rotation rightRotation = rotation(17);

  PIDCalc testPID = PIDCalc(0.5, 0.3, 0.2, 150);
  PIDdrive auton = PIDdrive(leftMotors, rightMotors, leftRotation, rightRotation, testPID);
  auton.DriveForDistance(24, 100);

  return 0;
}