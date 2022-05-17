// Projectgroup 1
// Bachelors-project in Electronics Design engineering
// 2022

// Alexander Riex ED4
// Henrik Nilsson ED3
// Konrad Råström ED3
// Petter Danev ED5

//-------------Libraries---------------
#include <Arduino.h>
#include <math.h>
#include "movement.h"

//---------Defining pins----------------
#define echopin_FL 24 // Forward Left sensor
#define trigpin_FL 22
#define echopin_FR 50 // Forward Right sensor
#define trigpin_FR 52
#define echopin_BL 28 // Back Left
#define trigpin_BL 30
#define echopin_BR 46 // Back Right
#define trigpin_BR 48

#define STATE 53      // STATE PIN HC05
#define buzzer_pin 32 // pin for audio buzzer

#define FWDpin_FL 8 // FWD Forward left
#define BWDpin_FL 9 // BWD
#define FWDpin_FR 7 // FWD Forward Right
#define BWDpin_FR 6 // BWD
#define FWDpin_BL  10 // FWD Backward LefB
#define BWDpin_BL  11 // BWD
#define FWDpin_BR 5 // FWD Backward Right
#define BWDpin_BR 4 // BWD

#define SENSOR_L 26 // IR-Sensor vänster för att hitta block 
#define SENSOR_R 44 // IR-Sensor höger för att hitta block

// --------------------- variables-------------
bool orth = false;
bool centered = false;
// ------------Functions------------------
void clear_pin(int x)
{
  digitalWrite(x, LOW);
}

int ultraSensor(int trig, int echo)
{
  long travelTime;
  float distance;
  float sum_distances = 0;
  float avg_distance;
  int num_measurements = 1;//--------------------------------------------
  
  for (int i = 0; i < num_measurements; i = i + 1) {
    clear_pin(trig);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    clear_pin(trig);
    travelTime = pulseIn(echo, HIGH);
    distance = (travelTime / 2) / 29.1 * 10; // distance in mm
    sum_distances = sum_distances + distance;
  }
  avg_distance = sum_distances / num_measurements;
  return avg_distance;
}


int real_distance(float distance, float angle)
{
  double real_dist = distance * sin(DEG_TO_RAD * angle);
  return real_dist;
}


bool rotational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance_angle, int PWM) {
  Serial.println("rotation");
   
    if ((dist_BL < (dist_FL - tolerance_angle)) ||
           (dist_FR < (dist_BR - tolerance_angle)))
  {
    rotate_centered_cclkw(150);
    delay(3);
    rotate_centered_cclkw(PWM - 26);
    orth = false;
  }
  else if ((dist_FL < (dist_BL - tolerance_angle) ||
            (dist_FR > (dist_BR + tolerance_angle))))
  {
    rotate_centered_clkw(150);
    delay(3);
    rotate_centered_clkw(PWM - 26);
    orth = false;
  }
  else if (!orth) {
    orth = true;
    quickbrake(PWM);
    // rotate_stop();
  }
  return orth;
}

void translational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance, int PWM) {
  Serial.println("translation");

    if (orth && ((dist_FL >= (dist_FR - tolerance)) &&
               (dist_FL <= (dist_FR + tolerance))))
  {
    translate_stop();
    centered = true;
  
  }
  else if (orth && ((dist_FL < (dist_FR - tolerance)) || 
                    (dist_BL < (dist_BR - tolerance))))
  {
    translate_right(200);
    delay(1);
    translate_right(PWM - 10);
  }
  else if (orth && ((dist_FL > (dist_FR + tolerance)) || 
                    (dist_BL > (dist_BR + tolerance))))
  {
    translate_left(200);
    delay(1);
    translate_left(PWM - 10);
  }
}

void total_correction(int tolerance_angle, int tolerance, int PWM, float angle) {
  Serial.println("total");
  double dist_FL;
  double dist_FR;
  double dist_BL;
  double dist_BR;
  while(!centered) {
    dist_FL = real_distance(ultraSensor(trigpin_FL, echopin_FL), angle);
    dist_FR = real_distance(ultraSensor(trigpin_FR, echopin_FR), angle);
    dist_BL = real_distance(ultraSensor(trigpin_BL, echopin_BL), angle);
    dist_BR = real_distance(ultraSensor(trigpin_BR, echopin_BR), angle);
   if (orth == false){
    orth = rotational_correction(dist_FL, dist_BL, dist_FR, dist_BR, tolerance_angle, PWM);
   }
   else if (centered == false){ 
    translational_correction(dist_FL, dist_BL, dist_FR, dist_BR, tolerance, PWM);
    if(centered){
    orth = false;
    }
   }
  }
  orth = false;
  centered = false;
}

void pickLeftCube()
{   int lastMeasurementIR = 0;
    int leftSensorHit = 0;
    delay(300);
    translate_BWD(100);
    delay(160);
    quickbrake(100);
    delay(300);
    translate_left(200);
    while(true)
    {
      lastMeasurementIR = leftSensorHit;
      for(int i = 0; i<5; i++)
      {
        leftSensorHit += digitalRead(SENSOR_L);
        delay(3);
      }
      if(leftSensorHit < 3)
      {
        leftSensorHit = 1;
      }
      else
      {
        leftSensorHit = 0;
      }
      if(leftSensorHit - lastMeasurementIR == -1)
      {
        delay(100);
        translate_right(200);
        delay(100);
        translate_stop();
        break;
      }
    }
}

void pickRightCube()
{
    int lastMeasurementIR = 0;
    int rightSensorHit = 0;
    delay(300);
    translate_BWD(100);
    delay(200);
    quickbrake(100);
    delay(300);
    translate_right(200);
    while(true)
    {
      lastMeasurementIR = rightSensorHit;
      for(int i = 0; i<5; i++)
      {
        rightSensorHit += digitalRead(SENSOR_R);
        delay(3);
      }
      if(rightSensorHit < 3)
      {
        rightSensorHit = 1;
      }
      else
      {
        rightSensorHit = 0;
      }
      if(rightSensorHit - lastMeasurementIR == -1)
      {
        delay(100);
        translate_left(200);
        delay(100);
        translate_stop();
        break;
      }
    }
}