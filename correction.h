// Projectgroup 1
// Bachelors-project in Electronics Design engineering
// 2022

// Alexander Riex ED4
// Henrik Nilsson ED3
// Konrad Råström ED3
// Petter Danev ED5

#ifndef correction_h
#define correction_h
#include <Arduino.h>
#include <math.h>


// ------------Function declaration--------------------------------

void clear_pin(int x);                            // Set digital pin low

int ultraSensor(int trig, int echo);              // Perform measurements with the ultrasonic sensor

int real_distance(float distance, float angle);   // Convert the distance recieved from sensor into orthogonal distance based on angle (trigonometry)

bool rotational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance, int PWM);  // Performs rotational correction, but needs to be inside a loop to work

void translational_correction (double dist_FL, double dist_BL, double dist_FR, double dist_BR, int tolerance, int PWM); // performs translational correction, need to be in a loop to work

void total_correction(int tolerance_angle, int tolerance, int PWM, float angle); // Performs total correction and does not stop until complete

void center_on_tape();

void pickLeftCube();

void pickRightCube();

#endif