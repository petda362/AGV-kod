// Projectgroup 1
// Bachelors-project in Electronics Design engineering
// 2022

// Alexander Riex ED4
// Henrik Nilsson ED3
// Konrad Råström ED3
// Petter Danev ED5

//-------------Libraries---------------
#include "movement.h"
#include "Servo.h"


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
// --------------------- changeable variables-----------------
float multiplier_FL = 0.98;         // Multipliers for seperate wheels, for adjusting motor speed (PWM * multiplier)
float multiplier_FR = 1;
float multiplier_BL = 0.98;
float multiplier_BR = 1;
float multiplier_rotation = 1;    // multiplier for rotation, for adjusting motor speed whilst rotating (PWM * multiplier)
int current_dir;
// ------------------------------- function ---------------------------

void diagonal_FW_right(int PWM)
{
  analogWrite(FWDpin_FL, (PWM * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (PWM * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));
}

void diagonal_FW_left(int PWM)
{
  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (PWM * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (PWM * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));
}

void translate_left(int PWM)
{

  analogWrite(FWDpin_FL, (0));
  analogWrite(BWDpin_FL, (PWM*1.2));

  analogWrite(FWDpin_FR, (PWM));
  analogWrite(BWDpin_FR, (0));

  analogWrite(FWDpin_BL, (PWM));
  analogWrite(BWDpin_BL, (0));

  analogWrite(FWDpin_BR, (0));
  analogWrite(BWDpin_BR, (PWM*1.2));

  current_dir = 3;                                      // Set current direction as translate right

}

void translate_right(int PWM)
{

  analogWrite(FWDpin_FL, (PWM*1.10));
  analogWrite(BWDpin_FL, (0));

  analogWrite(FWDpin_FR, (0));
  analogWrite(BWDpin_FR, (PWM));

  analogWrite(FWDpin_BL, (0));
  analogWrite(BWDpin_BL, (PWM));

  analogWrite(FWDpin_BR, (PWM*1.10));
  analogWrite(BWDpin_BR, (0));

  current_dir = 4;                                      // Set current direction as translate left
}

void translate_stop()
{

  analogWrite(FWDpin_FL, 0);
  analogWrite(BWDpin_FL, 0);

  analogWrite(FWDpin_FR, 0);
  analogWrite(BWDpin_FR, 0);

  analogWrite(FWDpin_BL, 0);
  analogWrite(BWDpin_BL, 0);

  analogWrite(FWDpin_BR, 0);
  analogWrite(BWDpin_BR, 0);
}

void rotate_stop()
{

  analogWrite(FWDpin_FL, 0);
  analogWrite(BWDpin_FL, 0);

  analogWrite(FWDpin_FR, 0);
  analogWrite(BWDpin_FR, 0);

  analogWrite(FWDpin_BL, 0);
  analogWrite(BWDpin_BL, 0);

  analogWrite(FWDpin_BR, 0);
  analogWrite(BWDpin_BR, 0);
}

void translate_BWD(int PWM)
{
  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (PWM * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (PWM * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (PWM * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (PWM * multiplier_BR));

    current_dir = 2;                                      // Set current direction as translate BWD
}

void translate_FWD(int PWM)
{
  analogWrite(FWDpin_FL, (PWM * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (PWM * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (PWM * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (PWM * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));

    current_dir = 1;                                      // Set current direction as translate FWD
}

void rotate_centered_cclkw(int PWM)
{
  analogWrite(FWDpin_FL, (0 * multiplier_FL * multiplier_rotation));
  analogWrite(BWDpin_FL, (PWM * multiplier_FL * multiplier_rotation));

  analogWrite(FWDpin_FR, (PWM * multiplier_FR * multiplier_rotation));
  analogWrite(BWDpin_FR, (0 * multiplier_FR * multiplier_rotation));

  analogWrite(FWDpin_BL, (0 * multiplier_BL * multiplier_rotation));
  analogWrite(BWDpin_BL, (PWM * multiplier_BL * multiplier_rotation));

  analogWrite(FWDpin_BR, (PWM * multiplier_BR * multiplier_rotation));
  analogWrite(BWDpin_BR, (0 * multiplier_BR * multiplier_rotation));

  current_dir = 5;                                      // Set current direction as rotate clockwise
}

void rotate_centered_clkw(int PWM)
{
  analogWrite(FWDpin_FL, (PWM * multiplier_FL * multiplier_rotation));
  analogWrite(BWDpin_FL, (0 * multiplier_FL * multiplier_rotation));

  analogWrite(FWDpin_FR, (0 * multiplier_FR * multiplier_rotation));
  analogWrite(BWDpin_FR, (PWM * multiplier_FR * multiplier_rotation));

  analogWrite(FWDpin_BL, (PWM * multiplier_BL * multiplier_rotation));
  analogWrite(BWDpin_BL, (0 * multiplier_BL * multiplier_rotation));

  analogWrite(FWDpin_BR, (0 * multiplier_BR * multiplier_rotation));
  analogWrite(BWDpin_BR, (PWM * multiplier_BR * multiplier_rotation));

  current_dir = 6;                                      // Set current direction as rotate counter clockwise
}

void rotate_clkw_front(int PWM)
{
  analogWrite(FWDpin_FL, (PWM * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (PWM * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));
  
  current_dir = 7;                                      // Set current direction as rotate clockwise on the rear axis
}

void rotate_cclkw_front(int PWM)
{
  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (PWM * multiplier_FL));

  analogWrite(FWDpin_FR, (PWM * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));

  current_dir = 8;                                      // Set current direction as rotate counter clockwise on the rear axis
}

void rotate_clkw_rear(int PWM)
{
  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (PWM * multiplier_BL));
  analogWrite(BWDpin_BL, (0 * multiplier_BL));

  analogWrite(FWDpin_BR, (0 * multiplier_BR));
  analogWrite(BWDpin_BR, (PWM * multiplier_BR));

  current_dir = 9;                                      // Set current direction as rotate clockwise on the front axis
}

void rotate_cclkw_rear(int PWM)
{
  analogWrite(FWDpin_FL, (0 * multiplier_FL));
  analogWrite(BWDpin_FL, (0 * multiplier_FL));

  analogWrite(FWDpin_FR, (0 * multiplier_FR));
  analogWrite(BWDpin_FR, (0 * multiplier_FR));

  analogWrite(FWDpin_BL, (0 * multiplier_BL));
  analogWrite(BWDpin_BL, (PWM * multiplier_BL));

  analogWrite(FWDpin_BR, (PWM * multiplier_BR));
  analogWrite(BWDpin_BR, (0 * multiplier_BR));

  current_dir = 10;                                      // Set current direction as rotate counter clockwise on the rear axis
}

void quickbrake(int PWM) {
    unsigned int brake_delay = (PWM) / 3;

    switch(current_dir) {           // Every set of movements has a number correlated to it
        case 1:                     // case = 1 current dir forward
            translate_BWD(PWM);
            delay(brake_delay);
            translate_stop();

        break;
        case 2:                     // case = 2 current dir backward
            translate_FWD(PWM);
            delay(brake_delay);
            translate_stop();
        break;
        case 3:                     // case = 3 current dir right
            translate_left(PWM);
            delay(brake_delay);
            translate_stop();
        break;
        case 4:                     // case = 4 current dir left
            translate_right(PWM);
            delay(brake_delay);
            translate_stop();

        break;
        case 5:                     // case = 5 current dir rotate clkw
            rotate_centered_cclkw(PWM);
            delay(brake_delay);
            rotate_stop();

        break;
        case 6:                     // case = 6 current dir rotate c-clkw
            rotate_centered_clkw(PWM);
            delay(brake_delay);
            rotate_stop();

        break;
        case 7:                     // case = 7 current dir rotate clockwise on the rear axis
            rotate_cclkw_rear(PWM);    
            delay(brake_delay);
            rotate_stop();
        break;
        case 8:                     // case = 8 current dir rotate counter clockwise on the rear axis
            rotate_clkw_rear(PWM);     
            delay(brake_delay);
            rotate_stop();

        break;
        case 9:                     // case = 9 current dir rotate clockwise on the front axis
            rotate_cclkw_front(PWM);
            delay(brake_delay);
            rotate_stop();

        break;
        case 10:                    // case 10 current dir rotate counter clockwise on the rear axis
            rotate_clkw_front(PWM);
            delay(brake_delay);
            rotate_stop();
        break;
    }

}
