#ifndef movement_h
#define movement_h
#include <Arduino.h>
#include <math.h>

void raise_storage();

void lower_storage();

void diagonal_FW_right(int PWM);                         // Vehicle translates diagonally forward to the right

void diagonal_FW_left(int PWM);                          // Vehicle translates diagonally forward to the right

void translate_right(int PWM);                           // vehicle translates to the right

void translate_left(int PWM);                            // Vehicle translates to the left

void translate_stop();                                   // Set all pwm signals to 0, stop all motors

void rotate_stop();                                      // Currently same as translate_stop ^

void translate_FWD(int PWM);                             // Vehicle drives forwards

void translate_BWD(int PWM);                             // Vehicle drives backwards

void rotate_centered_clkw(int PWM);                      // Rotates vehicle clockwise from the center axis

void rotate_centered_cclkw(int PWM);                     // Rotates vehicle counterclockwise from the center axis

void rotate_clkw_rear(int PWM);                          // Rotates the vehichle clockwise centered on the back axis (centered)

void rotate_cclkw_rear(int PWM);                         // Rotates the vehichle counter clockwise centered on the back axis (centered)

void rotate_clkw_front(int PWM);                         // Rotates the vehichle clockwise centered on the front axis (centered)

void rotate_cclkw_front(int PWM);                        // Rotates the vehichle counter clockwise centered on the front axis (centered)

void quickbrake(int PWM);                         // Forces an invers movement against the current direction of travel, causing a quick stop instead of rolling stop.

#endif