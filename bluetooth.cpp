// Projectgroup 1
// Bachelors-project in Electronics Design engineering
// 2022

// Alexander Riex ED4
// Henrik Nilsson ED3
// Konrad Råström ED3
// Petter Danev ED5
//---------------------------------Libraries--------------------------
#include <Arduino.h>
#include <math.h>
#include "bluetooth.h"
#include "movement.h"
#include "SoftwareSerial.h"
#include "correction.h"
#include <Servo.h>
#include "audio.h"

// --------Defining variables------------
//char State1;
//char Zone;
//char Instruction;
//char Ack;
//char direction;
//int i = 0;
//bool ignore = true;
//int Tape = 0;


// sensor variables 

//float angle = 18.33;
//int tolerance_angle = 6; //tolerance for correction functions
//int tolerance = 1; //tolerance for correction functions

/*
double last_real_distance_FL;
double last_real_distance_FR; 
double last_real_distance_BL;
double last_real_distance_BR;
double real_distance_FL;
double real_distance_FR;
double real_distance_BL;
double real_distance_BR;


int sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7, sensorValue8;
int sensorValue9, sensorValue10, sensorValue11, sensorValue12, sensorValue13, sensorValue14, sensorValue15, sensorValue16;
*/

// --------------------- changeable variables-----------------

//int PWM_3 = 70; // PWM för zon 3. Måste kalibreras!
//bool corrected = false;



//---------Defining pins----------------

#define SENSORA_F1 A8   // Sensor ramp forward
#define SENSORA_F2 A9
#define SENSORA_F3 A10
#define SENSORA_F4 A11
#define SENSORA_F5 A12
#define SENSORA_F6 A13
#define SENSORA_F7 A14
#define SENSORA_F8 A15

#define SENSORA_B1 A0  // sensor ramp backward
#define SENSORA_B2 A1
#define SENSORA_B3 A2
#define SENSORA_B4 A3
#define SENSORA_B5 A4
#define SENSORA_B6 A5
#define SENSORA_B7 A6
#define SENSORA_B8 A7

#define echopin_FL 24 // Forward Left sensor
#define trigpin_FL 22
#define echopin_FR 50 // Forward Right sensor
#define trigpin_FR 52
#define echopin_BL 28 // Back Left
#define trigpin_BL 30
#define echopin_BR 46 // Back Right
#define trigpin_BR 48



//#define servo_pin 2

//Servo plockservo;

// ------------------------------- function ---------------------------

/*
String Plocka(char inst, int PWM, String INBYTE)
{
    int time=1000;  //1000ms=1s

switch (inst)
{
case 'l':   // Vänster plock
    translate_left(PWM);
    delay(time);    
    translate_stop();
    Plockat();
    translate_right(PWM);
    delay(time);    
    translate_stop();
    INBYTE[2]='u';  // Plockning utförd
    INBYTE[1]='1';  //Klar
    break;

case 'r':   // Höger Plock
    translate_right(PWM);
    delay(time);    
    translate_stop();
    Plockat();
    translate_left(PWM);
    delay(time);    
    translate_stop();
    INBYTE[2]='u';  // Plockning utförd
    INBYTE[1]='1';  //Klar
    break;

case 'm':
    
    Plockat();
    sing(1);
    INBYTE[2]='u';  // Plockning utförd
    INBYTE[1]='1';  // Klar
    break;
case 'q':
    Plockat();
    break;
default:
    break;
}


return INBYTE;
}
*/

/*
String Tejpbitar(char inst, int PWM, String INBYTE)
{
    // Loopa hela funktionen tills man hittar rätt antal tejpbitar.
    while(INBYTE[1] == '0') //Loop while looking for the coorect tejpbit.
    {
    readIRData();
    translate_FWD(PWM);
    // IR-sensor kod.
    if(((sensorValue9+sensorValue10+sensorValue11+sensorValue12+sensorValue13+sensorValue14+sensorValue15+sensorValue16 + 600) / 8) < 
    ((sensorValue1+sensorValue2+sensorValue3+sensorValue4+sensorValue5+sensorValue6+sensorValue7+sensorValue8) / 8))
    {
        i++;
        switch (inst)
    {
    case 'x':
        if (i==1)
        {
            INBYTE[1]='1';  // Klar
            INBYTE[2]='p';  // Skickar tillbaka att den har passerat en tejpbit
            i=0;
             
        }
        break;
    case 'y':
        if (i==2)
        {
            INBYTE[1]='1';  // Klar
            INBYTE[2]='p';  // Skickar tillbaka att den har passerat en tejpbit
            i=0;
        }
        break;
    case 'z':
        if (i==3)
        {
            INBYTE[1]='1';  // Klar
            INBYTE[2]='p';  // Skickar tillbaka att den har passerat en tejpbit.    
            i=0;
        }
        break;
    
    default:
        INBYTE[2]='p';  // Skickar tillbaka att den har passerat en tejpbit.
      //  BTSerial.println(INBYTE);
        break;
    }
    
    }   
    }    
return INBYTE;
}
*/
/*
void readIRData()
{
   sensorValue1 = analogRead(SENSORA_F1);
   sensorValue2 = analogRead(SENSORA_F2);
   sensorValue3 = analogRead(SENSORA_F3);
   sensorValue4 = analogRead(SENSORA_F4);
   sensorValue5 =  analogRead(SENSORA_F5);
   sensorValue6 = analogRead(SENSORA_F6);
   sensorValue7 = analogRead(SENSORA_F7);
   sensorValue8 =  analogRead(SENSORA_F8);

   sensorValue9 = analogRead(SENSORA_B1);
   sensorValue10 = analogRead(SENSORA_B2);
   sensorValue11 = analogRead(SENSORA_B3);
   sensorValue12 = analogRead(SENSORA_B4);
   sensorValue13 =  analogRead(SENSORA_B5);
   sensorValue14 = analogRead(SENSORA_B6);
   sensorValue15 = analogRead(SENSORA_B7);
   sensorValue16 =  analogRead(SENSORA_B8);
//int calc1 = (sensorValue1 + sensorValue2 + sensorValue7 + sensorValue8);
//int calc2 = (sensorValue9 + sensorValue10 + sensorValue15 + sensorValue16);
  // Serial.println(calc1 , "    ", calc2);
}
*/





