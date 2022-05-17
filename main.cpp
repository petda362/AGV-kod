// Projectgroup 1
// Bachelors-project in Electronics Design Engineering
// 2022

// Alexander Riex ED4
// Henrik Nilsson ED3
// Konrad Råström ED3
// Petter Danev ED5

//-------------Libraries---------------
#include <Arduino.h>
#include "SoftwareSerial.h"
#include <math.h>
#include "correction.h"
#include "bluetooth.h"
#include "movement.h"
#include "audio.h"
#include "Servo.h"

//---------Defining pins---------------- GÄLLER MED SHIELD
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

// Motordrivare
#define FWDpin_FL 8     // FWD Forward left
#define BWDpin_FL 9     // BWD
#define FWDpin_FR 7     // FWD Forward Right
#define BWDpin_FR 6     // BWD
#define FWDpin_BL  10   // FWD Backward LefB
#define BWDpin_BL  11   // BWD
#define FWDpin_BR 5     // FWD Backward Right
#define BWDpin_BR 4     // BWD


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

#define Switchpin_left 34
#define Switchpin_right 36

// Servo ----------------
#define servo_pin 38 // servo1 38, servo2 40, servo3 42
Servo plockservo;

#define servo_pin2 40 // servo1 38, servo2 40, servo3 42
Servo storage_servo;

#define servo_pin3 42 // servo1 38, servo2 40, servo3 42
Servo putter_servo;



//--------Changeable variables-----------
int tolerance = 2;                 // Allowed error before the translation correction algorithm is implemented
int tolerance_angle = 1;           // Allowed error before the rotational correction algoritm is implemented
float angle = 15;                  // Angle of the sensors from the vehicle
int start_pwm = 200;               // Base PWM before modifiers. from 0 to 255
int correction_pwm = 60;
int startup_sound = 4;             // if 1 = sing if 0 dont sing
bool plock = false;
int PWM_3 = 60;                    // PWM för zon 3. Måste kalibreras!
bool corrected = false;


// --------Defining variables------------
long travelTime_FL;
long travelTime_FR;
long travelTime_BL;
long travelTime_BR;

int distance_FL;
int distance_FR;
int distance_BL;
int distance_BR;

double real_distance_FL;
double real_distance_FR;
double real_distance_BL = 0;
double real_distance_BR;

double last_real_distance_FL;
double last_real_distance_FR;
double last_real_distance_BL;
double last_real_distance_BR;

int sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7, sensorValue8;
int sensorValue9, sensorValue10, sensorValue11, sensorValue12, sensorValue13, sensorValue14, sensorValue15, sensorValue16;

String op = "0p";

String BTBYTE;  // Received signal string.

String Empty;
char tempCommand[4];// variable for new string
String Command;

SoftwareSerial BTSerial(13,12); // RX , TX

char State1;
char Zone;
char Instruction;
String Ack;
char direction;
String sendback;


//int i = 0;
bool ignore = true;
int Tape = 0;
bool L_lost;
bool R_lost;
bool read_error;
char last_inst;
char last_Zone;
bool orthogonal;
int counter_correct = 0;
bool I_korridor = true;

int Switch_left;
int Switch_right;

// -------------------------Setup-------------------
void setup()
{
  pinMode(trigpin_FL, OUTPUT);
  pinMode(echopin_FL, INPUT);
  pinMode(trigpin_FR, OUTPUT);
  pinMode(echopin_FR, INPUT);
  pinMode(trigpin_BL, OUTPUT);
  pinMode(echopin_BL, INPUT);
  pinMode(trigpin_BR, OUTPUT);
  pinMode(echopin_BR, INPUT);

  pinMode(FWDpin_FL, OUTPUT);
  pinMode(BWDpin_FL, OUTPUT);
  pinMode(FWDpin_FR, OUTPUT);
  pinMode(BWDpin_FR, OUTPUT);
  pinMode(FWDpin_BL, OUTPUT);
  pinMode(BWDpin_BL, OUTPUT);
  pinMode(FWDpin_BR, OUTPUT);
  pinMode(BWDpin_BR, OUTPUT);

  Serial.begin(9600);           // Serial Communication is starting with 9600 of baudrate speed
  BTSerial.begin(9600);

  // IR sensorernas input.
  pinMode(SENSORA_F1,INPUT);
  pinMode(SENSORA_F2,INPUT);
  pinMode(SENSORA_F3,INPUT);
  pinMode(SENSORA_F4,INPUT);
  pinMode(SENSORA_F5,INPUT);
  pinMode(SENSORA_F6,INPUT);
  pinMode(SENSORA_F7,INPUT);
  pinMode(SENSORA_F8,INPUT);

  pinMode(SENSORA_B1,INPUT);
  pinMode(SENSORA_B2,INPUT);
  pinMode(SENSORA_B3,INPUT);
  pinMode(SENSORA_B4,INPUT);
  pinMode(SENSORA_B5,INPUT);
  pinMode(SENSORA_B6,INPUT);
  pinMode(SENSORA_B7,INPUT);
  pinMode(SENSORA_B8,INPUT);

  pinMode(buzzer_pin, OUTPUT);

  plockservo.attach(servo_pin);
  storage_servo.attach(servo_pin2);
  putter_servo.attach(servo_pin3);
  
  plockservo.write(120);
  storage_servo.write(120);
  putter_servo.write(90); 
}

// ----------Functions------------------------------------
void raise_storage() {
    storage_servo.write(180);
    for(int pos=180; pos >= 0 ; pos-=10){
        storage_servo.write(pos);
        delay(15);
    }
}

void lower_storage() {
      storage_servo.write(0);
    for(int pos=0; pos <= 120; pos+=10){
        storage_servo.write(pos);
        delay(15);
  }
}

void readUSdist() //läser av realdistance för alla ultraljud sensorer
{
    real_distance_FL = real_distance(ultraSensor(trigpin_FL, echopin_FL), angle);
    real_distance_FR = real_distance(ultraSensor(trigpin_FR, echopin_FR), angle);
    real_distance_BL = real_distance(ultraSensor(trigpin_BL, echopin_BL), angle);
    real_distance_BR = real_distance(ultraSensor(trigpin_BR, echopin_BR), angle);
}

void readIRData() // läser alla IR sensorer
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

        delay(10); 
}
       
void center_on_tape() // Centrerar AGV på en tejp med IR-sensorramperna
{
  int errorFront;
  int errorBack;
  bool backCentered = false;
  bool frontCentered = false;
  int timer = 0;
    while(true)
    {
        while(true)
        {
            readIRData();
            int frontLeftUS = sensorValue1 + sensorValue2 + sensorValue3 + sensorValue4;
            int frontRightUS = sensorValue5 + sensorValue6 + sensorValue7 + sensorValue8;
            errorFront = frontLeftUS - frontRightUS;
            frontCentered = false;
            

            if(sensorValue4 > 500 && sensorValue5 > 500)
            {
                frontCentered = true;
                rotate_stop();
                break;
            }

            if(errorFront > 0 && !frontCentered)
            {
            rotate_cclkw_front(90);
            }
            else if(errorFront < 0 && !frontCentered)
            {
            rotate_clkw_front(90);
            }
            else
            {
                rotate_stop();
                break;
            }
    
        }
        while(true)
        {
            readIRData();
            int backRightUS = sensorValue9 + sensorValue10 + sensorValue11 + sensorValue12;
            int backLeftUS = sensorValue13 + sensorValue14 + sensorValue15 + sensorValue16;
            errorBack = backRightUS - backLeftUS;

            backCentered = false;
    
            if(sensorValue12 > 540 && sensorValue13 > 540)
            {
                backCentered = true;
                rotate_stop();
                break;
            }

            if(errorBack > 0 && !backCentered)
            {
                rotate_cclkw_rear(90);
            }
            else if(errorBack < 0 && !backCentered)
            {
                rotate_clkw_rear(90);
            }
            else
            {
                rotate_stop();
                break;
            }
        }

        readIRData();
    
        if((errorFront > -180 && errorFront < 180 && errorBack > -180 && errorBack < 180) && sensorValue4 > 500 && sensorValue5 > 500 && sensorValue12 > 500 && sensorValue13 > 500)
        {
            break;
        }
            frontCentered = false;
            backCentered = false;
    
        if(timer == 30)
        {
            break;
        }
        ++timer;
    }
}

void correct_FWD(int PWM_C) // corrigera AGV medans den kör
{
    if(abs(real_distance_FL-real_distance_FR) > 8 || abs(real_distance_BL-real_distance_BR) > 8 )
    {
                translate_stop();
                translate_BWD(PWM_C);
                delay(150);
                translate_stop();
                delay(150);
                                 
                orthogonal = false;
                while(!orthogonal)
                {
                    readUSdist();
                    orthogonal = rotational_correction(real_distance_FL,real_distance_BL,real_distance_FR,real_distance_BR,1,PWM_3 - 15);
                }

                while(true){
                    if(real_distance_FR + real_distance_BR > real_distance_FL + real_distance_BL)
                    {
                    translate_right(PWM_3+18);
                    }
                    else if(real_distance_FR + real_distance_BR < real_distance_FL + real_distance_BL)
                    {
                    translate_left(PWM_3+18);   
                    }
                    readUSdist();
                    if (abs((real_distance_FL + real_distance_BL) - (real_distance_FR+real_distance_BR)) < tolerance)
                    {
                        translate_stop();
                        break;
                    }
                }
                orthogonal = false;
                while(!orthogonal)
                {
                    readUSdist();
                    orthogonal = rotational_correction(real_distance_FL,real_distance_BL,real_distance_FR,real_distance_BR,1,PWM_3 - 15);
                }

                translate_FWD(PWM_C);
    }
}

void Plockat() // funktion för att plocka klossen från hyllan
{
    delay(25);
    for(int pos=0; pos <= 120; pos+=10){
        plockservo.write(pos);
        delay(15);
    }
}

void to_hylla() // Kör fram till hyllkant
{
    plockservo.write(0); // fäller upp plockarmen
        center_on_tape(); // centrerar på tejpen
        translate_FWD(PWM_3+8);
        while(true) // kör fram tills båda brytarna är intryckta
        { 
            Switch_left = digitalRead(Switchpin_left);
            Switch_right = digitalRead(Switchpin_right);
            if (Switch_left == 1 && Switch_right == 1)
            {
                translate_stop();
                break;
            }
        }
}

String Tapestop(int nr, int PWM,String outmes) // funktion för att stanna vid tejp nr
{
Tape = 0;
I_korridor = false;
        translate_FWD(PWM);
        counter_correct = 0;
        while(Tape < nr)
        {
            readIRData();
            
            if (I_korridor == true)
            {
                correct_FWD(PWM);          
            }              
            if (((sensorValue9+sensorValue10+sensorValue11+sensorValue12+sensorValue13+sensorValue14+sensorValue15+sensorValue16 + 2400) / 8) < ((sensorValue1+sensorValue2+sensorValue3+sensorValue4+sensorValue5+sensorValue6+sensorValue7+sensorValue8) / 8))
            {
                Tape = Tape + 1;
                I_korridor = true;
                if (Tape == nr)
                {
                    delay(190);
                    quickbrake(PWM);
                    total_correction(tolerance_angle, tolerance, PWM_3+25, angle);
                    total_correction(tolerance_angle, tolerance, PWM_3+25, angle);
                    outmes[2] = 'p';
                }
                else
                {
                    sendback = outmes[0] + op;
                    BTSerial.println(sendback);
                    sendback = "";
                }
                delay(220);
            }
        }
        translate_stop();
        return outmes;
}

String Instructions(char inst, int PWM, String Outmes_inst, char Zone) // Utför instruktion beroende på vad ÖS skickat
{

    switch (inst)   // Instruktion
    {
    case 's':
        translate_stop();
        rotate_stop();
        last_inst = 's';
        Outmes_inst[1]='1';  //Klar
        break;
    
    case 'f':   // Framåt

    readUSdist();

    if (Zone == '3')    // om zon 3 och AGV ska fram till hyllkanten och stanna
    { 
        center_on_tape();
        to_hylla();
        
        Outmes_inst[1] = '1';
        last_inst = 'f';
        break;
    }

    // Andra zoner än zon 3
    translate_FWD(PWM);
    if (last_inst == 'f')
    {
        delay(1150); 
        I_korridor = false;
    }
    else{
        total_correction(tolerance_angle, tolerance, PWM_3+25, angle);
        total_correction(tolerance_angle, tolerance, PWM_3+25, angle);
        translate_FWD(PWM);
    }
        L_lost = false;
        R_lost = false;
        while (true){
            
        last_real_distance_FL = real_distance_FL;
        last_real_distance_FR = real_distance_FR;
        last_real_distance_BL = real_distance_BL;
        last_real_distance_BR = real_distance_BR;
        
        readUSdist();
        readIRData();
            if (((sensorValue9+sensorValue10+sensorValue15+sensorValue16) / 4) > ((sensorValue1+sensorValue2+sensorValue7+sensorValue8 + 800) / 4))
            {
                  sendback = Outmes_inst[0] + op;
                  BTSerial.println(sendback);
                  sendback = "";
                  I_korridor = true;

                delay(50);
            }

        if(real_distance_FL > (last_real_distance_FL + 26))
        {
            L_lost = true;
        }
        if(real_distance_FR > (last_real_distance_FR + 26))
        {
            R_lost = true;
        }

        if (Zone == '4'){
            if(L_lost == true || R_lost == true){
                delay(300);
                break;
            }
        }
        else if(R_lost && L_lost)
        {
            if (last_Zone == '1')
            {
                delay(240);
            }
            else{
            delay(300);
            }
            break;
        }
            if(Zone != '1' && L_lost == false && R_lost == false && I_korridor == true)
            {
                correct_FWD(PWM);            
            }
        }
        translate_stop();
        delay(200);
        //I_korridor = false;
        Outmes_inst[1]='1';  //Klar
        last_inst = 'f';
        break;
    
    case 'b':
        translate_BWD(PWM);
        delay(250);
        // ska den inte stanna här?
        Outmes_inst[1]='1';  //Klar
        last_inst = 'b';
        break;

    case 'l':   // Plocka kub till Vänster 
        center_on_tape();
        to_hylla(); // kör fram till hyllkanten

        pickLeftCube(); // Hitta kub till vänster
        
        //kör fram lite för att korrigera fel på hjul
        translate_FWD(115);
         while(true)    // kör fram tills båda brytarna är intryckta
         { 
            Switch_left = digitalRead(Switchpin_left);
            Switch_right = digitalRead(Switchpin_right);
            if (Switch_left == 1 && Switch_right == 1)
            {
                translate_stop();
                break;
            }
        }
        
        Plockat();  // Plocka kub
        sing(1);    // spela ljud
        plockservo.write(0);
        // kör tillbaka till mitten av korridoren
        translate_BWD(100);
        delay(200);
        quickbrake(100);
        delay(50);
        translate_right(150); 
        while(true){
            readIRData();
            if(sensorValue3+sensorValue4+sensorValue5+sensorValue6 > 1200 && sensorValue11+sensorValue12+sensorValue13+sensorValue14 > 1200)
            {
                translate_left(150);
                delay(80);
                translate_stop();
                break;
            }
        }
        translate_BWD(100);
        delay(450);
        quickbrake(100);
        delay(50);
        plockservo.write(120); // fäll ner plockarmen

        Outmes_inst[2]='u';  // Plockning utförd
        Outmes_inst[1]='1';  // Klar
        last_inst = 'l';
        break;

    case 'r':   // Plocka kub till Höger     
        center_on_tape();
        to_hylla(); // kör fram till hyllkanten
        
        pickRightCube();    // Hitta kub till höger

        translate_FWD(115);
         while(true)    // kör fram tills båda brytarna är intryckta
         { 
            Switch_left = digitalRead(Switchpin_left);
            Switch_right = digitalRead(Switchpin_right);
            if (Switch_left == 1 && Switch_right == 1)
            {
                translate_stop();
                break;
            }
        }

        Plockat();  // Plocka kub
        sing(1);    // spela ljud
        plockservo.write(0);
        // Kör tillbaka till mitten av korridoren
        translate_BWD(100);
        delay(200);
        quickbrake(100);
        delay(50);
        translate_left(150); 
        while(true)
        {
            readIRData();
            if (sensorValue3+sensorValue4+sensorValue5+sensorValue6 > 1200 && sensorValue11+sensorValue12+sensorValue13+sensorValue14 > 1200)
            {
                translate_right(150);
                delay(85);
                translate_stop();
                break;
            }
        }

        delay(50);
        translate_BWD(100);
        delay(450);
        quickbrake(100);
        delay(50);
        plockservo.write(120); // fäll ner plockarmen
        
        Outmes_inst[2]='u';  // Plockning utförd
        Outmes_inst[1]='1';  // Klar
        last_inst = 'r';
        break;

    case 'm':   // Plocka kub i Mitten
        plockservo.write(0);    // Rotera ner plockmekanismen 
        center_on_tape();       // Centrera på tejp
        to_hylla();             // kör fram till hyllkanten
    
        Plockat();              // Plocka kub
        sing(1);                // Spela låt
    
        // kör tillbaka till mitten av korridoren
        translate_BWD(100);
        delay(500);
        quickbrake(100);
        plockservo.write(130); // Rotera upp plockmekanismen 
    
        Outmes_inst[2]='u';  // Plockning utförd
        Outmes_inst[1]='1';  // Klar
        last_inst = 'm';
        translate_stop();
        break;

    case 'h': // Roterar 90 grader medurs
    delay(200);
        rotate_centered_clkw(PWM);
        delay(644);
        rotate_stop();
        delay(250);
        Outmes_inst[1]='1';
        // last_inst = 'h';       
        break;

    case 't': // Rotera 180 grader
        rotate_centered_clkw(start_pwm);
        delay(1600);
        rotate_stop();
        center_on_tape();
        Outmes_inst[1]='1';
        last_inst = 't';       
        break;
    
    case 'L':   // Spela låt
        sing(5);
        Outmes_inst[1] = '1';
        last_inst = 'L';
        break;

    case 'v': // Roterar 90 grader moturs
        delay(200);
        rotate_centered_cclkw(PWM);
        delay(632);
        rotate_stop();
        delay(250);
        Outmes_inst[1]='1';  
        //last_inst = 'v';     
        break;

    case 'x': // Stanna på tejp nr 1
        Outmes_inst = Tapestop(1,PWM,Outmes_inst);
        Outmes_inst[1]='1';  //Klar
        last_inst = 'x';
        break;

    case 'y': // Stanna på tejp nr 2
        Outmes_inst = Tapestop(2,PWM,Outmes_inst);
        Outmes_inst[1]='1';  //Klar
        last_inst = 'y';
        break;

    case 'z': // Stanna på tejp nr 3
        Outmes_inst = Tapestop(3,PWM,Outmes_inst);  
        Outmes_inst[1]='1';  //Klar
        last_inst = 'z';
        break;

    case 'C':   // Bara korrektion
        total_correction(tolerance_angle, tolerance, PWM_3+10, angle);  // Korrektion
        Outmes_inst[1] = '1'; //Klar
        last_inst = 'C';
        break;

    case 'T':
        center_on_tape();       // Centrera på tejp
        Outmes_inst[1] = '1';   //Klar
        break;
    
    case 'w':   // Slutkommando
      rotate_centered_clkw(210); // Rotera höger
      sing(2);                   // Spela låt
      translate_stop();          // Stanna
      Outmes_inst[1] = '1';      // Klar
      last_inst = 'w';
      break;

    case 'a':   //Avlastning
        to_hylla();                 // Kör fram till hylla
        raise_storage();            // Lyft förvarings rampen
        delay(300);
        putter_servo.write(0);      // Knuffa ut klossarna 
        delay(1450);
        putter_servo.write(90);     // Knuffa ut klossarna 
        delay(500);
        putter_servo.write(180);    // Knuffa ut klossarna 
        delay(1450);
        putter_servo.write(90);     // Knuffa ut klossarna 
        delay(500);
        lower_storage();            // Sänk tillbaka förvarings rampen
        delay(200);
        sing(1);                    // Spela låt
        translate_BWD(100);         // Kör bakåt
        delay(450);
        translate_stop();           // Stanna
        delay(450);
        plockservo.write(120);      // Sänk plockmekanism
        Outmes_inst[1] = '1';       // Klar
        Outmes_inst[2] = 'k';       // Avlastat till ÖS
        last_inst = 'a';
    break;

    default:
        Outmes_inst[2] = 'f';   // Ogiltig instruktions karaktär.
        break;
    }
    last_Zone = Zone;
    return Outmes_inst;
}


String readBluetoothData(String BTBYTE, int PWM, bool plock)    //Behandlar meddelandet från ÖS
{
    
    String Out_mes="000";   // Return string.
    // Dela upp BTBYTE meddelanden (*/*/*/*).
    Out_mes[0]=BTBYTE[0];   // Ack
    Ack = Out_mes[0];       // Ack.
    State1=BTBYTE[1];       // Auto eller manuell.
    Zone=BTBYTE[2];         // Vilken zon.
    Instruction=BTBYTE[3];  // Instruktion.
  
    switch (State1) // Manuell eller automatisk körning
    {
    case '0':   // Manuell
    case '1':   // Automatisk
            switch (Zone)   // Zoner
            {
                case '1':   // Zon 1
                        Out_mes=Instructions(Instruction, PWM, Out_mes, Zone);
                    break;
    
                case '2':   // Zon 2
                        Out_mes=Instructions(Instruction, PWM, Out_mes, Zone);
                    break;
        
                case '3':   // Zon 3
                        Out_mes=Instructions(Instruction, PWM_3, Out_mes,Zone);
                    break;
                
                case '4': //Zon 4 
                         Out_mes=Instructions(Instruction, PWM, Out_mes,Zone);
                         break;

                default:    // Ingen zone.
                    Out_mes[2]='f';    // Ogiltig instruktions karaktär.
                    break;
            }
        break;
        
    default:
        break;
    }
    return Out_mes; 
}

//-----------Main loop-------------------------------------
void loop()
{
    String INBYTE;  // Transmitting signal string.
    if(BTSerial.available())    // Meddelande från ÖS till AGV    
    {
        BTBYTE=BTSerial.readString(); // Läser det nya meddelandet från ÖS 
        //bygger ny string av BTbyte som inte har det osynliga tecknet
        Command ="";
        read_error = false;
        for (int i = 0; i < 4; ++i)
        {
            if(BTBYTE[i]=='\0')
            {
                read_error = true;
                break;
            }
            else
            {
                Command = Command + BTBYTE[i];
            }
        }
        if (read_error == false)
        {
            Serial.println(Command);
            INBYTE = readBluetoothData(Command, start_pwm, plock); // Behandla meddelandet. Returnera meddelande som ska tillbaka till ÖS.
            plock = true;
            BTSerial.println(INBYTE);     // Skickar meddelande till ÖS    
        }
        else
        {
            BTSerial.println("Error reading input"); // Error meddelande skickas till ÖS
        }
    }
}


