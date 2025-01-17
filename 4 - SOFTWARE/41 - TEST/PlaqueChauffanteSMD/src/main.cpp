#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Button.h>
#include "Heater.h"

#define led_builtin 8
#define NTC_1 0
#define NTC_2 1
#define pin_Zero_crossing 2
#define Bouton1 6
#define Bouton2 7

LiquidCrystal_I2C lcd(0x27,16,2); 
int GPIO_SDA = 8;              
int GPIO_SCL = 9;              


int ceramic[3]= {3, 4, 5};


Button bonton_1(Bouton1);
Button bonton_2(Bouton2);

Heater Heater_1(ceramic[0]);
Heater Heater_2(ceramic[1]);
Heater Heater_3(ceramic[3]);

// Variable global 

bool Start;

bool ledstate;

double Beta = 3950;

float Temp_1;
float Temp_2;
float Temp;
float Temp_NTC_Exte[3];

double ramp;

unsigned long microTime;
unsigned long PreviousMicro;
double deltaT;
double SP_ramp = 30;

long previous_ms ;
long delay_ms = 15;

double Temperature[3];

double Cmd_delay[3];
double Cmd_Pourcentage[3];


float T_Setpoint[3];
float T_filtre[3];
float T_Prev[3];
float T_filtre_prev[3];

float P_T[3];
float I_T[3];
float D_T[3];
float previousError_T[3];

float Kp_T = 0.4;//0.3 est bien
float Ki_T = 0.00005;//0.015
float Kd_T = 0.003;//0.01

float PIDvalue_T[3];

double time_starting_ramp;
long time_instant;

unsigned long TimeZeroCrossing;

bool start;


float temperature_NTC(int Pin_NTC,int average_number)
{
  float Sum_temperature = 0.0;
  float Temperature;
  double temp;
  for (int i = 0; i < average_number; i++)
  { float sensorValue = analogRead(Pin_NTC);
    // Serial.println(sensorValue);
    double U_NTC = (sensorValue/4096) * 3.3;
    double R_NTC = ((3.3-U_NTC)/U_NTC)*100000;
    double temp = 1/((1/298.15)+((1/Beta)*log(R_NTC/100000))) ;
    temp -= 273.15;
    Sum_temperature += temp;}
  Temperature = Sum_temperature / average_number;
  return Temperature;
}

void filtre_temperature()
{
  for (int i = 0; i < 2; i++)
  {
    T_filtre[i] = 0.854*T_filtre_prev[i] + 0.0728*Temperature[i]+ 0.0728*T_Prev[i];
    T_filtre_prev[i]= T_filtre[i];
    T_Prev[i] = Temperature[i];
  }
  
  
}

double ramp_temp(float SP, float ramp)
{
  microTime = micros();
  
  if (SP_ramp >= SP)
  {
    SP_ramp = SP;
  }
  else
  {
    deltaT = (microTime - PreviousMicro)/1.0e6;
    SP_ramp += deltaT*ramp;
  }
  PreviousMicro = microTime;
  return SP_ramp; 
}

double ramp_soldering_SMD()
{

  microTime = micros();
  deltaT = (microTime - PreviousMicro)/1.0e6;

  if (time_instant >= 0 && time_instant < 140000 )
  {
    SP_ramp += deltaT*1.333;
  }
  else if (time_instant >= 140000 && time_instant < 240000)
  {
    SP_ramp += deltaT*0.333;
  }
  else if (time_instant >= 240000 && time_instant < 300000)
  {
    SP_ramp += deltaT*1;
  }
  else
  { 
    if (SP_ramp >= 30)
    {
      SP_ramp -= deltaT*1;
    }
    else
    {
      SP_ramp = 30;
    }
  }
  
  PreviousMicro = microTime;
  return SP_ramp;
}

void regulation_Opto_Triac(float Temp[3]) //SP=point de consigne ,  PV=mesure
{
  float error_T[3];
  for (int i = 0; i < 3; i++)
  {

    error_T[i] = T_Setpoint[0] - Temp[i];
    

    P_T[i] = error_T[i];
    I_T[i] = I_T[i] + error_T[i];
    D_T[i] = error_T[i] - previousError_T[i];

    PIDvalue_T[i] = (Kp_T * P_T[i]) + (Ki_T * I_T[i]) + (Kd_T * D_T[i]);
    previousError_T[i] = error_T[i];
    

    if (PIDvalue_T[i] >=100)
    {
      Cmd_Pourcentage[i]=100;
    }
    else if (PIDvalue_T[i] <=0)
    {
      Cmd_Pourcentage[i]=0;
    }
    else
    {
      Cmd_Pourcentage[i] = PIDvalue_T[i];
    }
    
    Cmd_delay[i] = (100 - Cmd_Pourcentage[i])*100; //en micros secondes (donc pour 90% de puissance, ça fera un délai de 1000 micros secondes donc 1ms ce qui équivaut bien à 10% de 10ms)
  }
   
}

void trigger_zero_crossing()
{ 
  if (start == 1)
  {
    digitalWrite(ceramic[0],LOW);
    for (int i = 0; i < 1; i++)
    {
      if (Cmd_delay[i] >= 9850)
    {
      digitalWrite(ceramic[i],LOW);
    }
    else if (Cmd_delay[i] >= 0 && Cmd_delay[i] < 50)
    {
      digitalWrite(ceramic[i],HIGH);
    }
    else
    {
      delayMicroseconds(Cmd_delay[i]);
      digitalWrite(ceramic[i],HIGH);    
    }
    }
  }
  else
  {
    for (int i = 0; i < 1; i++)
    {
    digitalWrite(ceramic[i],LOW);
    }
  }
  
  
  
}

void trigger()
{
  TimeZeroCrossing = micros();
}

void impulsion_Dirac()
{
  if (time_instant >= 20000 && Temp <= 35)
  {
    T_Setpoint[0] = 200;
    start = 1;

  }
  
}

void Teleplot_Print()
{
  Serial.print(">SP:");
  Serial.println(T_Setpoint[0],8);
  Serial.print(">Cmd_1 %:");
  Serial.println(Cmd_Pourcentage[0],8);
  Serial.print(">Cmd_2 %:");
  Serial.println(Cmd_Pourcentage[1],8);
  Serial.print(">Cmd_3 %:");
  Serial.println(Cmd_Pourcentage[2],8);
  
  Serial.print(">Temp 1:");
  Serial.println(Temp_1,8);
  Serial.print(">Temp 2:");
  Serial.println(Temp_2,8);
  Serial.print(">T_filtre 1:");
  Serial.println(Temp_NTC_Exte[0],8);
  Serial.print(">T_filtre 2:");
  Serial.println(Temp_NTC_Exte[1],8);
  Serial.print(">T_filtre 3:");
  Serial.println(Temp_NTC_Exte[2],8);  
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting ESP32C3");

  //pinMode(led_builtin, OUTPUT);
  pinMode(NTC_1, INPUT);
  pinMode(NTC_2, INPUT);

  pinMode(ceramic[0], OUTPUT);
  pinMode(ceramic[1], OUTPUT);
  pinMode(ceramic[2], OUTPUT);

  bonton_1.begin();
  bonton_2.begin();

  Wire.begin(GPIO_SDA,GPIO_SCL);
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Starting ESP32C3");
  delay(2000);
  lcd.clear();

  pinMode(pin_Zero_crossing, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_Zero_crossing), trigger, RISING);

  time_starting_ramp = millis();

}

void loop() {

  time_instant = millis();

  Temp_1 = temperature_NTC(NTC_1,10);
  Temp_2 = temperature_NTC(NTC_2,10);
  Temperature[0] = Temp_1;
  Temperature[1] = Temp_2;
  filtre_temperature();

  Temp_NTC_Exte[0] = T_filtre[0];
  Temp_NTC_Exte[1] = T_filtre[0];
  Temp_NTC_Exte[2] = T_filtre[0];

  T_filtre[2] = T_filtre[1];
  T_filtre[1] = (T_filtre[0]+T_filtre[1])/2;

  if (bonton_1.pressed())
  {
    Start = 1;
    lcd.clear();
  }

  if (bonton_2.pressed())
  {
    Start = 0;
    lcd.clear();
  }
  

  if (Start == 1)
  {
    lcd.setCursor(0,0);
    lcd.print("Heating...");
    T_Setpoint[0] = ramp_soldering_SMD();

    regulation_Opto_Triac(Temp_NTC_Exte);
    // Heater_1.Command_Heater(TimeZeroCrossing, Cmd_delay[0]);
    // Heater_2.Command_Heater(TimeZeroCrossing, Cmd_delay[1]);
    // Heater_3.Command_Heater(TimeZeroCrossing, Cmd_delay[2]);
    Heater_1.Heater_On();
    Heater_2.Heater_On();
    Heater_3.Heater_On();

    lcd.setCursor(11,0);
    lcd.print(T_Setpoint[0]);
  }
  else
  {
    lcd.setCursor(0,0);
    lcd.print("Cooling...");
    SP_ramp = 0;
    Heater_1.Heater_Off();
    Heater_2.Heater_Off();
    Heater_3.Heater_Off();
  }

    lcd.setCursor(1,1);
    lcd.print(T_filtre[0]);
    lcd.setCursor(10,1);
    lcd.print(T_filtre[2]);
  
  









  // T_Setpoint[0] = ramp_temp(200,0.5);
  // T_Setpoint[0] = ramp_soldering_SMD();

  // impulsion_Dirac();
  
  // if (start == 1)
  // {
  //   
  // }
  
  
  Teleplot_Print();

}
