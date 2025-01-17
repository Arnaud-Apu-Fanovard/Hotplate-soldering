#include <Arduino.h>

#define led_builtin 8
#define NTC_1 0
#define NTC_2 1
#define HotPlate 5


// Variable global 

bool ledstate;

double Beta = 3950;

float Temp_1;
float Temp_2;
float Temp;

bool CV_Hotplate;
bool CV_tempo;

float hysteresis;

unsigned long microTime;
unsigned long PreviousMicro;
double deltaT;
double SP_ramp;

float temperature_NTC(int Pin_NTC)
{
  float Sum_temperature = 0.0;
  float Temperature;
  double temp;
  for (int i = 0; i < 10; i++)
  { float sensorValue = analogRead(Pin_NTC);
    double U_NTC = (sensorValue/4096) * 3.3;
    double R_NTC = ((3.3-U_NTC)/U_NTC)*100000;
    double temp = 1/((1/298.15)+((1/Beta)*log(R_NTC/100000))) ;
    temp -= 273.15;
    Sum_temperature += temp;}
  Temperature = Sum_temperature / 10;
  return Temperature;
}

void actuators(bool CV)
{
  // digitalWrite(HotPlate, CV);
  digitalWrite(led_builtin, CV);
}

bool regulation_temp(float PV, float SP, float hysteresis)
{ 
  // Serial.print(SP-hysteresis);
  // Serial.print(SP+hysteresis);

  if (PV <= SP - hysteresis || (PV < SP + hysteresis && CV_tempo == LOW))
  {CV_tempo = LOW;}
  else
  {CV_tempo = HIGH;}
  return CV_tempo;
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

void setup() {
  Serial.begin(9600);
  Serial.println("Starting ESP32C3");

  pinMode(led_builtin, OUTPUT);
  pinMode(NTC_1, INPUT);
  pinMode(NTC_2, INPUT);

  pinMode(HotPlate, OUTPUT);
}

void loop() {

  Temp_1 = temperature_NTC(NTC_1);
  Temp_2 = temperature_NTC(NTC_2);
  Temp = (Temp_1+Temp_2)/2;

  Serial.print(">Temp:");
  Serial.println(Temp_1,8);
  Serial.print(">Ramp:");
  Serial.println(ramp_temp(250,1),10);

  CV_Hotplate = regulation_temp(Temp_1,40,2);
  actuators(CV_Hotplate);
}

