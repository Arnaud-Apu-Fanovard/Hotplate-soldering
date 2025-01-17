#include <Arduino.h>
#include "Heater.h"

Heater::Heater(int pin)
{
    Heater_pin = pin;
    pinMode(Heater_pin,OUTPUT);
}

void Heater::Command_Heater(unsigned long TimeZeroCrossing,unsigned long cmd) //cmd = Microsecondes 0 - 10000
{   
    if (cmd > 10000)
    {cmd = 10000;}
    else if (cmd <0)
    {cmd = 0;}
    
    if (micros() >= (TimeZeroCrossing + cmd))
    {digitalWrite(Heater_pin, HIGH);}
    else
    {digitalWrite(Heater_pin, LOW);}  
}

void Heater::Heater_On()
{
    digitalWrite(Heater_pin, HIGH);
}

void Heater::Heater_Off()
{
    digitalWrite(Heater_pin, LOW);
}