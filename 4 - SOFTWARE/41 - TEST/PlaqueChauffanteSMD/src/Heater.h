#ifndef Heater_
#define Heater_
#include <Arduino.h>

class Heater
{
    private :
        int Heater_pin;
    public :
        Heater(int);
        void Command_Heater(unsigned long,unsigned long);
        void Heater_On();
        void Heater_Off();
};

#endif