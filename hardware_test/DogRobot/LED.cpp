#include"LED.h"
#include"Arduino.h"


LED::LED(byte p,bool state):pin(p)
{
   pinMode(pin,OUTPUT);
   digitalWrite(pin,state);
}

LED::~LED()
{
    disattach();
} 


void LED::on()
{
    digitalWrite(pin,HIGH);
}

void LED::off()
{
   digitalWrite(pin,LOW);
}

bool LED::getState()
{
    return digitalRead(pin);
}

void LED::disattach()        //引脚回收，恢复到上电状态
{
    digitalWrite(pin,LOW);
    pinMode(pin,INPUT);
}