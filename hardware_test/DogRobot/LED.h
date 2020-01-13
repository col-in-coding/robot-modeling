#ifndef _LED_H
#define _LED_H

//导入Arduino核心头文件
#include"Arduino.h"  


class LED
{
     private:
          byte pin;        //控制led使用的引脚
     
     
     public:
          
          LED(byte p , bool state=LOW );   //构造函数
          
          ~LED();          //析构函数

          byte getPin();   //获取控制的引脚
          
          void on();      //打开LED

          void off();     //关闭LED

          bool getState();  //获取LED状态
          void disattach(); //释放引脚与LED的绑定，使得引脚可以控制其他的东西

};


#endif