#ifndef DualL298PMotorShield_h
#define DualL298PMotorShield_h

#include <Arduino.h>

class DualL298PMotorShield
{
  public:  
    // CONSTRUCTORS
    DualL298PMotorShield(); // Default pin selection.
    DualL298PMotorShield(unsigned char INA1, unsigned char INB1, 
                           unsigned char INA2, unsigned char INB2); // User-defined


    // PUBLIC METHODS
    void init(); // Initialize TIMER 1, set the PWM to 20kHZ. 
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
 //   void setM1Brake(int brake); // Brake M1. 
 //   void setM2Brake(int brake); // Brake M2.
 //   void setBrakes(int m1Brake, int m2Brake); // Brake both M1 and M2.



  private:
    //这里定义了L298P的控制引脚，读者可以根据自己买的L298P灵活修改
    unsigned char _INA1;  //M1马达转动方向IN1，接Arduino D5
    unsigned char _INB1;  //M1马达转动方向IN2，接Arduino D6
    static const unsigned char _PWM1 = 9;
  //  unsigned char _EN1DIAG1;
  //  unsigned char _CS1;
    unsigned char _INA2;  //M2马达转动方向IN3，接Arduino D7
    unsigned char _INB2;  //M2马达转动方向IN4，接Arduino D8
    static const unsigned char _PWM2 = 10;
  //   unsigned char _EN2DIAG2;
  //  unsigned char _CS2;

};

#endif
