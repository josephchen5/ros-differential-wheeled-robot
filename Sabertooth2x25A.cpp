#include "Sabertooth2x25A.h"
#include <Servo.h>
Servo motor1;  // create servo object to control motor1
Servo motor2;  // create servo object to control motor2

// Constructors ////////////////////////////////////////////////////////////////

Sabertooth2x25A::Sabertooth2x25A()
{
  //Pin map
  _INA1 = 5;
  _INB1 = 6;
//  _EN1DIAG1 = 6;
//  _CS1 = A0; 
  _INA2 = 7;
  _INB2 = 8;
//  _EN2DIAG2 = 12;
//  _CS2 = A1;
}

Sabertooth2x25A::Sabertooth2x25A(unsigned char INA1, unsigned char INB1, 
                                               unsigned char INA2, unsigned char INB2)
{
  //Pin map
  //PWM1 and PWM2 cannot be remapped because the library assumes PWM is on timer1
  _INA1 = INA1;
  _INB1 = INB1;
//  _EN1DIAG1 = EN1DIAG1;
//  _CS1 = CS1;
  _INA2 = INA2;
  _INB2 = INB2;
//  _EN2DIAG2 = EN2DIAG2;
//  _CS2 = CS2;
}



// Public Methods //////////////////////////////////////////////////////////////
void Sabertooth2x25A::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_INA1,OUTPUT);
  pinMode(_INB1,OUTPUT);
//  pinMode(_PWM1,OUTPUT);
  motor1.attach(_PWM1);  // attaches the servo on pin 9 to the servo object

//  pinMode(_EN1DIAG1,INPUT);
//  pinMode(_CS1,INPUT);
  pinMode(_INA2,OUTPUT);
  pinMode(_INB2,OUTPUT);
 // pinMode(_PWM2,OUTPUT);
  motor2.attach(_PWM2);  // attaches the servo on pin 9 to the servo object
//  pinMode(_EN2DIAG2,INPUT);
//  pinMode(_CS2,INPUT);


}
// Set speed for motor 1, speed is a number betwenn -400 and 400
void Sabertooth2x25A::setM1Speed(int speed)
{

  speed = map(speed, -400, 400, 40, 140);
  motor1.write(speed); 

}

// Set speed for motor 2, speed is a number betwenn -400 and 400
void Sabertooth2x25A::setM2Speed(int speed)
{

  speed = map(speed, -400, 400, 32, 132);
  motor2.write(speed); 

}

// Set speed for motor 1 and 2
void Sabertooth2x25A::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}


