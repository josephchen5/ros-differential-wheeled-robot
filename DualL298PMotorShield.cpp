#include "DualL298PMotorShield.h"

// Constructors ////////////////////////////////////////////////////////////////

DualL298PMotorShield::DualL298PMotorShield()
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

DualL298PMotorShield::DualL298PMotorShield(unsigned char INA1, unsigned char INB1, 
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
void DualL298PMotorShield::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_INA1,OUTPUT);
  pinMode(_INB1,OUTPUT);
  pinMode(_PWM1,OUTPUT);
//  pinMode(_EN1DIAG1,INPUT);
//  pinMode(_CS1,INPUT);
  pinMode(_INA2,OUTPUT);
  pinMode(_INB2,OUTPUT);
  pinMode(_PWM2,OUTPUT);
//  pinMode(_EN2DIAG2,INPUT);
//  pinMode(_CS2,INPUT);

  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
  // Timer 1 configuration
  // prescaler: clockI/O / 1
  // outputs enabled
  // phase-correct PWM
  // top of 400
  //
  // PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  TCCR1A = 0b10100000;
  TCCR1B = 0b00010001;
  ICR1 = 400;
  #endif


}
// Set speed for motor 1, speed is a number betwenn -400 and 400
void DualL298PMotorShield::setM1Speed(int speed)
{
  unsigned char reverse = 0;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
  OCR1A = speed;
  #else
  analogWrite(_PWM1,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  #endif
  if (speed == 0)
  {
    digitalWrite(_INA1,LOW);   // Make the motor coast no
    digitalWrite(_INB1,LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(_INA1,LOW);
    digitalWrite(_INB1,HIGH);
  }
  else
  {
    digitalWrite(_INA1,HIGH);
    digitalWrite(_INB1,LOW);
  }  
}

// Set speed for motor 2, speed is a number betwenn -400 and 400
void DualL298PMotorShield::setM2Speed(int speed)
{
  unsigned char reverse = 0;

  if (speed < 0)
  {
    speed = -speed;  // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
  if (speed > 400)  // Max 
    speed = 400;
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
  OCR1B = speed;
  #else
  analogWrite(_PWM2,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  #endif 

  if (speed == 0)
  {
    digitalWrite(_INA2,LOW);   // Make the motor coast no
    digitalWrite(_INB2,LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(_INA2,LOW);
    digitalWrite(_INB2,HIGH);
  }
  else
  {
    digitalWrite(_INA2,HIGH);
    digitalWrite(_INB2,LOW);
  }
}

// Set speed for motor 1 and 2
void DualL298PMotorShield::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}


