// I2C Digital Potentiometer
// by Nicholas Zambetti <http://www.zambetti.com>
// and Shawn Bonkowski <http://people.interaction-ivrea.it/s.bonkowski/>

// Demonstrates use of the Wire library
// Controls AD5171 digital potentiometer via I2C/TWI

// Created 31 March 2006

// This example code is in the public domain.

// This example code is in the public domain.

// Modified by Ross 21 Jan 2020


#include "printf.h"
#include <Wire.h>


void setup()
{
  Serial.begin(115200);
  printf_begin();

  Wire.begin(); // join i2c bus (address optional for master)
}


void loop()
{
  static uint8_t val;


//  // AD5171
//  if (val >= 64) {              // if reached 64th position (max)
//    val = 0;                    // start over from lowest value
//  }
//  Wire.beginTransmission(44);   // transmit to device #44 (0x2c); device address is specified in datasheet
//  Wire.write(byte(0x00));       // sends instruction byte  
//  Wire.write(val);              // sends potentiometer value byte (0-63)  

  // AD5248
  Wire.beginTransmission(0x58 >> 1);  // slave address with AD0 = AD1 = 0, right shifted for Wire's 7-bit addressing
  Wire.write(byte(0x00));             // AD (subaddress select) = 0, SD (short wiper to Bx terminal) = 0 (disable)
//  Wire.write(byte(0x80));             // AD (subaddress select) = 1, SD (short wiper to Bx terminal) = 0 (disable)
  Wire.write(val);                    // data byte (0-255)

  Wire.endTransmission();       // stop transmitting

//  Serial.print("sent val=");
//  Serial.println(val);

  val++;
  delay(50);
}

