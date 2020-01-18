 //  spi-demo.ino  by C.Spindler     9-19-2019
 //  name changed and minor changes 1-18-2020
 
 //  based on exercises in book "Exploring Arduino" 
 //  by Jeremy Blum, Chapter 9 SPI Buss. 
 //  similar to Listing 9-1 on page 192

 //  worked well on 1-13-2020 at 11:00 pm with ciruit 
 //  shown on drawing SPI-DEMO, Dwg. No. CFS-1-13-2020-A

 // take great care with current thru digital POT pins
 // max currrent is 2.5 ma. 

 // this was developred for an Arduino Mega 2560 board and uses
 // pins for it. I was originally to use 2 digital pots (MCP4231)
 // and trash from that is evident.  I will leave it because it 
 // is working, and clean it up in spi_demo_nano 
 
 /*
   We're using Arduino's SPI Library, see arduino.cc/en/reference/SPI
   for Arduino Mega2560 boards, by default,
   use Pin 52 = CLK , 50 = MISO and 51 = MOSI
   also, recommends setting pin 53 to OUTPUT to avoid accedentially
   borking the board if pin 53, the default SS pin for the Mega2560
   board, is activated.  SPI Library only supports Master Mode.

   MCP4231 datasheet says:
        -max speed of the chip is 10 mhz
        -data is bigendian MSBFIRST
        -SDI latches on rising edge
        -SDO latches on falling edge
        -data clock is idle BOTH high and low! (because this chip determines
          idle polarity on the fly, picking HIGH if SCK is HIGH when the chip
          is selected by driving CS to LOW[or picking LOW if SCK was LOW when 
          chip was selected]) so SPI_MODE0 or SPI_MODE3 are good.
        -lenght of comand line is determined by framing the command with CS 
         meaning that the chip uses 8 bit (or 16 bit) commands by selecting 
         chip, sending 8 bits (or 16 bits) and then deselecting chip.   
        -chip has 9 usefull data in bits, last bit of byte 1 and all 8 bits of 
         byte 2 

   Blum's example says Pin 11 = MOSI, 12 = MISO, 13 = CLK, but 
   that is only for Arduino Uno boards.
*/   

#include <SPI.h>

const int SS1 = 7; //Pin 7 is SS (slave select) 
                    // for device 1
const int SS2 = 4;  //Pin 4 is SS for device 2

const byte REG0 = B00000000;//Register 0 Write command
const byte REG1 = B00010000;//Register 1 Write command

/* see the SPI command formats on page 188, Figure 9-4
 * 
 * The text says we are using a 16 bit command format,  
 * which is 2 bytes long.  Of the first byte,
 *  the 4 most significant bits are the address: 
 *          0000 is 0 and 0001 is 1.
 *  the next 2 bits are the "command":         
 *          00 means "write data"
 *  the next 2 bits of the first byte are data,         
 *  and all 8 bits of the 2nd byte are data.        
 *   
 */




void setup() {
  
  // setup slave select pins for output
  pinMode(SS1, OUTPUT);
  pinMode(SS2, OUTPUT);

  // per instructions from arduino.cc/en/reference/SPI 
  // set the Mega2560's SS pin, 53, to OUTPUT to make it inoperative
  pinMode(53,OUTPUT);

  // initialize SPI 
//  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE3)); debug
  SPI.begin();   // depricated , but may work better
}

void setLed(int SS, int reg, int level) {

  digitalWrite(SS, LOW); // set the named pin LOW 
                         // to select that chip
  SPI.transfer(reg  ); // send the first byte to pick the register 
                       // number of the potentionmeter of the selected chip
  SPI.transfer(level); // send the 2nd byte to set the level of that 
                       // potentiometer
  digitalWrite(SS,HIGH); // de-select that chip                     
                                             
}


void loop() {

  setLed(SS1 ,REG0, 0);
  setLed(SS1 ,REG1, 0);
//  setLed(SS2 ,REG0, 0);
//  setLed(SS2 ,REG1, 0);
  delay(1000);
  setLed(SS1 ,REG0, 80);
  setLed(SS1 ,REG1, 80);
//  setLed(SS2 ,REG0, 65);
//  setLed(SS2 ,REG1, 65);
  delay(1000);
  setLed(SS1 ,REG0, 128);
  setLed(SS1 ,REG1, 128);
//  setLed(SS2 ,REG0, 128);
//  setLed(SS2 ,REG1, 128);
  delay(1000);

/*  for (int i = 0 ; i < 128 ; i++ )
  {
    setLed(SS1 ,REG0, i);
    setLed(SS1 ,REG0, 127-i);
    setLed(SS1 ,REG1, i);
    setLed(SS1 ,REG1, 127-i);
  }
  
  delay(400);

  for (int i = 0 ; i < 128 ; i++ )
  {
    setLed(SS1 ,REG0, 127-i);
    setLed(SS1 ,REG0, i);
    setLed(SS1 ,REG1, 127-i);
    setLed(SS1 ,REG1, i);
  }

  delay(400);    
  */
  
  }
