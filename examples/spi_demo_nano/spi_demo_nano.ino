 //  spi_demo_nano.ino  by C.Spindler     1-18-2020 1:24 pm
 //  based on an operational sketch spi-demo.ino for MEGA 2560
 //  operation verified on 1-19-2020 
 
 //  based on exercises in book "Exploring Arduino" 
 //  by Jeremy Blum, Chapter 9 SPI Buss. 
 //  similar to Listing 9-1 on page 192

 // this sketch will use only one
 // digital pot and is being altered to fit Adruino Nano 
 // shown on drawing SPI-DEMO, Dwg. No. CFS-1-13-2020-A

 // take great care with current thru digital POT pins
 // max currrent is 2.5 ma. 
 // max voltage compared to Vss (Gnd) is Vdd + 0.3v so about 5.3 v

  
 /*
   We're using Arduino's SPI Library, see arduino.cc/en/reference/SPI
   for Arduino NANO boards, by default,
   use Pin 13 (D13) = SCK , 12 (D12) = MISO and 11 (D11) = MOSI
   also, recommends setting pin 11 (D11) to OUTPUT to avoid accedentially
   borking the board if pin 11, the default SS pin for the Nano
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
         chip, sending 8 bits (or 16 bits) and then deselecting chip. Read and 
         write commands are only 16 bit, while increment and decrement commands
		     are only 8 bit.  We only use the 16 bit Write command. 
        
*/   

#include <SPI.h>

const int SS1 = 7; //Pin 7 is SS (slave select) for device 1

const byte REG0 = B00000000;//Register 0 Write command for Px0 pins
const byte REG1 = B00010000;//Register 1 Write command for Px1 pins

// * see the SPI command formats on Blum, page 188, Figure 9-4
// * also shown on MCP4231 datasheet, fig. 7-1 and table 7-2
// * 
// * Write commands are 16 bits long and are sent in two 
// * consecutive 8 bit SPI.transfer() instructions.
 
// * The first 8 bits are called the Command Byte, choose from
// * REG0 or REG1 above. 
 
// * The second 8 bits are called the Data Byte, and while it is
// * possible to use more bit, MCP4231 has only 129 steps in resistance
// * so the Data Byte should range from 0 to 128 (00h to 80h)(b00000000 to b10000000)
 




void setup() {
  
  // setup slave select pins for output
  pinMode(SS1, OUTPUT);

  // per instructions from arduino.cc/en/reference/SPI 
  // set the Nano's SS pin, 11, to OUTPUT to make it inoperative
  pinMode(11,OUTPUT);

//  initialize SPI 
    SPI.begin();
 
}

void commandPot(int SS, int reg, int level) {

  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  
  digitalWrite(SS, LOW); // set the named pin LOW 
                         // to select that chip
  SPI.transfer(reg  ); // send the first byte to pick the register 
                       // number of the potentionmeter of the selected chip
  SPI.transfer(level); // send the 2nd byte to set the level of that 
                       // potentiometer
  digitalWrite(SS,HIGH); // de-select that chip             

  SPI.endTransaction();
                                             
}


void loop() {

  commandPot(SS1 ,REG0, 0);
  commandPot(SS1 ,REG1, 0);

  delay(1000);
  commandPot(SS1 ,REG0, 80);
  commandPot(SS1 ,REG1, 80);

  delay(1000);
  commandPot(SS1 ,REG0, 128);
  commandPot(SS1 ,REG1, 128);

  delay(1000);

  
  }
