// theramin_build3.ino by C.Spindler 1-26-2020
// based on theramin_build2.ino in examples

// tested on circuit shown in drawing THERAMIN , dwgno CFS-1-24-2020-A,
// omitting the "out to house amplifer" circuit, of course.
// works well except as noted below on 1-26-2020 at 11:00 pm


// for refinements and problem fixes:
//      1. tone are coming in notes about one per second, too slow
//      2. pitch range control could be better
//      3. use preprocessor directives to selectively compile,
//         or, not compile, all printing and serial code. (done on
//         1-26-2020, no discernable increase in speed of notes)
//      4. Use unsigned types for integer arithmetic, to assure that
//         results of arithmetic operations are at least well defined
//         and testable. Comment caution in range of results. (done 
//         on 1-26-2020).

// based on theramin.ino in ...ElectricGarden\Arduino\,
// from Single_Buzzer.ino, operational on 1-5-2020
// and on vl53l0x_dual_test.ino by C.Spindler 1-25-2020
// which worked well on TOF sensors on 1-25-2020 at 1:00 pm
// and on spi_demo_nano.ino, verified on a Nano on 1-19-2020.

   #define ENABLE_SERIAL

// from Single_Buzzer.ino 
// don't #include <Wire.h> , because it's clearly in Adafruit_VL53L0X.h
#define SPEAKER  9      // use pin D9 on Nano
#define MAXFREQUENCY  1200


// from spi_demo_nano.ino
#include <SPI.h>  // comment this out if your radio library establishs SPI
#define SS1  7 //Pin 7 is SS (slave select) for device 1
#define REG0 B00000000 //Register 0 Write command for Px0 pins
#define REG1 B00010000 //Register 1 Write command for Px1 pins


// from vl53l0x_dual_test.ino
#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 4  //#define SHT_LOX1 7
#define SHT_LOX2 3  //#define SHT_LOX1 6

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
// in Adafruit_VL53L0X LIBRARY, IN vl53l0x_def.h, there is the def:
// typedef struct {...} VL53L0X_RangingMeasurementData_t; there is a data
// member: "uint16_t RangeMilliMeter; /*!< range distance in millimeter. */"
// so measure1.RangeMilliMeter is type uint16_t, the distance measured in mm 


/* 
    in setID(){...}
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    #ifdef ENABLE_SERIAL 
    Serial.println(F("Failed to boot first VL53L0X"));
    #endif
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    #ifdef ENABLE_SERIAL
    Serial.println(F("Failed to boot second VL53L0X"));
    #endif
    while(1);
  }
}

void read_dual_sensors() {

// try with debug = true below, sensor #1 fails to start ?? !!!!!
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  #ifdef ENABLE_SERIAL 
  Serial.print("1: ");
  #endif
  if(measure1.RangeStatus != 4) {     // if not out of range
    #ifdef ENABLE_SERIAL 
    Serial.print(measure1.RangeMilliMeter);
    #endif
    //  //////////////////////////////////
    pitch(measure1.RangeMilliMeter);
    //  //////////////////////////////////
  } else {
    #ifdef ENABLE_SERIAL 
    Serial.print("Out of range");
    #endif
  }
  #ifdef ENABLE_SERIAL 
  Serial.print(" ");
  #endif

  // print sensor two reading
  #ifdef ENABLE_SERIAL  
  Serial.print("2: ");
  #endif
  
  if(measure2.RangeStatus != 4) {
    #ifdef ENABLE_SERIAL 
    Serial.print(measure2.RangeMilliMeter);
    #endif
    //  //////////////////////////////////
    volume(measure2.RangeMilliMeter);
    //  //////////////////////////////////
  } else {
    #ifdef ENABLE_SERIAL  
    Serial.print("Out of range");
    #endif
  }
  #ifdef ENABLE_SERIAL  
  Serial.println();
  #endif
}

void pitch(uint16_t dist){
   if(dist <= MAXFREQUENCY ){
        tone(SPEAKER, dist );
        } else {
              noTone(SPEAKER); // comment out noTone() to sustain last tone
        }
}

void commandPot(int SS, int reg, int level) {
  // from spi_demo_nano.ino
  
 // SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
 // only use SPI.beginTransaction(...) & SPI.endTransaction(...) in 
 // coordination with your radio library, which uses SPI internally
   
  digitalWrite(SS, LOW); // set the named pin LOW 
                         // to select that chip
  SPI.transfer(reg  ); // send the first byte to pick the register 
                       // number of the potentionmeter of the selected chip
  SPI.transfer(level); // send the 2nd byte to set the level of that 
                       // potentiometer
  digitalWrite(SS,HIGH); // de-select that chip             

 //  SPI.endTransaction();
                                             
}

void volume(uint16_t dist){

// map distances from TOF sensor (30 mm to 1000 mm) to the 129 steps,
// (0 to 128) of the MCP4231 103E digital potentiometer

#define BOTTOMRANGE 100
#define TOPRANGE 500
#define LOSTEP 0
#define HISTEP 128

uint16_t rangeStep = (TOPRANGE-BOTTOMRANGE)/(HISTEP - LOSTEP);
uint16_t volumeLevel = LOSTEP;
if (dist >= TOPRANGE ){
    volumeLevel = HISTEP;
    } else { 
        if (dist > (BOTTOMRANGE + rangeStep )){
          // Careful, intermediate product may be too big for unit16_t and 
          // would be automatically promoted by the compiler to uint32_t. 
          // Also, assure that for all values of dist that can reach this
          // statement, the result in volumeLevel are in the range from
          // ( LOSTEP+1 ) to ( HISTEP-1 ), and that no intermediate arithemetic 
          // results are less than zero or higher than uint32_t can hold.
          volumeLevel = (uint16_t)((dist-BOTTOMRANGE)*HISTEP/(TOPRANGE-BOTTOMRANGE));
        }
    }        

    #ifdef ENABLE_SERIAL  
    Serial.print(" volumeLevel = ");
    Serial.print(volumeLevel);
    #endif
    commandPot(SS1 ,REG0, volumeLevel); // sets wiper on pot pins P0W
    commandPot(SS1 ,REG1, volumeLevel); // sets wiper on pot pins P1W
  
}


void setup() {

  // from spi_demo_nano.ino

  // setup slave select pins for output
  pinMode(SS1, OUTPUT);

  // per instructions from arduino.cc/en/reference/SPI 
  // set the Nano's SS pin, 11, to OUTPUT to make it 
  // unuasable as a chip select for whole Nano board as a SPI slave
  // also, we use pin D11 for MOSI, which is compatable with OUTPUT
  pinMode(11,OUTPUT);

//  initialize SPI 
    SPI.begin(); // comment this out if your radio library initializes SPI

  #ifdef ENABLE_SERIAL 
  Serial.begin(115200);
  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }
  #endif
  
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  #ifdef ENABLE_SERIAL
  Serial.println("Shutdown pins inited...");
  #endif

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  #ifdef ENABLE_SERIAL
  Serial.println("Both in reset mode...(pins are low)");
  Serial.println("Starting...");
  #endif
  
  setID();
 
}

void loop() {
   
  read_dual_sensors();
  delay(100);
}
