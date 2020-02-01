// theramin_build4.ino by C.Spindler 1-28-2020
// based on theramin_build3.ino by C.Spindler 1-26-2020
// which was based on theramin_build2.ino in examples

// the point of ...build4 is to change over to the Pololu library
// for controlling the VL53L0x devices, which actually made effectively
// continuous tones in sketch Single_Buzzer.ino using a circuit shown 
// on drawing SINGLE_BUZZER wiring diagram, dwgno. CFS-1-5-2020-B1

#define ENABLE_SERIAL // comment out to drop compilation of serial print code
#define ENABLE_STOPWATCH // comment out unless you want to to measure elapsed time between loops

// from Single_Buzzer.ino 
#include <Wire.h> // not in ...build3 because it's clearly in Adafruit_VL53L0X.h
#include <VL53L0X.h> // this is the header for the Pololu library for vl53l0x
#define SPEAKER  9      // use pin D9 on Nano
#define MAXFREQUENCY  1200

// from Single_Buzzer.ino
// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

// #define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

#define HIGH_SPEED  // if you choose HIGH_ACCURACY you can hear the steps in pitch
// #define HIGH_ACCURACY  


// from spi_demo_nano.ino and is used to enable the digital potentiomenter
#include <SPI.h>  // comment this out if your radio library establishs SPI
#define SS1  7 //Pin 7 is SS (slave select) for device 1
#define REG0 B00000000 //Register 0 Write command for Px0 pins
#define REG1 B00010000 //Register 1 Write command for Px1 pins


// address we will assign if dual sensor is present
#define SENSOR1_ADDRESS 0x33
#define SENSOR2_ADDRESS 0x37

// set the pins to shutdown
#define SHT_SENSOR1 4  // D4 is the Nano pin conected to XSHUT for sensor1
#define SHT_SENSOR2 3  // D3 is the Nano pin conected to XSHUT for sensor2



 VL53L0X sensor1; // constructor for Pololu's sensor control objects
 VL53L0X sensor2; 

// this holds the measurement
// VL53L0X_RangingMeasurementData_t measure1;
// VL53L0X_RangingMeasurementData_t measure2;
// in Adafruit_VL53L0X LIBRARY, IN vl53l0x_def.h, there is the def:
// typedef struct {...} VL53L0X_RangingMeasurementData_t; there is a data
// member: "uint16_t RangeMilliMeter; /*!< range distance in millimeter. */"
// so measure1.RangeMilliMeter is type uint16_t, the distance measured in mm 


/* 
    in setID(){...} , 
	(  in the new arrangement using the Pololu code library, VL53L0X.cpp:
			initialization is by using:   		bool VL53L0X::init(bool io_2v8)
			i2c address assignment is by: 		void VL53L0X::setAddress(uint8_t new_address)  )
	  1. initialize both sensor1 and sensor2  objects separately.
       (it happens in setup{...}  )		
    2. Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    3. Keep sensor #1 awake by keeping XSHUT pin high
    4. Put all other sensors into shutdown by pulling XSHUT pins low
    5. Set a new address for sensor #1 using sensor1.setAddress(...), 
	     Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
	  6. Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
	  7. Set a new address for sensor #2 using sensor2.setAddress(...),
       Pick any number but 0x29 and whatever you set the first sensor to.
 */
void setIdAndInit() {
  // initialize all sensors in setup {...}
    // all reset
    
  digitalWrite(SHT_SENSOR1, LOW);    
  digitalWrite(SHT_SENSOR2, LOW);
  delay(10);
  
  // all unreset
  digitalWrite(SHT_SENSOR1, HIGH);
  digitalWrite(SHT_SENSOR2, HIGH);
  delay(10);

  // activating SENSOR1 and reseting SENSOR2
  digitalWrite(SHT_SENSOR1, HIGH);
  digitalWrite(SHT_SENSOR2, LOW);

  sensor1.setTimeout(500); 
  if (!sensor1.init())
  {
    Serial.println("Failed to detect and initialize first sensor!");
    while (1) {}
  }
  
  // setting address for SENSOR1
  sensor1.setAddress(SENSOR1_ADDRESS);
  delay(10);

  //  activate SENSOR2
  digitalWrite(SHT_SENSOR2, HIGH);
  delay(10);
  
  sensor2.setTimeout(500); 
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize second sensor!");
    while (1) {}
  }
  // setting address for SENSOR2
  sensor2.setAddress(SENSOR2_ADDRESS);
  delay(10);
 
}

void read_dual_sensors() {  

uint16_t dist1 = sensor1.readRangeSingleMillimeters();
bool dist1Valid = !(sensor1.timeoutOccurred());
  // print sensor one reading
  #ifdef ENABLE_SERIAL 
  Serial.print("1: ");
  if(dist1Valid){
      Serial.print(dist1);
  } else {
      Serial.print(" TIMEOUT");
  }
 
 #endif
    //  //////////////////////////////////
  if(dist1Valid){  
    pitch(dist1);
  }
    //  //////////////////////////////////

    
uint16_t dist2 = sensor2.readRangeSingleMillimeters();
bool dist2Valid = !(sensor2.timeoutOccurred());
   // print sensor two reading
  #ifdef ENABLE_SERIAL 
   Serial.print("  2: ");
  if(dist2Valid){
      Serial.print(dist2);
  } else {
      Serial.print(" TIMEOUT");
  }
  Serial.print("  ");
 #endif
    //  //////////////////////////////////
   if(dist2Valid) {
    volume(dist2);
   }
    //  //////////////////////////////////

  #ifdef ENABLE_SERIAL
   Serial.println();
 #endif
}

void pitch(uint16_t dist){
  
   uint16_t frequency = dist; //need not be identically equal
   
   if(dist <= MAXFREQUENCY ){
        tone(SPEAKER, frequency );
          #ifdef ENABLE_SERIAL
          Serial.print("  frequency = ");
          Serial.print(frequency);
          #endif
        } else {
            //  noTone(SPEAKER); // comment out noTone() to sustain last tone
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


pinMode(SHT_SENSOR1,OUTPUT);
pinMode(SHT_SENSOR2,OUTPUT);


  #ifdef ENABLE_SERIAL 
  Serial.begin(115200);
  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }
  #endif

  #ifdef ENABLE_STOPWATCH
     #ifndef ENABLE_SERIAL 
     Serial.begin(115200);
     // wait until serial port opens for native USB devices
     while (! Serial) { delay(1); }
     Serial.println("STOPWATCH enabled");
     #endif
  #endif

  Wire.begin();

  setIdAndInit();


  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor1.setSignalRateLimit(0.1);
  sensor2.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
sensor1.setMeasurementTimingBudget(20000);
 sensor2.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor1.setMeasurementTimingBudget(200000);
  sensor2.setMeasurementTimingBudget(200000);
#endif

 
}

void loop() {

  #ifdef ENABLE_STOPWATCH
  unsigned long time1 = millis();
  unsigned int k ;
  for( k = 0; k < 100; k++) {
  #endif
  
  read_dual_sensors();
//  delay(100);

  #ifdef ENABLE_STOPWATCH
  }
  unsigned long time2 = millis();
  Serial.print(" elapsed time for ");
  Serial.print( k );
  Serial.print(" loops is ");
  Serial.print( (time2-time1));
  Serial.println(" ms");
  
  #endif
}
