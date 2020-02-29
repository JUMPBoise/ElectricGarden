// theramin_build6.ino by C.Spindler 2-9-2020
// ...build6 will:
//      1. read sensors only when they test ready, like ...build5, ie
//         when (content of RESULT_INTERRUPT_STATUS register & 0x07)!= 0, reading the 
//         assosciated sensor object will not require waiting.  All using public resources
//         of the Pololu library VL53L0X. 
//             a. use a reasonable range for ergonomic use considering new flower physical 
//                instruement, say 100 to 600 mm same on all sensors, but individually programable.
//             b. use Arduino map (...) function.
//             c. map real ergonomic ranges to range uint16_t (0 to 1023), out of void 
//                read_three_sensors(...)function.
//             d. when an individual sensor is beyond ergonomic range, but there is no deadstick 
//                timeout, always continue last dist reading in ergonomic range. 
//      2. improve ergonomic operation of sensors on pitch, volume and other
//          a. if all sensor unused (ie over a max DEAD_RANGE) for DEADSTICK_TIMEOUT_PERIOD ms,
//             all sound ceases. MODIFY SO THAT SEVERAL CONSECUTIVE READINGS LESS THAN DEADRANGE ARE 
//             NEEDED TO LEAVE DEADSTICK_TIMEOUT, to avoid leaving for a single spurlious reading.
//          b. Control deadstick_timeout with a global variable: bool deadstickTimeout. Set and re-set it 
//              in void read_three_sensors(...)function.
//          b. if volume sensor or pitch sensor goes out of range, volume level or pitch continues unchanged,
//             unless stopped for deadstick 
//          c. when theramin comes out of deadstick, always go to a default pitch and volume, choosen so that 
//             when a user interupts ANY sensor, the user immediately hears sound.
//          d. range of pitch will be increased.
//      3. improve serial output to keep debugging but avoid burdening loop(...) with printing in use. 
//
// based on theramin_build5.ino by C.Spindler 2-7-2020
// which tested at 2 ms per loop{...} at 11:30 pm on 2-7-2020 



// #include <LibPrintf.h> // always compile serial and printf

//#define ENABLE_RADIO
//#define ENABLE_DEBUG_PRINT
#define ENABLE_D // comment out to drop tagged printing 
#define ENABLE_STOPWATCH // comment out unless you want to to measure elapsed time between loops

#ifdef ENABLE_STOPWATCH
  static unsigned long time1;
  static unsigned long k ; // by default, on creation, static is initalized to zero
#endif

// from Single_Buzzer.ino 
#include <Wire.h> // not in ...build3 because it's clearly in Adafruit_VL53L0X.h
#include <VL53L0X.h> // this is the header for the Pololu library for vl53l0x
#include <RF24.h>

#ifdef ENABLE_DEBUG_PRINT  
#include "printf.h"
#endif

#ifdef ENABLE_D 
#ifndef ENABLE_DEBUG_PRINT
#include "printf.h"
#endif
#endif

#define SPEAKER  6      // use pin D6 on Nano

// from spi_demo_nano.ino and is used to enable the digital potentiomenter
#include <SPI.h>  // comment this out if your radio library establishes SPI
#define SS1  7 //Pin 7 is SS (slave select) for device 1
#define REG0 B00000000 //Register 0 Write command for Px0 pins
#define REG1 B00010000 //Register 1 Write command for Px1 pins

// address we will assign if dual sensor is present
#define SENSOR1_ADDRESS 0x33
#define SENSOR2_ADDRESS 0x35
#define SENSOR3_ADDRESS 0x37

// set the pins to shutdown
#define SHT_SENSOR1 2  // D2 is the Nano pin conected to XSHUT for sensor1
#define SHT_SENSOR2 3  // D3 is the Nano pin conected to XSHUT for sensor2
#define SHT_SENSOR3 4  // D4 is the Nano pin conected to XSHUT for sensor3

static constexpr uint8_t widgetId = 30;
static constexpr uint32_t activeTxIntervalMs = 50L;
static constexpr uint32_t inactiveTxIntervalMs = 500L;  // should be a multiple of activeTxIntervalMs


// ---------- radio configuration ----------

// Nwdgt, where N indicates the payload type (0: stress test; 1: position
// and velocity; 2: measurement vector; 3,4: undefined; 5: custom)
#define TX_PIPE_ADDRESS "2wdgt"

// Set WANT_ACK to false, TX_RETRY_DELAY_MULTIPLIER to 0, and TX_MAX_RETRIES
// to 0 for fire-and-forget.  To enable retries and delivery failure detection,
// set WANT_ACK to true.  The delay between retries is 250 us multiplied by
// TX_RETRY_DELAY_MULTIPLIER.  To help prevent repeated collisions, use 1, a
// prime number (2, 3, 5, 7, 11, 13), or 15 (the maximum) for TX_MAX_RETRIES.
#define WANT_ACK false
#define TX_RETRY_DELAY_MULTIPLIER 0
#define TX_MAX_RETRIES 0

// Probably no need to ever set auto acknowledgement to false because the sender
// can control whether or not acks are sent by using the NO_ACK bit.
#define ACK_WIDGET_PACKETS true

// Possible data rates are RF24_250KBPS, RF24_1MBPS, or RF24_2MBPS.  (2 Mbps
// works with genuine Nordic Semiconductor chips only, not the counterfeits.)
#define DATA_RATE RF24_1MBPS

// Valid CRC length values are RF24_CRC_8, RF24_CRC_16, and RF24_CRC_DISABLED
#define CRC_LENGTH RF24_CRC_16

// nRF24 frequency range:  2400 to 2525 MHz (channels 0 to 125)
// ISM: 2400-2500;  ham: 2390-2450
// WiFi ch. centers: 1:2412, 2:2417, 3:2422, 4:2427, 5:2432, 6:2437, 7:2442,
//                   8:2447, 9:2452, 10:2457, 11:2462, 12:2467, 13:2472, 14:2484

// temporarily, we set RF_CHANNEL to 76, to keep using the radio hardware, but not interfere 
// with testing of light patterns, put it back to channel 80 later
#define RF_CHANNEL 76   // Electric Garden Theremin is on ch. 80, Illumicone is on ch. 97

// RF24_PA_MIN = -18 dBm, RF24_PA_LOW = -12 dBm, RF24_PA_HIGH = -6 dBm, RF24_PA_MAX = 0 dBm
#define RF_POWER_LEVEL RF24_PA_MIN


/**********************************************************
 * Widget Packet Header and Payload Structure Definitions *
 **********************************************************/

union WidgetHeader {
  struct {
    uint8_t id       : 5;
    uint8_t channel  : 2;
    bool    isActive : 1;
  };
  uint8_t raw;
};

// pipe 0
struct StressTestPayload {
  WidgetHeader widgetHeader;
  uint32_t     payloadNum;
  uint32_t     numTxFailures;
};

// pipe 1
struct PositionVelocityPayload {
  WidgetHeader widgetHeader;
  int16_t      position;
  int16_t      velocity;
};

// pipe 2
struct MeasurementVectorPayload {
  WidgetHeader widgetHeader;
  int16_t      measurements[15];
};

// pipe 5
struct CustomPayload {
  WidgetHeader widgetHeader;
  uint8_t      buf[31];
};


// ************************************ GLOBAL VARIABLES **************************

RF24 radio(9, 10);    // CE on pin 9, CSN on pin 10, also uses SPI bus (SCK on 13, MISO on 12, MOSI on 11)

static MeasurementVectorPayload payload;

static bool isActive;
static bool wasActive;

// TODO:  Need to select the pattern somehow.
static uint8_t patternNum = 1;


constexpr uint16_t DEAD_RANGE = 1200; // 1200 mm. 
constexpr uint16_t DEADSTICK_TIMEOUT_PERIOD = 5000; // 5000 ms. 

constexpr uint16_t  LoErgoRange = 100; // the ergonomic range where sensors are used to play music
constexpr uint16_t  HiErgoRange = 600; // is fixed here to be 100 mm to 600 mm
constexpr uint16_t  LoNormalRange = 0;    // the ergonomic range will be mapped into a normalized
constexpr uint16_t  HiNormalRange = 1023; // range, from 0 to 1023

#define DEFAULTDIST1 400
#define DEFAULTDIST2 400
#define DEFAULTDIST3 400

uint16_t dist1 = DEFAULTDIST1 , dist2 = DEFAULTDIST2 , dist3 = DEFAULTDIST3; // distN is global
bool deadstickTimeout = false;
bool sensor3InRange = false;
uint8_t reallyBack = 0;
bool noToneFlag = true;

VL53L0X sensor1; // constructor for Pololu's sensor control objects
VL53L0X sensor2; 
VL53L0X sensor3; 

/* 
    in  void setIdAndInit() 
	(  in the new arrangement using the Pololu code library, VL53L0X.cpp:
			initialization is by using:   		bool VL53L0X::init(bool io_2v8)
			i2c address assignment is by: 		void VL53L0X::setAddress(uint8_t new_address)  )
	  1. Reset all sensors by setting all of their XSHUT pins low ,
    2. then, one sensor at at time:
        a. after 10 ms, activate the sensor by making its XSHUT pin high, and leaving it high for rest of sketch
        b. after 10 ms, initiate sensor using bool VL53L0X::init(bool io_2v8)
        c. after 10 ms, assign sensor its new address using void VL53L0X::setAddress(uint8_t new_address) 
    3. Pick any number but 0x29 and whatever addres the other sensors areassigned.  Addresses must be under 0x7F. 
       Going with 0x30 to 0x3F is probably OK.
    4. 10 ms after last address is assigned, start continuous reading of sensors, one sensor object at a time,
       using void VL53L0X::startContinuous();
	  
 */


void setIdAndInit() {
 
    // all reset
    
  digitalWrite(SHT_SENSOR1, LOW);    
  digitalWrite(SHT_SENSOR2, LOW);
  digitalWrite(SHT_SENSOR3, LOW);
  delay(10);
  
  // activating SENSOR1 
  digitalWrite(SHT_SENSOR1, HIGH);
  delay(10);

  //init SENSOR1
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
  
  // init SENSOR2
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize second sensor!");
    while (1) {}
  }
  // setting address for SENSOR2
  sensor2.setAddress(SENSOR2_ADDRESS);
  delay(10);

  //  activate SENSOR3
  digitalWrite(SHT_SENSOR3, HIGH);
  delay(10);
  
  // init SENSOR3
  if (!sensor3.init())
  {
    Serial.println("Failed to detect and initialize third sensor!");
    while (1) {}
  }
  // setting address for SENSOR3
  sensor3.setAddress(SENSOR3_ADDRESS);
  delay(10);

  sensor1.startContinuous(); // necessary to use CONTINUOUS mode
  sensor2.startContinuous();
  sensor3.startContinuous();
 
}

void read_three_sensors() {  

// only output is dist1,dist2 & dist3,  normalized to 0 to 1023 range,
// and bool   
  
// the VL53L0X sensor is nominally 30 to 2000 mm, but practically 30 mm to about
// 1300 mm is all that is reliable. We will call anything measuring over DEAD_RANGE (or 1200 mm)
// out of range, and infer that no one is using that sensor to play the theramin.

// global variables dist1, dist2 and dist 3 will be set the distances from each sensor,  
// normalized by mapping from the ergomomic range to the range from 0 to 1023. We'll read them 
// with local variables measure1, measure2 and measure3. 

static uint16_t measure1,measure2,measure3; // static initializes to zero
static uint32_t timeLastUsed;

#ifdef ENABLE_D  
printf("read_3_sensorsA measure1: %6d measure2: %6d measure3: %6d\n", measure1,measure2,measure3); 
#endif

// the condition below for reading sensorN is the same test that is used by
// uint16_t VL53L0X::readRangeContinuousMillimeters(void) to determine whether to
// actually get the sensor measurement or wait.
// we only read the sensor if we KNOW we will not wait
// if sensor isn't ready, measureX just stays same, no change, it's STATIC

  if((sensor1.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
        measure1 = sensor1.readRangeContinuousMillimeters();
        }

  if((sensor2.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
        measure2 = sensor2.readRangeContinuousMillimeters();
        }

  if((sensor3.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
        measure3 = sensor3.readRangeContinuousMillimeters();
        }      
 
               
#ifdef ENABLE_D 
printf("read_3_sensorsB measure1: %6d measure2: %6d measure3: %6d\n", measure1,measure2,measure3); 
#endif


  uint32_t now = millis();
  if ( measure1 < DEAD_RANGE || measure2 < DEAD_RANGE || (sensor3InRange = (measure3 < DEAD_RANGE ))) {
      timeLastUsed = now;
      } 

#ifdef ENABLE_D 
printf("read_3_sensorsC              timeLastUsed: %6d now: %6d delta: %6d \n", timeLastUsed, now, now - timeLastUsed); 
#endif

      
  if( (now - timeLastUsed)  > DEADSTICK_TIMEOUT_PERIOD ) {  
      // DECLARE DEADSTICK_TIMEOUT
      deadstickTimeout = true;
      reallyBack = 0;
      dist1 = DEFAULTDIST1;
      dist2 = DEFAULTDIST2;
      dist3 = DEFAULTDIST3; 
      } else { 
        if (reallyBack >= 10 ) {
          deadstickTimeout = false;

#ifdef ENABLE_D 
printf("read_3_sensorsD dist1: %6d dist2: %6d dist3: %6d\n", dist1,dist2,dist3); 
#endif
            
          if (measure1 > LoErgoRange && measure1 < HiErgoRange ) {
             // If measureN is out of ergonomic range, distN remains unchanged            
             dist1 = map(measure1, LoErgoRange, HiErgoRange, LoNormalRange, HiNormalRange);
            }
         
         if (measure2 > LoErgoRange && measure2 < HiErgoRange ) {
            // If measureN is out of ergonomic range, distN remains unchanged            
             dist2 = map(measure2, LoErgoRange, HiErgoRange, LoNormalRange, HiNormalRange);
             }
         
          if (measure3 > LoErgoRange && measure3 < HiErgoRange ) {
             // If measureN is out of ergonomic range, distN remains unchanged            
             dist3 = map(measure3, LoErgoRange, HiErgoRange, LoNormalRange, HiNormalRange);
            }
#ifdef ENABLE_D 
printf("read_3_sensorsD dist1: %6d dist2: %6d dist3: %6d \n", dist1,dist2,dist3);   
#endif
           }  else {
           reallyBack++;
        }
     }
    
}

uint16_t getPitch(uint16_t dist){

// dist is normalized from 0 to 1023
// and it is always maintained at last dist if measure at sensor is out of range  

static uint16_t lastDist; //static initializes to 0
static uint16_t frequency;

   const uint16_t LoPitch = 30;    //  lowest frequency toned is 30 hz
   const uint16_t HiPitch = 6000;  //  highest frequency toned is 6000 hz   

#ifdef ENABLE_D 
printf("\nline 371 lastDist: %6d dist: %6d frequency: %6d\n",lastDist, dist, frequency); 
#endif
   if(lastDist != dist ) {
       frequency = map(dist,LoNormalRange,HiNormalRange,LoPitch,HiPitch);
       lastDist = dist;
       }
#ifdef ENABLE_D 
printf("line 376 lastDist: %6d dist: %6d frequency: %6d\n",lastDist, dist, frequency); 
#endif      
   return frequency;     
  }

//************************************BENDPITCH*************************************
uint16_t bendPitch(uint16_t pitch , uint16_t dist){

//  we will map dist, which is normalized from 0 to 1023, into a reversed
//  higher pitch range.

// if the user moves hand away from sensor3 , we want bend to stop,
// so I have created a bool sensor3OutOfRange which is true if 
// sensor3 is out of range. 

uint16_t T = 300 ; // T is period of the sharp rising chirp, in ms, init to 1000
// const uint16_t shortT = 200;
// const uint16_t longT = 400;

#ifdef  ENABLE_D
printf("bendPitchA    pitch: %6d dist: %6d \n", pitch, dist);
#endif

if(sensor3InRange) {
//        T= map(dist, HiNormalRange, LoNormalRange, shortT, longT);
        T=300;
        pitch = pitch + millis() % T ;
        }

#ifdef  ENABLE_D
printf("bendPitchB    pitch: %6d dist: %6d \n", pitch, dist);
#endif


     
   return pitch;  
          

           
  }

//************************************BENDPITCH*************************************



void setPitch(uint16_t freq) {
    static uint16_t lastFreq; // static initalizes to 0

    if (freq != lastFreq || noToneFlag) {      
       tone(SPEAKER, freq );// tone(pin,freq) sustains last freq unless noTone() is called
       lastFreq = freq;
       noToneFlag = false;
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

uint8_t getVolumeLevel(uint16_t dist){
// map normalized dist which ranges from 0 to 1023, into the 129 steps,
// (0 to 128) of the MCP4231 103E digital potentiometer


const uint16_t LoStep = 0;    
const uint16_t HiStep = 128;   
static uint16_t lastDist; //static initializes to 0
static uint8_t volumeLevel; //static initializes to 0


// so if dist = zero on first call, volumeLevel is defined as zero
// after that, if dist != lastDist, volumeLevel is calculate, or 
// if dist = lastDist, volumeLevel is unchanged.

  if(lastDist != dist ) {
       volumeLevel = map(dist,LoNormalRange,HiNormalRange,LoStep,HiStep);
       lastDist = dist;
       }
       
   return volumeLevel;     
  }



void setVolumeLevel(uint8_t volumeLevel) {
      commandPot(SS1 ,REG0, volumeLevel); // sets wiper on pot pins P0W
      commandPot(SS1 ,REG1, volumeLevel); // sets wiper on pot pins P1W
}


void broadcastMeasurements()
{
  constexpr uint8_t numDistanceMeasmts = 3;

  payload.measurements[0] = patternNum;
  payload.measurements[1] = dist1 >= 0 && dist1 <= 1023 ? dist1 : 1023;
  payload.measurements[2] = dist2 >= 0 && dist2 <= 1023 ? dist2 : 1023;
  payload.measurements[3] = dist3 >= 0 && dist3 <= 1023 ? dist3 : 1023;

  payload.widgetHeader.isActive = isActive;

  if (!radio.write(&payload, sizeof(WidgetHeader) + sizeof(int16_t) * (numDistanceMeasmts + 1), !WANT_ACK)) {
#ifdef ENABLE_DEBUG_PRINT
    Serial.println(F("radio.write f7ailed.return volumeL"));
#endif
  }
  else {
#ifdef ENABLE_DEBUG_PRINT
    Serial.println(F("radio.write succeeded."));
#endif
  }
}


void configureRadio(
  RF24&             radio,
  const char*       writePipeAddress,
  bool              wantAcks,
  uint8_t           txRetryDelayMultiplier,
  uint8_t           txMaxRetries,
  rf24_crclength_e  crcLength,
  rf24_pa_dbm_e     rfPowerLevel,
  rf24_datarate_e   dataRate,
  uint8_t           channel)
{
  radio.begin();

  radio.setPALevel(rfPowerLevel);
  radio.setRetries(txRetryDelayMultiplier, txMaxRetries);
  radio.setDataRate(dataRate);
  radio.setChannel(channel);
  radio.enableDynamicAck();         // allow sending payloads with or without ack request
  radio.enableDynamicPayloads();
  radio.setCRCLength(crcLength);

  radio.openWritingPipe((const uint8_t*) writePipeAddress);

#ifdef ENABLE_DEBUG_PRINT
  radio.printDetails();
#endif

  // Widgets only transmit data.
  radio.stopListening();
}


void setup() {

#ifdef ENABLE_STOPWATCH
    time1 = millis();
#endif


  Serial.begin(115200);
#ifdef ENABLE_DEBUG_PRINT
  printf_begin();
#endif

#ifdef ENABLE_D 
#ifndef ENABLE_DEBUG_PRINT
  printf_begin();
#endif
#endif

  delay(500);            
  // consider using: while (! Serial) { delay(1); }
  // but you don't want sketch to fail if no serial interface!

  // from spi_demo_nano.ino

  pinMode(SHT_SENSOR1,OUTPUT);
  pinMode(SHT_SENSOR2,OUTPUT);
  pinMode(SHT_SENSOR3,OUTPUT);

  // setup slave select pins for output
  pinMode(SS1, OUTPUT);
  // Deassert chip select for now so that we can talk to the radio module.
  digitalWrite(SS1, HIGH);

//  initialize SPI 
//  SPI.begin(); // comment this out if your radio library initializes SPI

  // It appears that RF24 will set up the SPI interface for us.
  configureRadio(radio, TX_PIPE_ADDRESS, WANT_ACK, TX_RETRY_DELAY_MULTIPLIER,
                 TX_MAX_RETRIES, CRC_LENGTH, RF_POWER_LEVEL, DATA_RATE,
                 RF_CHANNEL);

  Wire.begin();

  setIdAndInit();

  payload.widgetHeader.id = widgetId;
  payload.widgetHeader.channel = 0;
}


void loop()
{ 
  static int32_t lastTxMs;

  uint8_t Volume;
  uint16_t Pitch;

  uint32_t now = millis();

  read_three_sensors();

if( deadstickTimeout ) { 
    noTone(SPEAKER);
    noToneFlag = true;
     
#ifdef ENABLE_D 
printf("loop()A                               DEADSTICK_TIMEOUT \n"); 
#endif     
  
    } else { 


           
       Pitch = getPitch(dist1);
       Pitch = bendPitch(Pitch, dist3);  //should have dist = dist3
       Volume = getVolumeLevel(dist2);

       setPitch(Pitch);
       setVolumeLevel(Volume);
               
#ifdef ENABLE_D 
printf("loop()B       Pitch = %6d  Volume = %6d\n", Pitch,Volume); 
#endif               

     
           
  }



#ifdef ENABLE_STOPWATCH
  // #ifdef ENABLE_STOPWATCH, then time1 was initiaized to millis() in setup(), 
  // and k, a static variable,  is initalized to zero on creation
  k++;
  if (k >= 100) {
    unsigned long time2 = millis();
    Serial.print(" elapsed time for ");
    Serial.print(k);
    Serial.print(" loops is ");
    Serial.print((time2-time1));
    Serial.println(" ms");
    time1 = time2;
    k=0;
  }
#endif

  // The theremin is active when a deadstick timeout has not occurred.
  isActive = !deadstickTimeout;

  // Periodically broadcast the measurements to the pixel minions.
  if (now - lastTxMs >= activeTxIntervalMs) {
    // When the theremin isn't active, we don't need to broadcast measurements as often.
    if (isActive || wasActive || now - lastTxMs >= inactiveTxIntervalMs) {
      lastTxMs = now;
      broadcastMeasurements();
      wasActive = isActive;
    }
  }

}
