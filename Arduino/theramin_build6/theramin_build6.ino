// theramin_build6.ino by C.Spindler 2-9-2020
// ...build6 will:
//      1. read sensors only when they test ready, like ...build5
//      2. improve ergonomic operation of sensors on pitch, volume and other
//          a. if all sensor unused (ie over a max DEAD_RANGE) for DEADSTICK_TIMEOUT_PERIOD ms,
//             all sound ceases
//          b. if volume sensor or pitch sensor goes out of range, volume level or pitch continues unchanged,
//             unless stopped for deadstick 
//          c. if pitch is bent up frequency with bend sensor, it returns to unbent condition when out of range, 
//             unless stopped by deadstick
//          d. range of pitch will be increased.
//      3. improve serial output (will not use printf(..), seems time wasting)
//
// based on theramin_build5.ino by C.Spindler 2-7-2020
// which tested at 2 ms per loop{...} at 11:30 pm on 2-7-2020 
//  ...build5  used public functions of the Pololu library
// for controlling the VL53L0x devices, which allow us to determine if a measurement
// is ready to be read, and then only read a TOF sensor when we know we will not wait.


// #include <LibPrintf.h> // always compile serial and printf

#define ENABLE_RADIO
//#define ENABLE_DEBUG_PRINT
#define ENABLE_LOOP_PRINT // comment out to drop printing in loop{...}
//#define ENABLE_STOPWATCH // comment out unless you want to to measure elapsed time between loops

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
#define RF_CHANNEL 80   // Electric Garden Theremin is on ch. 80, Illumicone is on ch. 97

// RF24_PA_MIN = -18 dBm, RF24_PA_LOW = -12 dBm, RF24_PA_HIGH = -6 dBm, RF24_PA_MAX = 0 dBm
#define RF_POWER_LEVEL RF24_PA_MAX


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


RF24 radio(9, 10);    // CE on pin 9, CSN on pin 10, also uses SPI bus (SCK on 13, MISO on 12, MOSI on 11)

static MeasurementVectorPayload payload;

static bool isActive;
static bool wasActive;

// TODO:  Need to select the pattern somehow.
static uint8_t patternNum = 1;

uint16_t dist1, dist2, dist3; // distN is global
bool deadstickTimeout = false;
unsigned long millis(void); // a function prototype


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
// Probably no need to ever set auto acknowledgement to false because the sender
// can control whether or not acks are sent by using the NO_ACK bit.
#define ACK_WIDGET_PACKETS true


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

// the following line is the same test that is used by
// uint16_t VL53L0X::readRangeContinuousMillimeters(void)
  if((sensor1.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
        dist1 = sensor1.readRangeContinuousMillimeters();
        // bool distNValid = (distN != 65535);
        }

  if((sensor2.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
        dist2 = sensor2.readRangeContinuousMillimeters();
        // bool distNValid = (distN != 65535);
        }

    if((sensor3.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
        dist3 = sensor3.readRangeContinuousMillimeters();
        // bool distNValid = (distN != 65535);
        }              

}

uint16_t getPitch(uint16_t dist){

// we will map an acceptable distance range from sensor loDist to hiDist
// to the frequency range that we desire loPitch to hiPitch, linearly
// there will be deadbands, from hiDist to MAXDISTANCE where we use hiPitch,
// and from loDist to MINDISTANCE where we use loPitch
//
// if the distance goes past MAXDISTANCE, getPitch just returns last valid frequency
// that sustained tone will work well with bendPitch(), and will eventually be stopped
// if there is a deadstickTimeout.  

   #define MAXDISTANCE  1100ul
   #define loDIST 50ul
   #define hiDIST 800ul
   #define loPITCH 30ul
   #define hiPITCH 6000ul
   
   static uint16_t last_frequency; // static initializes to zero by default
   uint16_t frequency;

   if(dist >= MAXDISTANCE ){           // if dist out of normal range, hold pitch & return
          frequency = last_frequency;
          return frequency;
          } 
          
   frequency = loPITCH;
   if (dist >= hiDIST) {
          frequency = hiPITCH;
          }else {
          if(dist > loDIST) { //only dist > loDIST and dist < hiDist reach here
            
              //map (loDIST to hiDIST) into (loPITCH to hiPITCH)
              // Careful, intermediate product may be too big for unit16_t and 
              // would be automatically promoted by the compiler to uint32_t. 
              // Assure that all of your defined literals are a sufficiently large
              // data type, for example unsigned long: 700ul
              // Also, assure that for all values of dist that can reach this
              // statement, the result in frequency are in the range from
              // loPITCH to hiPITCH, and that no intermediate arithemetic 
              // results are less than zero or higher than uint32_t can hold.
              
              frequency =(uint16_t) (loPITCH +  ((dist - loDIST)*(hiPITCH-loPITCH))/(hiDIST-loDIST)  ) ;

              }
         }
      last_frequency = frequency;
      return frequency;     
  }


void setPitch(uint16_t freq) {
       tone(SPEAKER, freq );// tone(pin,freq) sustains last freq unless noTone() is called
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

// map distances from TOF sensor  to the 129 steps,
// (0 to 128) of the MCP4231 103E digital potentiometer

#define MAXDISTANCE2 800ul

#define BOTTOMRANGE 30ul
#define TOPRANGE 600ul
#define LOSTEP 0ul
#define HISTEP 128ul

uint8_t volumeL;
static uint8_t lastVolumeL; 

if (dist >= MAXDISTANCE2){
     volumeL = lastVolumeL;

    }else {

uint8_t rangeStep = (TOPRANGE-BOTTOMRANGE)/(HISTEP - LOSTEP);
volumeL = LOSTEP;
if (dist >= TOPRANGE ){
    volumeL = HISTEP;
    } else { 
        if (dist > (BOTTOMRANGE + rangeStep )){
          // Careful, intermediate product may be too big for unit16_t and 
          // would be automatically promoted by the compiler to uint32_t. 
          // Assure that all of your defined literals are a sufficiently large
          // data type, for example, unsigned long: 700ul         
          // Also, assure that for all values of dist that can reach this
          // statement, the result in volumeLevel are in the range from
          // ( LOSTEP+1 ) to ( HISTEP-1 ), and that no intermediate arithemetic 
          // results are less than zero or higher than uint32_t can hold.
          volumeL = (uint8_t)((dist-BOTTOMRANGE)*HISTEP/(TOPRANGE-BOTTOMRANGE));
        }
    }        

    lastVolumeL = volumeL;

    }
return volumeL;
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
    Serial.println(F("radio.write failed."));
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

//huh?  printf_init(Serial);
  Serial.begin(115200);
#ifdef ENABLE_DEBUG_PRINT
  printf_begin();
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

// ======= vvv uh, no.
//  // per instructions from arduino.cc/en/reference/SPI 
//  // set the Nano's SS pin, 11, to OUTPUT to make it 
//  // unuasable as a chip select for whole Nano board as a SPI slave
//  // also, we use pin D11 for MOSI, which is compatable with OUTPUT
//  pinMode(11,OUTPUT);
// ======= ^^^ uh, no.

//  initialize SPI 
//  SPI.begin(); // comment this out if your radio library initializes SPI

  configureRadio(radio, TX_PIPE_ADDRESS, WANT_ACK, TX_RETRY_DELAY_MULTIPLIER,
                 TX_MAX_RETRIES, CRC_LENGTH, RF_POWER_LEVEL, DATA_RATE,
                 RF_CHANNEL);

  Wire.begin();

  setIdAndInit();

  payload.widgetHeader.id = widgetId;
  payload.widgetHeader.channel = 0;
}


void loop() {

#define DEAD_RANGE 1200ul
#define DEADSTICK_TIMEOUT_PERIOD 5000ul

  static unsigned long timeLastUsed;
  static int32_t lastTxMs;

  uint8_t Volume;
  uint16_t Pitch;

  uint32_t now = millis();

  read_three_sensors();

//  Serial.print("dist1: ");
//  Serial.print( dist1);
//  Serial.print("   dist2: ");
//  Serial.print( dist2); 
//  Serial.print("   dist3: ");
//  Serial.print( dist3);
  
  // here we'll call the musical functions
  // at this time there are only two output parameters, Pitch and Volume, which will be
  // global variables. 
  // if no one uses the sensors for DEADSTICK_TIMEOUT_PERIOD milliseconds, then we'll stop all sound
  // pitch will be adjustable over a wider range in software
  // sensor1 will be pitch, sensor2 will be volume and sensor3 will be special effects
  //       first (maybe only?) special effect: wammy bar, which bends pitch up and 
  //       returns to un-modified pitch when not used.

  if (!(dist1 > DEAD_RANGE && dist2 > DEAD_RANGE && dist3 > DEAD_RANGE)) {
    timeLastUsed = now;
  }

//  Serial.println();
//  Serial.print(" timeLastUsed: ");
//  Serial.print(timeLastUsed);
//  Serial.print(" now: ");
//  Serial.print( now );
//  Serial.print(" delta: ");
//  Serial.println(now - timeLastUsed);

  if ((now - timeLastUsed) > DEADSTICK_TIMEOUT_PERIOD) { 
    if (!deadstickTimeout) {
      // declare a DEADSTICK_TIMEOUT
      Serial.println(" DEADSTICK_TIMEOUT ");
      deadstickTimeout = true;
      noTone(SPEAKER);
    }
  }
  else {
    deadstickTimeout = false;
  }

  if (!deadstickTimeout) {
    Pitch = getPitch(dist1);
    //Pitch = bendPitch(Pitch, dist3);
    Volume = getVolumeLevel(dist2);

    setPitch(Pitch);
    setVolumeLevel(Volume);

    //Serial.print( "   Pitch = ");
    //Serial.print( Pitch );
    //Serial.print(" Volume = ");
    //Serial.println( Volume );
  }
   
  // #ifdef ENABLE_STOPWATCH, then time1 was initiaized to millis() in setup, 
  // and k, a static variable,  is initalized to zero on creation
#ifdef ENABLE_STOPWATCH
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
