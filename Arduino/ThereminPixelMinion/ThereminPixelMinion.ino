
/****************************************************************************
 *                                                                          *
 * Theremin Pixel Pattern Generator                                         *
 *                                                                          *
 * Author(s):  Ross Butler                                                  *
 *                                                                          *
 * January 2019                                                             *
 *                                                                          *
 * based on RF_mesh_TREES_receiver_side (Feb. 2019 version) by Jesse Cordtz *
 * and GardenSpinner (Dec. 2019 version) by Ross Butler                     *
 *                                                                          *
 ****************************************************************************/


// TODO:  force pattern 1 if no data received for some predetermined period (5 seconds or so)
// TODO:  get status led stuff from JohnsKaleidoscopeMirror


/***********
 * Options *
 ***********/

#define ENABLE_WATCHDOG
#define ENABLE_DEBUG_PRINT



/************
 * Includes *
 ************/

#include <avr/pgmspace.h>

#ifdef ENABLE_WATCHDOG
  #include <avr/wdt.h>
#endif

#include <FastLED.h>
#include <SPI.h>
#include "RF24.h"

#ifdef ENABLE_DEBUG_PRINT
  #include "printf.h"
#endif



/******************************
 * Target Structure Selection *
 ******************************/

// Enable only one of these.
//#define TARGET_IS_TREE
//#define TARGET_IS_CLOUD
#define TARGET_IS_ROSS_DEVL



/*********************************************
 * Implementation and Behavior Configuration *
 *********************************************/

#define LAMP_TEST_PIN A0
#define RGB_LED_RED_PIN 3
#define RGB_LED_GREEN_PIN 5
#define RGB_LED_BLUE_PIN 6
#define SIMULATION_PIN A1 

#define NUM_STRIPS 3

#define STRIP_0_CHIPSET WS2812B
#define STRIP_0_COLOR_ORDER GRB
#define STRIP_0_PIN 2

#define STRIP_1_CHIPSET WS2812B
#define STRIP_1_COLOR_ORDER GRB
#define STRIP_1_PIN 4

#define STRIP_2_CHIPSET WS2812B
#define STRIP_2_COLOR_ORDER GRB
#define STRIP_2_PIN 7

//constexpr uint8_t numBottomSectionPixels[NUM_STRIPS] = {150, 150, 150};
//constexpr uint8_t numTopSectionPixels[NUM_STRIPS] = {50, 50, 50};
constexpr uint8_t numBottomSectionPixels[NUM_STRIPS] = {15, 64, 150};
constexpr uint8_t numTopSectionPixels[NUM_STRIPS] = {35, 32, 50};

/*
#if defined(TARGET_IS_TREE)
  #define NUM_LEDS 300
  #define NUM_LEDS_IN_BOTTOM_PART 150
  #define BRIGHTNESS 200
#elif defined(TARGET_IS_CLOUD)
  #define NUM_LEDS 300
  #define NUM_LEDS_IN_BOTTOM_PART 150
  #define BRIGHTNESS 200
#elif defined(TARGET_IS_ROSS_DEVL)
  #define NUM_LEDS 50
  #define NUM_LEDS_IN_BOTTOM_PART 25
  #define BRIGHTNESS 64
#else
  #error No target defined.
#endif
*/

constexpr uint8_t overallBrightness = 64;

#define LED_FRAMES_PER_SECOND 24
#define PATTERN_UPDATE_INTERVAL_MS 30

#define LAMP_TEST_ACTIVE LOW
#define LAMP_TEST_INTENSITY 255

// Use these for common-anode RGB LED.
//constexpr uint8_t rgbLedLowIntensity = 255;
//constexpr uint8_t rgbLedHighIntensity = 0;
// Use these for common-cathode RGB LED.
constexpr uint8_t rgbLedLowIntensity = 0;
constexpr uint8_t rgbLedHighIntensity = 255;

#define SIMULATION_ACTIVE LOW
#define SIMULATED_MEASUREMENT_UPDATE_INTERVAL_MS 15
#define SIMULATED_MEASUREMENT_STEP 1

// The theremin collects and sends three distance measurements.
constexpr uint8_t numDistanceMeasmts = 3;

// the maximum distance (in mm) when simulating measurements
constexpr int16_t maxDistance = 1000;

// We need to receive at least newPatternRepetitionThreshold consecutive
// messages with the same new pattern number before we change the pattern.
// This should prevent unwanted flashing if a bad message is received.  (It
// is rare, but a corrupt message can make it through the NRF24L01's CRC check.)
constexpr uint8_t newPatternRepetitionThreshold = 3;

// free memory that must remain after allocating pixel data arrays
constexpr uint16_t reservedRamSize = 128;

// for rejecting bad messages that make it past the CRC check
// (which happens with the NRF24L01+ way more than you'd think)
constexpr int16_t minValidPatternNum = 0;
constexpr int16_t maxValidPatternNum = 255;
constexpr int16_t minValidDistance = 0;
constexpr int16_t maxValidDistance = 4000;



/***********************
 * Radio Configuration *
 ***********************/

// Possible data rates are RF24_250KBPS, RF24_1MBPS, or RF24_2MBPS (genuine Noric chips only).
#define DATA_RATE RF24_1MBPS

// Valid CRC length values are RF24_CRC_8, RF24_CRC_16, and RF24_CRC_DISABLED
#define CRC_LENGTH RF24_CRC_16

// nRF24 frequency range:  2400 to 2525 MHz (channels 0 to 125)
// ISM: 2400-2500;  ham: 2390-2450
// WiFi ch. centers: 1:2412, 2:2417, 3:2422, 4:2427, 5:2432, 6:2437, 7:2442,
//                   8:2447, 9:2452, 10:2457, 11:2462, 12:2467, 13:2472, 14:2484
// Illumicone widgets use channel 97, so it is wise to pick something else.
#define RF_CHANNEL 97  //80

// Nwdgt, where N indicates the pipe number (0-6) and payload type (0: stress test;
// 1: position & velocity; 2: measurement vector; 3,4: undefined; 5: custom
constexpr uint8_t readPipeAddresses[][6] = {"0wdgt", "1wdgt", "2wdgt", "3wdgt", "4wdgt", "5wdgt"};
constexpr int numReadPipes = sizeof(readPipeAddresses) / (sizeof(uint8_t) * 6);

// Probably no need to ever set auto acknowledgement to false because the sender
// can control whether or not acks are sent by using the NO_ACK bit.
#define ACK_WIDGET_PACKETS true

// RF24_PA_MIN = -18 dBm, RF24_PA_LOW = -12 dBm, RF24_PA_HIGH = -6 dBm, RF24_PA_MAX = 0 dBm
#define RF_POWER_LEVEL RF24_PA_MAX

// 250 us additional delay multiplier (0-15)
#define TX_RETRY_DELAY_MULTIPLIER 15

// max retries (0-15)
#define TX_MAX_RETRIES 15



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



/***********************
 * Types and Constants *
 ***********************/



/***********
 * Globals *
 ***********/

static uint8_t numPixels[NUM_STRIPS];
static CRGB* pixels[NUM_STRIPS];

static bool widgetIsActive;
static uint8_t currentPatternNum;
static int16_t currentDistance[numDistanceMeasmts];

static RF24 radio(9, 10);    // CE on pin 9, CSN on pin 10, also uses SPI bus (SCK on 13, MISO on 12, MOSI on 11)



/***********
 * Helpers *
 ***********/

uint16_t freeRam() 
{
  // Based on code retrieved on 1 April 2015 from
  // https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory

  extern int __heap_start, *__brkval; 
  int v; 
  return (uint16_t) &v - (__brkval == 0 ? (uint16_t) &__heap_start : (uint16_t) __brkval); 
}



/************
 * Patterns *
 ************/

/* all off */
void pattern0(CRGB* pixelSection, uint8_t numPixelsInSection, uint8_t iStrip, uint8_t iSection)
{
  fill_solid(pixelSection, numPixelsInSection, CRGB::Black);
}


/* rainbow */
void pattern1(CRGB* pixelSection, uint8_t numPixelsInSection, uint8_t iStrip, uint8_t iSection)
{
  fill_rainbow(pixelSection, numPixelsInSection, 0, 255 / numPixelsInSection);
}


/* section highlight */
void pattern15(CRGB* pixelSection, uint8_t numPixelsInSection, uint8_t iStrip, uint8_t iSection)
{
  static uint8_t colorIdx;
  const CRGB colors[] = {CRGB::Red, CRGB::Yellow, CRGB::Green, CRGB::Cyan, CRGB::Blue, CRGB::Purple};

  // Always start at the first color when doing the first section of the
  // first strip.  This makes the pattern static instead of flickering.
  if (iStrip == 0 && iSection == 0) {
    colorIdx = 0;
  }

  fill_solid(pixelSection, numPixelsInSection, colors[colorIdx]);
  pixelSection[0] = CRGB::White;
  pixelSection[numPixelsInSection - 1] = CRGB::White;

  if (++colorIdx >= sizeof(colors) / sizeof(CRGB)) {
    colorIdx = 0;
  }
}


void updatePattern()
{
  if (digitalRead(LAMP_TEST_PIN) == LAMP_TEST_ACTIVE) {
    CRGB lampTestColor = CRGB::White;
    lampTestColor.nscale8_video(LAMP_TEST_INTENSITY);
    for (uint8_t i = 0; i < NUM_STRIPS; ++i) {
      if (pixels[i] != nullptr) {
        fill_solid(pixels[i], numPixels[i], lampTestColor);
      }
    }
    // TODO:  set all the pixels to white at LAMP_TEST_INTENSITY
    return;
  }

  for (uint8_t iStrip = 0; iStrip < NUM_STRIPS; ++iStrip) {
    if (pixels[iStrip] == nullptr) {
      continue;
    }
    for (uint8_t iSection = 0; iSection < 2; ++iSection) {
 
      CRGB* pixelSection;
      uint8_t numPixelsInSection;
      if (iSection == 0) {
        if (numBottomSectionPixels[iStrip] == 0) {
          continue;
        }
        pixelSection = pixels[iStrip];
        numPixelsInSection = numBottomSectionPixels[iStrip];
      }
      else {
        if (numTopSectionPixels[iStrip] == 0) {
          continue;
        }
        pixelSection = pixels[iStrip] + numBottomSectionPixels[iStrip];
        numPixelsInSection = numTopSectionPixels[iStrip];
      }

      switch (currentPatternNum) {
        case 0:
          pattern0(pixelSection, numPixelsInSection, iStrip, iSection);
          break;
        case 1:
          pattern1(pixelSection, numPixelsInSection, iStrip, iSection);
          break;
        case 15:
          pattern15(pixelSection, numPixelsInSection, iStrip, iSection);
          break;
        default:
          break;
          // TODO:  turn status LED orange or something because the pattern number is invalid
      }
    }
  }
}



/***********************
 * Radio Communication *
 ***********************/

bool handleMeasurementVectorPayload(const MeasurementVectorPayload* payload, uint8_t payloadSize)
{
  // Returns true if we got a valid measurement vector payload.

  static uint8_t newPatternRepetitionCount;

  // The theremin sends the pattern number plus a predefined number of distance measurements.
  constexpr uint16_t expectedPayloadSize = sizeof(WidgetHeader) + sizeof(int16_t) * (numDistanceMeasmts + 1);
  if (payloadSize != expectedPayloadSize) {
#ifdef ENABLE_DEBUG_PRINT
    Serial.print(F("got MeasurementVectorPayload from widget "));
    Serial.print(payload->widgetHeader.id);
    Serial.print(F(" with "));
    Serial.print(payloadSize);
    Serial.print(F(" bytes but expected "));
    Serial.print(expectedPayloadSize);
    Serial.println(F(" bytes"));    
#endif
    return false;
  }

  // Sanity check the data in case a bad message made it past the CRC check.
  bool gotValidData = (payload->measurements[0] >= minValidPatternNum && payload->measurements[0] <= maxValidPatternNum);
  for (uint8_t i = 1; gotValidData && i <= numDistanceMeasmts; ++i) {
    gotValidData = (payload->measurements[i] >= minValidDistance && payload->measurements[i] <= maxValidDistance);
  }
  if (!gotValidData) {
#ifdef ENABLE_DEBUG_PRINT
    Serial.println(F("got message with invalid data"));
#endif
    return false;
  }
  
  // There is only one theremin, so we don't worry about the widget id in payload->widgetHeader.id.

  if (payload->widgetHeader.isActive) {
    widgetIsActive = true;

    // Update currentPatternNum only if we've received the same new pattern number multiple times.
    uint8_t newPatternNum = (uint8_t) payload->measurements[0];
    if (newPatternNum != currentPatternNum && ++newPatternRepetitionCount >= newPatternRepetitionThreshold) {
      newPatternRepetitionCount = 0;
      currentPatternNum = newPatternNum;
#ifdef ENABLE_DEBUG_PRINT
      Serial.print(F("got new pattern "));
      Serial.println(newPatternNum);
#endif
    }

    for (uint8_t i = 0; i < numDistanceMeasmts; ++i) {
      currentDistance[i] = payload->measurements[i + 1];
    }

#ifdef ENABLE_DEBUG_PRINT
  Serial.print(F("got distances"));
  for (uint8_t i = 0; i < numDistanceMeasmts; ++i) {
    Serial.print(" ");
    Serial.print(currentDistance[i]);
  }
  Serial.println();
#endif

  }
  else {
    widgetIsActive = false;
  }

  return true;
}


void pollRadio()
{
  uint8_t pipeNum;
  if (!radio.available(&pipeNum)) {
    return;
  }

  constexpr uint8_t maxPayloadSize = 32 + sizeof(WidgetHeader);
  uint8_t payload[maxPayloadSize];
  uint8_t payloadSize = radio.getDynamicPayloadSize();
  if (payloadSize > maxPayloadSize) {
#ifdef ENABLE_DEBUG_PRINT
    Serial.print(F("got message on pipe "));
    Serial.print(pipeNum);
    Serial.print(F(" with payload size "));
    Serial.print(payloadSize);
    Serial.print(F(" but maximum payload size is "));
    Serial.println(maxPayloadSize);
#endif
    return;
  }
#ifdef ENABLE_DEBUG_PRINT
  Serial.print(F("got message on pipe "));
  Serial.println(pipeNum);
#endif

  radio.read(payload, payloadSize);

  bool gotValidPayload = false;
  switch(pipeNum) {
    case 2:
        gotValidPayload = handleMeasurementVectorPayload((MeasurementVectorPayload*) payload, payloadSize);
        break;
    default:
#ifdef ENABLE_DEBUG_PRINT
      Serial.print(F("got message on unsupported pipe "));
      Serial.println(pipeNum);
#endif
      break;
  }

  // TODO:  turn the status LED red if we didn't get a valid payload
}



/**************************
 * Measurement Simulation *
 **************************/

void updateSimulatedMeasurements()
{
  EVERY_N_MILLISECONDS(SIMULATED_MEASUREMENT_UPDATE_INTERVAL_MS) {
    // TODO:  make this more varied and realistic
    for (uint8_t i = 0; i < numDistanceMeasmts; ++i) {
      currentDistance[i] += SIMULATED_MEASUREMENT_STEP;
      if (currentDistance[i] >= maxDistance) {
        currentDistance[i] = 0;
      }
    }
    widgetIsActive = true;
  }
}



/******************
 * Initialization *
 ******************/

void initGpios()
{
  // FastLED takes care of setting up the pixel data output pins.

  pinMode(LAMP_TEST_PIN, INPUT_PULLUP);
  pinMode(RGB_LED_RED_PIN, OUTPUT);
  pinMode(RGB_LED_GREEN_PIN, OUTPUT);
  pinMode(RGB_LED_BLUE_PIN, OUTPUT);
  pinMode(SIMULATION_PIN, INPUT_PULLUP);

  // Turn the status LED half-intensity white.
  analogWrite(RGB_LED_RED_PIN , map(128, 0, 255, rgbLedLowIntensity, rgbLedHighIntensity));
  analogWrite(RGB_LED_GREEN_PIN , map(128, 0, 255, rgbLedLowIntensity, rgbLedHighIntensity));
  analogWrite(RGB_LED_BLUE_PIN , map(128, 0, 255, rgbLedLowIntensity, rgbLedHighIntensity));
}


void initRadio()
{
  Serial.println(F("Initializing radio..."));    
  radio.begin();

  radio.setPALevel(RF_POWER_LEVEL);
  radio.setRetries(TX_RETRY_DELAY_MULTIPLIER, TX_MAX_RETRIES);
  radio.setDataRate(DATA_RATE);
  radio.setChannel(RF_CHANNEL);
  radio.setAutoAck(ACK_WIDGET_PACKETS);
  radio.enableDynamicPayloads();
  radio.setCRCLength(CRC_LENGTH);

  // Unlike widgetRcvr, we don't open pipe 0.
  for (uint8_t i = 0; i < numReadPipes; ++i) {
      radio.openReadingPipe(i, readPipeAddresses[i]);
  }

#ifdef ENABLE_DEBUG_PRINT 
  radio.printDetails();
#endif
  
  radio.startListening();

  Serial.println(F("Radio initialized."));    
}


bool initPixels()
{
  bool retval = true;

  for (uint8_t i = 0; i < NUM_STRIPS; ++i) {

    pixels[i] = nullptr;

    numPixels[i] = numBottomSectionPixels[i] + numTopSectionPixels[i];
    if (numPixels[i] == 0) {
      continue;
    }

    uint16_t availableRam = freeRam() > reservedRamSize ? freeRam() - reservedRamSize : 0;
    uint16_t requiredRam = numPixels[i] * sizeof(CRGB);
#ifdef ENABLE_DEBUG_PRINT
    Serial.print(F("Pixel strip "));
    Serial.print(i);
    Serial.print(F(" requires "));
    Serial.print(requiredRam);
    Serial.print(F(" bytes; "));
    Serial.print(availableRam);
    Serial.println(F(" bytes are available."));
#endif
    if (requiredRam <= availableRam) {
      pixels[i] = new CRGB[numPixels[i]];
    }
    else {
#ifdef ENABLE_DEBUG_PRINT
      Serial.println(F("*** insufficient RAM for strip"));
#endif
      retval = false;
    }
  }

#if (NUM_STRIPS >= 1)
  if (pixels[0] != nullptr) {
    FastLED.addLeds<STRIP_0_CHIPSET, STRIP_0_PIN, STRIP_0_COLOR_ORDER>(pixels[0], numPixels[0]);
  }
#endif 
#if (NUM_STRIPS >= 2)
  if (pixels[1] != nullptr) {
    FastLED.addLeds<STRIP_1_CHIPSET, STRIP_1_PIN, STRIP_1_COLOR_ORDER>(pixels[1], numPixels[1]);
  }
#endif 
#if (NUM_STRIPS >= 3)
  if (pixels[2] != nullptr) {
    FastLED.addLeds<STRIP_2_CHIPSET, STRIP_2_PIN, STRIP_2_COLOR_ORDER>(pixels[2], numPixels[2]);
  }
#endif 
#if (NUM_STRIPS >= 4)
  #error Too many strips configured.
#endif
#if (!defined(NUM_STRIPS) || NUM_STRIPS <= 0)
  #error Invalid number of strips configured.
#endif

  FastLED.setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(overallBrightness);

  return retval;
}


void setup()
{
#ifdef ENABLE_DEBUG_PRINT
  Serial.begin(115200);
  printf_begin();
  Serial.println(F("Debug print enabled."));    
#endif

  initGpios();
  initRadio();
  initPixels();

#ifdef ENABLE_WATCHDOG
  wdt_enable(WDTO_1S);     // enable the watchdog
#endif
}



/************
 * Run Loop *
 ************/

void loop()
{
  if (digitalRead(SIMULATION_PIN) == SIMULATION_ACTIVE) {
    updateSimulatedMeasurements();
  }
  else {
    pollRadio();
  }

  // Periodically update the pattern.
  EVERY_N_MILLISECONDS(PATTERN_UPDATE_INTERVAL_MS) {
    updatePattern();
  }

  // Periodically write to the LEDs.
  EVERY_N_MILLISECONDS(1000 / LED_FRAMES_PER_SECOND) {
    FastLED.show();
  }

#ifdef ENABLE_WATCHDOG
  // Kick the dog (gently, of course) to let him know we're still alive.
  wdt_reset();
#endif
}
