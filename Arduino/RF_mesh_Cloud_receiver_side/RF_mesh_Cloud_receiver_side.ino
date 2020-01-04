#include <FastLED.h>
#include <SoftwareSerial.h>

//#define ROSS_DEVL


/*****************
 * Configuration *
 *****************/

//fastLED
#define LED_STRIP_PIN 3
#define COLOR_ORDER GRB
#define CHIPSET WS2812B
#ifndef ROSS_DEVL
// LED counts and brightness for trees
#define NUM_LEDS 300
#define BRIGHTNESS 255
#else
// LED counts and brightness for Ross's development board
#define NUM_LEDS 50
#define BRIGHTNESS 64
#endif

// timing
#define LED_FRAMES_PER_SECOND 24
#define PATTERN_UPDATE_INTERVAL_MS 30
#define HUE_STEP_INTERVAL_MS 20
#define PALETTE_BLEND_INTERVAL_MS 10

#ifndef ROSS_DEVL
// HC-12 configuration for JUMP
#define HC12_TX_TO_ARDUINO_RX_PIN 10
#define HC12_RX_FROM_ARDUINO_TX_PIN 11
#else
// HC-12 configuration for Ross'd development board
#define HC12_TX_TO_ARDUINO_RX_PIN 2
#define HC12_RX_FROM_ARDUINO_TX_PIN 4
#endif

#define NUM_BENDERS 3


/*********
 * Types *
 *********/

enum class MsgType {
  none,
  button,
  bender
};


/***********
 * Globals *
 ***********/

static CRGB leds[NUM_LEDS];

static uint16_t dist;             // A random number for our noise generator.
static uint16_t scale = 30;       // Wouldn't recommend changing this on the fly, or the animation will be really blocky.
static uint8_t maxChanges = 48;   // Value for blending between palettes.

// variables for aniamtion
static int waveA = 10;
static int waveB = 1;
static int waveC = 4;

static CRGBPalette16 currentPalette(RainbowColors_p);
static CRGBPalette16 targetPalette(PartyColors_p);

static SoftwareSerial HC12(HC12_TX_TO_ARDUINO_RX_PIN, HC12_RX_FROM_ARDUINO_TX_PIN);

static uint8_t gHue = 0;       // rotating "base color" used by many of the patterns

static byte benders[NUM_BENDERS];


/************
 * Patterns *
 ************/

// FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.


int bend(int value, byte benderNumber)
{
  // If the bender number is out of range 1 - NUM_BENDERS, ignore
  // the bend request and just return the value unmodified.
  if (benderNumber > NUM_BENDERS) {
    return value;
  }

  // bender
  // value   effect
  // ======= =========================================================================
  //   0- 63 divide value by (63 - bender) / 10 + 1.1 (i.e., divide by 7.4 to 1.1)
  //      64 no effect
  //  65-127 multiply value by (bender - 65) / 10 + 1.1 (i.e., multiply by 1.1 to 7.4)
  // 128-255 no effect

  int bentValue;

  uint8_t benderValue = benders[benderNumber - 1];

  if (benderValue == 64 || benderValue > 127) {
    bentValue = value;
  }
  // Below, we apply the benderValue, which is normalized to the
  // range 11 to 74 but not scaled to the advertised range 1.1 to 7.4, directly to value then divide
  // the result by 10, giving the final, bent value.  By doing the intermediate
  // calculations scaled up by a factor of 10 then doing the division by 10 last, we
  // avoid doing floating point math, which is very expensive time-wise on an 8-bit
  // Arduino that doesn't have floating point support in the processor.
  else if (benderValue <= 63) {
    bentValue = (value * 10 / ((63 - (int) benderValue) + 11));
  }
  else {
    bentValue = (value * (((int) benderValue - 65) + 11)) / 10;
  }

//  Serial.print("value=");
//  Serial.print(value);
//  Serial.print(" benderNumber=");
//  Serial.print(benderNumber);
//  Serial.print(" benderValue=");
//  Serial.print(benderValue);
//  Serial.print(" bentValue=");
//  Serial.println(bentValue);

  return bentValue;
}


void addGlitter(fract8 chanceOfGlitter)
{
  if(random8() < chanceOfGlitter) {
    leds[random16(NUM_LEDS)] += CRGB::White;
  }
}

void addRed(fract8 chanceOfGlitter)
{
  if(random8() < chanceOfGlitter) {
    leds[random16(NUM_LEDS)] += CHSV(0, 255, 192);
  }
}
void addPink(fract8 chanceOfGlitter)
{
  if(random8() < chanceOfGlitter) {
    leds[random16(NUM_LEDS)] += CHSV(224, 255, 192);
  }
}

// ---------- Noise Generator1 ----------
void fillnoise8()
{
  // Just ONE loop to fill up the LED array as all of the pixels change.
  for (int i = 0; i < NUM_LEDS; i++) {
    // Get a value from the noise function. I'm using both x and y axis.
    uint8_t index = inoise8(i*scale, dist+i*scale) % 255;
    // With that value, look up the 8 bit colour palette value and assign it to the current LED.
    leds[i] = ColorFromPalette(currentPalette, index, 255, LINEARBLEND);
  }
  // Moving along the distance (that random number we started out with). Vary it a bit with a sine wave.
  dist += beatsin8(waveA, waveB, waveC);

  // In some sketches, I've used millis() instead of an incremented counter. Works a treat.
}


void pattern0_off()
{
  // sine variables
  waveA = 12;           // high number more speratic 0-150 default 10
  waveB = .5;           // fast pulse a high number 0-5     default 1
  waveC = 2;            // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(PALETTE_BLEND_INTERVAL_MS) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                           // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
     targetPalette = CRGBPalette16(CRGB::Black); }
}


void pattern1_rainbowSparkle()
{
  // FastLED's built-in rainbow generator
  fill_rainbow(leds, NUM_LEDS, gHue, 7);
  addGlitter(bend(20, 1));
}


void pattern2_ocean()
{
  // sine variables
  waveA = 25;           // high number more speratic 0-150 default 10
  waveB = .3;           // fast pulse a high number 0-5     default 1
  waveC = 1;            // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(PALETTE_BLEND_INTERVAL_MS) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                           // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
     targetPalette = CRGBPalette16(OceanColors_p);
  }
}


void pattern3_rainbowStripe()
{
  // sine variables
  waveA = 20;           // high number more speratic 0-150 default 10
  waveB = 1;            // fast pulse a high number 0-5     default 1
  waveC = 2;            // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(PALETTE_BLEND_INTERVAL_MS) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                           // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
     targetPalette = CRGBPalette16(RainbowColors_p);
  }
}


void pattern4_party()
{
  // sine variables
  waveA = 26;           // high number more speratic 0-150 default 10
  waveB = 3.5;          // fast pulse a high number 0-5     default 1
  waveC = 16;           // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(PALETTE_BLEND_INTERVAL_MS) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                          // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
    targetPalette = CRGBPalette16(PartyColors_p);   // max of 4 colors
 }
}


void pattern5_cloud()
{
  // sine variables
  waveA = 2;            // high number more speratic 0-150 default 10
  waveB = .1;           // fast pulse a high number 0-5     default 1
  waveC = 2;            // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(PALETTE_BLEND_INTERVAL_MS) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                           // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
    targetPalette = CRGBPalette16(CloudColors_p);   // max of 4 colors
  }
}


void pattern6_lava()
{
  // sine variables
  waveA = 3;            // high number more speratic 0-150 default 10
  waveB = .3;           // fast pulse a high number 0-5     default 1
  waveC = 1;            // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(PALETTE_BLEND_INTERVAL_MS) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    fillnoise8();                                                           // Update the LED array with noise at the new location
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
    targetPalette = CRGBPalette16(LavaColors_p); //max of 4 colors
  }
}


void pattern7_pinky()
{
  // sine variables
  waveA = 3;    // high number more speratic 0-150 default 10
  waveB = .3;   // fast pulse a high number 0-5     default 1
  waveC = 1;    // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(PALETTE_BLEND_INTERVAL_MS) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    // Update the LED array with noise at the new location
   // fillnoiseT();
   // fillnoiseB();
    addRed(bend(30, 1));
    addPink(60);
    addGlitter(bend(25, 2));
    fadeToBlackBy(leds, NUM_LEDS, bend(10, 3));
  }

  EVERY_N_SECONDS(1) {            // Change the target palette to a random one periodically.
    targetPalette = CRGBPalette16(LavaColors_p);  // max of 4 colors
  }
}


void pattern8_singleTrailz()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy(leds, NUM_LEDS, bend(30, 1));
  unsigned int pos = beatsin16(30, 0, NUM_LEDS - 1);    // (speed, firstled, lastled)
  if (pos < NUM_LEDS) {           // it should always be in bounds, but defensive programming is a good thing
    leds[pos] += CHSV(gHue, 255, 192);
  }
}


bool pattern15_startup()
{
  static unsigned int dot = 0;
  
  EVERY_N_MILLISECONDS(30) {
    if (dot > 0) {
      // clear the led we turned on last time
      leds[dot - 1] = CRGB::Black;
    }
    leds[dot] = CRGB::Blue;
    ++dot;
  }

  if (dot < NUM_LEDS) {
    return true;          // keep running
  }
  else {
    return false;         // switch to another pattern
  }
}


/**********************
 * Setup and Run Loop *
 **********************/

void setup()
{
  delay(3000);              // sanity delay

  Serial.begin(9600);       // Open serial port to computer
  HC12.begin(9600);         // Open serial port to HC12

  //fastled setup
  FastLED.addLeds<CHIPSET, LED_STRIP_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  dist = random16(12345);   // A semi-random number for our noise generator

  // Initialize the benders to have no effect.
  for (byte i = 0; i < NUM_BENDERS; ++i) {
    benders[i] = 64;
  }
}


void loop()
{
  const byte startButtonMarker = '^';
  const byte startBenderMarker = '&';
  const byte endMarker = '%';
  const byte rxBufSize = 3;

  static int Active;
  static int State = 1;                     // the current pattern being displayed (or 86 if none)
  static MsgType rxMsgType = MsgType::none; // the type of message being received but not complete yet
  static MsgType msgType = MsgType::none;   // the type of complete message currently in rxBuf and needing to be processed
  static byte rxBuf[rxBufSize];             // holds received data until we have a complete message
  static byte rxBufLen;                     // index of the next slot to fill in rxBuf and also the number of bytes in the buffer
  static boolean gotStartMarker = false;    // gets set true when we receive any start marker

//  // Print the average (with N = 64) time spent in each loop iteration.
//  static unsigned long lastLoopMs;
//  static unsigned long sumLoopMs;
//  static byte loopTimeCount;
//  unsigned long now = millis();
//  sumLoopMs += (now - lastLoopMs);
//  lastLoopMs = now;
//  ++loopTimeCount;
//  if (loopTimeCount >= 64) {
//    unsigned long avgLoopMs = sumLoopMs >> 6;
//    Serial.print("avgLoopMs=");
//    Serial.println(avgLoopMs);
//    sumLoopMs = 0;
//    loopTimeCount = 0;
//  }

//  // for cylcing the code without the controller
//  static long timer = 60000;
//  static long timekeeper = 0;
//  unsigned long timerMillis = millis();
//  if (timerMillis - timekeeper > timer) {
//    timekeeper = timerMillis;
//    State = State + 1;
//    if (State == 8) {
//      State = 0;
//    }
//  }

  // Process data received by the HC-12 until it has
  // no more data or until we have a complete message.
  while (HC12.available() && msgType == MsgType::none) {
    byte incomingByte = HC12.read();            // get an incoming byte from HC-12
    Serial.print("0x");
    Serial.println(incomingByte, HEX);
    // Saves the data between the start and end markers.
    if (gotStartMarker) {                       // We have received a start marker?
      if (incomingByte != endMarker) {
        if (rxBufLen < rxBufSize) {             // There is room in the buffer?
          rxBuf[rxBufLen++] = incomingByte;     // Put byte in receive buffer and increment the index of the next slot to fill.
        }
        else {                                  // Buffer overflow, so throw away everything and wait for a new message.
          Serial.println("rxBuf overflow!");
          gotStartMarker = false;
          rxMsgType = MsgType::none;            // not currently receiving a message
        }
      }
      else {                                    // We've reached the end marker.
        gotStartMarker = false;                 // Don't save any more characters.
        msgType = rxMsgType;                    // indicate what type of complete message is in the buffer
        rxMsgType = MsgType::none;              // not currently receiving a message
      }
    }
    // Checks whether the received message starts with the start marker.
    else if (incomingByte == startButtonMarker) {
      gotStartMarker = true;
      rxMsgType = MsgType::button;
      rxBufLen = 0;
    }
    else if (incomingByte == startBenderMarker) {
      gotStartMarker = true;
      rxMsgType = MsgType::bender;
      rxBufLen = 0;
    }
  }

  if (msgType == MsgType::button) {
    msgType = MsgType::none;                    // Indicate that the message has been handled.

    // Bluetooth Comm -------------------------------------
    // convert
    // current = msgData.toInt();

    byte msgData = rxBuf[0];
    if      (msgData == '0') Active = 0;
    else if (msgData == '1') Active = 1;
    else if (msgData == '2') Active = 2;
    else if (msgData == '3') Active = 3;
    else if (msgData == '4') Active = 4;
    else if (msgData == '5') Active = 5;
    else if (msgData == '6') Active = 6;
    else if (msgData == '7') Active = 7;
    else if (msgData == '8') Active = 8;
    else Active = 86;

    // Change the state (pattern) if the message told us to do so.
    if (Active < 80) {
      State = Active;
    }

    Serial.print("got button message; msgData=");
    Serial.print(msgData);
    Serial.print(" Active=");
    Serial.print(Active);
    Serial.print(" State=");
    Serial.println(State);
  }

  // Process a bender message.
  if (msgType == MsgType::bender) {
    msgType = MsgType::none;                    // Indicate that the message has been handled.
    if (rxBufLen == NUM_BENDERS) {              // Accept only complete bender messages.
      // Valid bender values are 0 - 127 but are transmitted with the high-order bit
      // set so that they are not confused with message start or end characters.
      // If the high-order bit isn't set, we'll ignore the value because it is probably
      // junk.  Otherwise, we'll clear the high-order bit and save the bender value.
      for (byte i = 0; i < NUM_BENDERS; ++i) {
        if (rxBuf[i] & 0x80) {                  // high-order bit is set (byte is probably a valid bender value)?
          benders[i] = rxBuf[i] & 0x7f;         // keep only lower 7 bits (0 - 127)
          Serial.print("benders[");
          Serial.print(i);
          Serial.print("]=");
          Serial.println(benders[i]);
        }
      }
    }
  }

  // slowly cycle the "base color" through the rainbow
  EVERY_N_MILLISECONDS(HUE_STEP_INTERVAL_MS) {
    gHue++;
  }

  // Periodically update the patterns.
  EVERY_N_MILLISECONDS(PATTERN_UPDATE_INTERVAL_MS) {

    switch (State) {
      case 0:
        pattern0_off();
        break;
      case 1:
        pattern1_rainbowSparkle();
        break;
      case 2:
        pattern3_rainbowStripe();
        break;
      case 3:
      pattern4_party();
        break;
      case 4:
        pattern2_ocean();
        break;
      case 5:
        pattern5_cloud();
        break;
      case 6:
        pattern6_lava();
        break;
      case 7:
        pattern7_pinky();
        break;
      case 8:
        pattern8_singleTrailz();
        break;
      case 15:
        // When pattern 15 is finished, switch to pattern 5.
        if (!pattern15_startup()) {
          State = 5;
        }
        break;
      //default:
        // TODO ross 9 Feb 2018:  We should indicate an internal error somehow if State is invalid.
    }
  }

  // Write to the LEDs approximately LED_FRAMES_PER_SECOND times per second.
  EVERY_N_MILLISECONDS(1000 / LED_FRAMES_PER_SECOND) {
    LEDS.show();
  }
}

