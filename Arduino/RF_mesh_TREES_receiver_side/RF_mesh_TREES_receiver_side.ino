#include <FastLED.h>
#include <SoftwareSerial.h>


/*****************
 * Configuration *
 *****************/

//fastLED 
#define LED_STRIP_PIN 3
#define COLOR_ORDER GRB
#define CHIPSET WS2812B
#define NUM_LEDS    50  //ross 300
#define NUM_LEDS_IN_BOTTOM_PART 25  //ross 150
#define BRIGHTNESS 200
#define FRAMES_PER_SECOND 30

// HC-12 configuration for JUMP
#define HC12_TX_TO_ARDUINO_RX_PIN 10
#define HC12_RX_FROM_ARDUINO_TX_PIN 11
// HC-12 configuration for Ross'd development board
//#define HC12_TX_TO_ARDUINO_RX_PIN 2
//#define HC12_RX_FROM_ARDUINO_TX_PIN 4


/***********
 * Globals *
 ***********/

static CRGB leds[NUM_LEDS];

static uint16_t dist;             // A random number for our noise generator.
static uint16_t scale = 30;       // Wouldn't recommend changing this on the fly, or the animation will be really blocky.
static uint8_t maxChanges = 48;   // Value for blending between palettes.

//variables for aniamtion
static int waveA = 10;
static int waveB = 1;
static int waveC = 4;

static CRGBPalette16 currentPalette(RainbowColors_p);
static CRGBPalette16 targetPalette(PartyColors_p);

static SoftwareSerial HC12(HC12_TX_TO_ARDUINO_RX_PIN, HC12_RX_FROM_ARDUINO_TX_PIN);

static int randomNumber;
static int randomNumberB;
static int randomNumberT;

static uint8_t gHue = 0;       // rotating "base color" used by many of the patterns


/************
 * Patterns *
 ************/

// FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.


void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[random16(NUM_LEDS)] += CRGB::White;
  }
}


// ---------- Noise Generator1 (for bottom part of tree) ----------
void fillnoiseB()
{
  // Just ONE loop to fill up the LED array as all of the pixels change.
  for(int i = 0; i < NUM_LEDS_IN_BOTTOM_PART; i++) {
    // Get a value from the noise function. I'm using both x and y axis.
    uint8_t index = inoise8(i*scale, dist+i*scale) % 255;
    // With that value, look up the 8 bit colour palette value and assign it to the current LED.
    leds[i] = ColorFromPalette(currentPalette, index, 255, LINEARBLEND);
  }
  // Moving along the distance (that random number we started out with). Vary it a bit with a sine wave.
  dist += beatsin8(waveA, waveB, waveC);

  // In some sketches, I've used millis() instead of an incremented counter. Works a treat.
}


// ---------- Noise Generator2 (for top part of tree) ----------
void fillnoiseT()
{
  // Just ONE loop to fill up the LED array as all of the pixels change.
  for(int i = NUM_LEDS_IN_BOTTOM_PART; i < NUM_LEDS; i++) {
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
  //sine variables 
  waveA = 12;   // high number more speratic 0-150 default 10
  waveB = .5;   // fast pulse a high number 0-5     default 1
  waveC = 2;    // randomizes sine wave pulse higher number fast 0-80 default 4

  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    // Update the LED array with noise at the new location
    fillnoiseT();
    fillnoiseB();
  }

  EVERY_N_SECONDS(1) {            // Change the target palette to a random one every 5 seconds.
     targetPalette = CRGBPalette16(CRGB::Black);
  }

  LEDS.show();                    // Display the LEDs at every loop cycle.  
}


void pattern1_partySparkle()
{
  // sine variables 
  waveA = 25;   // high number more speratic 0-150 default 10
  waveB = .8;   // fast pulse a high number 0-5     default 1
  waveC = 1;    // randomizes sine wave pulse higher number fast 0-80 default 4
 
  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    // Update the LED array with noise at the new location
    fillnoiseT();
    fillnoiseB();
    addGlitter(80);
  }

  EVERY_N_SECONDS(1) {            // Change the target palette to a random one periodically.
    targetPalette = CRGBPalette16(PartyColors_p);
  }
  
  LEDS.show();                    // Display the LEDs at every loop cycle.  
  }


void pattern2_singleTrailz()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy(leds, NUM_LEDS, 30);
  unsigned int pos = beatsin16(30, 0, NUM_LEDS - 1);    // (speed, firstled, lastled)
  if (pos < NUM_LEDS) {           // it should always be in bounds, but defensive programming is a good thing
    leds[pos] += CHSV(gHue, 255, 192);
  }

  LEDS.show();                    // Display the LEDs at every loop cycle.  
}


void pattern3_rainbowSparkle()
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
  addGlitter(20);
  LEDS.show();                    // Display the LEDs at every loop cycle.  
}


void pattern4_randomDots()
{
  leds[randomNumber] += CHSV(gHue, 255, 192);
  fadeToBlackBy(leds, NUM_LEDS, 3);
  LEDS.show();                    // Display the LED's at every loop cycle. 
  // TODO:  Should this be += or just = ?  Incrementing by CRGB::Black probably doesn't do anything.
  leds[randomNumber] += CRGB::Black;
}


void pattern5_lava()
{
  // sine variables 
  waveA = 3;    // high number more speratic 0-150 default 10
  waveB = .3;   // fast pulse a high number 0-5     default 1
  waveC = 1;    // randomizes sine wave pulse higher number fast 0-80 default 4
  
  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    // Update the LED array with noise at the new location
    fillnoiseT();
    fillnoiseB();
  }

  EVERY_N_SECONDS(1) {            // Change the target palette to a random one periodically.
    targetPalette = CRGBPalette16(LavaColors_p);  // max of 4 colors
  }

  LEDS.show();                    // Display the LEDs at every loop cycle.  
}


void pattern6_trailzRandom()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy(leds, NUM_LEDS, 30);
  unsigned int pos = beatsin16(5, 0, NUM_LEDS_IN_BOTTOM_PART - 1);
  if (pos < NUM_LEDS) {           // it should always be in bounds, but defensive programming is a good thing
    leds[pos] += CHSV(gHue, 255, 192);
  }
  leds[randomNumberT] += CHSV(gHue, 255, 192);
  LEDS.show();                    // Display the LEDs at every loop cycle.  
}


void pattern7_lavaTrailzRandom()
{
  // sine variables 
  waveA = 3;    // high number more speratic 0-150 default 10
  waveB = .3;   // fast pulse a high number 0-5     default 1
  waveC = 10;   // randomizes sine wave pulse higher number fast 0-80 default 4
  
  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // Blend towards the target palette
    // Update the LED array with noise at the new location
    fillnoiseB();
  }

  EVERY_N_SECONDS(1) {             // Change the target palette to a random one periodically.
    targetPalette = CRGBPalette16(LavaColors_p);  //max of 4 colors
  }

  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy(leds, NUM_LEDS, 30);
  unsigned int pos = beatsin16(15, NUM_LEDS_IN_BOTTOM_PART, NUM_LEDS - 1);
  if (pos < NUM_LEDS) {           // it should always be in bounds, but defensive programming is a good thing
    leds[pos] += CHSV(gHue, 255, 192);
  }
  leds[randomNumberT] += CHSV(gHue, 255, 192);
  LEDS.show();                    // Display the LEDs at every loop cycle.  
}


void pattern8_multiTrailz()
{
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy(leds, NUM_LEDS, 20);
  byte dothue = 0;
  for(int i = 0; i < 5; i++) {
    unsigned int pos = beatsin16(i + 1.5, 0, NUM_LEDS-1);
    if (pos < NUM_LEDS) {         // it should always be in bounds, but defensive programming is a good thing
      leds[pos] |= CHSV(dothue, 200, 255);
      dothue += 32;
    }
  }
 
  // Show the leds 
  FastLED.show();
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
  FastLED.setBrightness( BRIGHTNESS );

  dist = random16(12345);          // A semi-random number for our noise generator
}


void loop()
{
  static int Active;
  static int State = 1;       // the current pattern being displayed (or 86 if none)
  static String rxBuf;        // holds received data until we have a complete message

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

 //variables for activating if loop for parsing data
  const byte startMarker = '^';
  const byte endMarker = '%';
  boolean startBTdata = false;          // gets set true when we receive a start marker
  String BTdata = "";                   // if not empty, contains a complete message
  // ==== Storing the incoming data into a String variable
  while (HC12.available()) {            // If HC-12 has data
    byte incomingByte = HC12.read();    // get an incoming byte from HC-12
    // Saves the data between the start and end markers.
    if (startBTdata == true) {          // We have received a start marker?
      if (incomingByte != endMarker) {
        rxBuf += char(incomingByte);    // Add the byte to the receive buffer.
      }
      else {                            // We've reached the end marker.
        startBTdata = false;            // Don't save any more characters.
        BTdata = rxBuf;
        rxBuf = "";
      }
    }
    // Checks whether the received message starts with the start marker.
    else if (incomingByte == startMarker) {
      startBTdata = true;               // start saving the message data
    }
  }
 
  // Bluetooth Comm -------------------------------------
  // convert
  // current = BTdata.toInt();

  if      (BTdata == "0") Active = 0;
  else if (BTdata == "1") Active = 1;
  else if (BTdata == "2") Active = 2;
  else if (BTdata == "3") Active = 3;
  else if (BTdata == "4") Active = 4;
  else if (BTdata == "5") Active = 5;
  else if (BTdata == "6") Active = 6;
  else if (BTdata == "")  Active = 86;

  // We're finished with the current message, so clear it.
  BTdata = "";

  // Change the state (pattern) if the message told us to do so.
  if (Active < 80) {
    State = Active;
  }

  Serial.print("BTdata=");
  Serial.print(BTdata);
  Serial.print(" Active=");
  Serial.print(Active);
  Serial.print(" State=");
  Serial.println(State);

  randomNumber = random(0, NUM_LEDS);                         // random number generator for entire tree
  randomNumberB = random(0, NUM_LEDS_IN_BOTTOM_PART);         // random number generator for bottom
  randomNumberT = random(NUM_LEDS_IN_BOTTOM_PART, NUM_LEDS);  // random number generator for top

  // slowly cycle the "base color" through the rainbow
  EVERY_N_MILLISECONDS(20) {
    gHue++;
  }

  switch (State) {
    case 0:
      pattern0_off();
      break;
    case 1:
      pattern1_partySparkle();
      break;
    case 2:
      pattern2_singleTrailz();
      break;
    case 3:
      pattern3_rainbowSparkle();
      break;
    case 4:
      pattern4_randomDots();
      break;
    case 5:
      pattern5_lava();
      break;
    case 6:
      pattern6_trailzRandom();
      break;
    case 7:
      pattern7_lavaTrailzRandom();
      break;
    case 8:
      pattern8_multiTrailz();
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

